/**@file
This is the common code for all Arduino servo boards.

The program is written to be a simple module that can pass values to servos.
Each board needs its own settings file. Because of limitations of the
Arduino command line interface, these are included as \b settings.h. The
Makefile manages which configuration is currently located in the same directory
as this file and named \b settings.h. Other Arduino boards in the project, such
as flex sensor boards, use an identical folder structure and include process.

See @ref servo_rhand.h for an example of a settings header, and @ref Arduino for
information servo IDs and other relevant values.
**/

#include <Servo.h>
#include "settings.h"

#include <stdint.h>

/**
The representation of the servos.  Used to keep track of pin assignments and
send output.
@see getServoIndexFromID(), Arduino
**/
Servo servo [SERVOS];

/**
The adjusted angles of the servos given the @ref limit of each servo.
A table calculated at initialization for optimization purposes.
Its index corresponds to @ref servo.
The program uses the convention "angles", but continuous rotation servos are
technically compatible.
@see setAdjustedAngles (), getAdjustedAngle ()
**/
uint8_t adjusted_angles [SERVOS] [181];
/**
The current position of each servo. These are updated whenever a servos value
is changed and retrieved upon @ref DUMP_SIGNAL.
**/
uint8_t current_pos [SERVOS] = { 0 };

/// The print buffer.
char buf [BUFSIZE];

enum
{
    /// Index for the @ref limit variables.
    MIN_LIM = 0,
    /// Index for the @ref limit variables.
    MAX_LIM = 1,
    /// Incoming signal commanding termination of the current loop.
    CANCEL_SIGNAL = 255,
    /// Outgoing signal indicating that we are waiting for input.
    WAIT_RESPONSE = 254,
    /// Incoming signal indicating that we are to print information to serial.
    DUMP_SIGNAL = 253,
    /// Beginning of some multi-byte response.
    START_RESPONSE = 252,
    /// End of some response.
    END_RESPONSE = 251,
    /// The length of the response to the dump signal.
    DUMP_RESPONSE_LEN = SERVOS * 2 + 4,
};

/**
The \b setup function is called at the beginning of the program. In it, we call
@ref setAdjustedAngles for each servo and assign a pin to each @ref servo.
**/
void setup ()
{
    int angle, pin;

    serialSetup ();

    serialDebugPrint ("starting...\n");
    for (uint8_t servo_index = 0; servo_index < SERVOS; servo_index++)
    {
        serialDebugPrintIntPretty ("calibrating servo: ", servo_index, "\n");
        setAdjustedAngles (servo_index);

// simulations don't have pins
        serialDebugPrintIntPretty ("assinging servo: ", servo_index, "\n");
        pin = getServoPinFromIndex (servo_index);
        if (pin < 0)
        {
            serialDebugPrint ("invalid servo index.\n");
        }
        else
        {
            serialDebugPrintIntPretty ("pin: ", pin, "\n");
            servo [servo_index].attach (pin);
            pinMode (pin, OUTPUT);
        }

        serialDebugPrintIntPretty ("default: ", default_pos [servo_index], "\n");
        setServoFromIndex (servo_index, default_pos [servo_index]);
    }
}

/**
The \b loop function is called continually until the program exits. It
performs actions based on @ref Arduino. Recieves two unsigned 8-bit
integers, a servo id and a servo angle, then calls @ref setServoFromID(). If
the @ref DUMP_SIGNAL is recieved at any time, calls @ref dump() and continues.
If the @ref CANCEL_SIGNAL is recieved, it does nothing and continues. If, at
the beginning of the function, there is no pending input, it transmits @ref
WAIT_RESPONSE, then does nothing until input is available.
**/
void loop ()
{
    static int servo_id, servo_angle;

    // indicate that we are waiting for input
    if (! serialCmdAvailable ())
    {
        serialDebugPrint ("WAIT\n");
        serialCmdWrite (WAIT_RESPONSE);
    }

    servo_id = serialCmdGetByte ();
    // WAIT_SIGNAL recieved
    if (servo_id < 0)
    {
        return;
    }
    if (servo_id == DUMP_SIGNAL)
    {
        dump ();
        return;
    }
    serialDebugPrintIntPretty ("recieved servo id: ", servo_id, "\n");

    servo_angle = serialCmdGetByte ();
    if (servo_angle < 0)
    {
        return;
    }
    if (servo_angle == DUMP_SIGNAL)
    {
        dump ();
        return;
    }
    serialDebugPrintIntPretty ("recieved servo value: ", servo_angle, "\n");

    setServoFromID (servo_id, servo_angle);
}

/**
Calculates modified angles for a servo. The function scales angles from 0-180
to some minimum and maximum callibration, then stores them for later retrieval.
@param servo_index The servo to calculate angles for.
@see getAdjustedAngle(), adjusted_angles
**/
void setAdjustedAngles (uint8_t servo_index)
{
    // doubles used for conversion
    double dangle, dmin, dmax;

    for (uint8_t angle = 0; angle <= 180; angle++)
    {
        // arduino has its own map function, but it's broken on some systems
        // conversion to double avoids integer rounding errors
        dangle = angle;
        dmin = limit [servo_index] [MIN_LIM];
        dmax = limit [servo_index] [MAX_LIM];
        dangle = dmin + ((dangle * (dmax - dmin)) / 180);
        adjusted_angles [servo_index] [angle] = dangle;
    }
}

/**
Retrieves modified angles for a servo.
@return The adjusted angle.
@param servo_index The servo index to use.
@param servo_angle The angle to retrieve.
@see setAdjustedAngles(), adjusted_angles
**/
uint8_t getAdjustedAngle (uint8_t servo_index, uint8_t servo_angle)
{
    // the bitmask was set before we were using uint8_t
    // todo- test and remove bitmask
    return adjusted_angles [servo_index] [servo_angle] & 0xff;
}

/**
Writes an angle to a pin associated with a servo index. Servos are controlled
by sending varying pulse widths over their signal wire. The adjusted angle will
be sent to the servo. Updates @ref current_pos with the non-adjusted angle.  If
the servo index is invalid, the function does nothing and exits, but if the
angle is out of range, it is changed to a valid value.
@param servo_index The servo to write to.
@param servo_angle The angle to write after adjusting. It may be reversed.
@see getAdjustedAngle(), reverse
**/
void setServoFromIndex (uint8_t servo_index, uint8_t servo_angle)
{
    if (servo_index >= SERVOS || servo_index < 0)
    {
        serialDebugPrint ("servo index out of range.\n");
        return;
    }

    if (servo_angle > 180)
    {
        servo_angle = 180;
    }
    if (servo_angle < 0)
    {
        servo_angle = 0;
    }

    current_pos [servo_index] = servo_angle;

    if (reverse [servo_index])
    {
        servo_angle = 180 - servo_angle;
        serialDebugPrintIntPretty ("reversing angle, now is: ", servo_angle, "\n");
    }

    servo_angle = getAdjustedAngle (servo_index, servo_angle);
    serialDebugPrintIntPretty ("readjusted value: ", servo_angle, "\n");

    servo [servo_index].write (servo_angle);
}

/**
Writes an angle to a pin associated with a servo ID.
@see setServoFromIndex(), getServoIndexFromID()
**/
void setServoFromID (int servo_id, uint8_t servo_angle)
{
    int servo_index;

    servo_index = getServoIndexFromID (servo_id);
    if (servo_index < 0)
    {
        serialDebugPrint ("servo ID not valid.\n");
        return;
    }

    setServoFromIndex (servo_index, servo_angle);
}

/**
Writes various information about the board to the serial port. Triggered on
reception of @ref DUMP_RESPONSE_LEN. Transmits the ID of the board, the number
of servos, each servo ID, and the current position of the servos.
@see @ref Arduino, START_RESPONSE, END_RESPONSE, current_pos,
getServoIDFromIndex()
**/
void dump ()
{
    int i = 0;

    // if you change this function, please verify DUMP_RESPONSE_LEN
    if (BUFSIZE < DUMP_RESPONSE_LEN)
    {
        return;
    }

    buf [i++] = START_RESPONSE;
    buf [i++] = BOARD_ID;
    buf [i++] = SERVOS;
    for (int servo_index = 0; servo_index < SERVOS; servo_index++)
    {
        buf [i++] = getServoIDFromIndex (servo_index);
        buf [i++] = current_pos [servo_index];
    }
    buf [i++] = END_RESPONSE;

    Serial.write (buf, i);
}

/**
Waits for serial input to become available then returns.
**/
void serialWait ()
{
    while (!Serial.available ());
}
