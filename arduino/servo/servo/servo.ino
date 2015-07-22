/**@file
This is the common code for all Arduino servo boards.

The program is written to be a simple module that can pass values to servos.
Each board needs its own settings file. Because of limitations of the
Arduino command line interface, these are included as \b settings.h. The
Makefile manages which configuration is currently located in the same directory
as this file and named \b settings.h. A better build system in the future could
use the tools \b avrdude or \b ino.

See @ref rhand.h for an example of a settings header, and @ref Arduino for
information servo IDs and other relevant values.
**/

#ifndef SIMULATION
// allow simulations to define their own servo libraries
#include <Servo.h>
// assume that simulation has its own callibration
#include "settings.h"
#else
// prototyping is only required if not using arduino
#include <servo.h>
#endif

// Arduino's servo library
#include <stdio.h>
#include <stdint.h>

/**
The representation of the servos.  Used to keep track of pin assignments and
send output.
@see getServoFromID(), Arduino
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

char buf [BUFSIZE]; /// The print buffer.

enum
{
    MIN_LIM = 0, /// Index for the @ref limit variables.
    MAX_LIM = 1, /// Index for the @ref limit variables.
    CANCEL_SIGNAL = 255, /// Incoming signal commanding termination of the current loop.
    WAIT_RESPONSE = 254, /// Outgoing signal indicating that we are waiting for input.
    DUMP_SIGNAL = 253, /// Incoming signal indicating that we are to print information to serial.
    DUMP_START_RESPONSE = 252, /// Beginning of our serial response to the dump signal.
    DUMP_END_RESPONSE = 251, /// End of our serial response to the dump signal.
    DUMP_RESPONSE_LEN = SERVOS * 2 + 4, /// The length of the response to the dump signal.
};

/**
The \b setup function is called at the beginning of the program. In it, we call
@ref setAdjustedAngles for each servo and assign a pin to each @ref servo.
**/
void setup ()
{
    int angle, pin;

    Serial.begin(9600);

    serialPrint ("starting...\n");
    for (uint8_t servo_num = 0; servo_num < SERVOS; servo_num++)
    {
        serialPrintIntPretty ("calibrating servo: ", servo_num, "\n");
        setAdjustedAngles (servo_num);

// simulations don't have pins
#ifndef SIMULATION
        serialPrintIntPretty ("assinging servo: ", servo_num, "\n");
        pin = getServoPinFromNum (servo_num);
        if (pin < 0)
        {
            serialPrint ("invalid servo index.\n");
        }
        else
        {
            serialPrintIntPretty ("pin: ", pin, "\n");
            servo [servo_num].attach (pin);
            pinMode (pin, OUTPUT);
        }
#endif

        serialPrintIntPretty ("default: ", default_pos [servo_num], "\n");
        setServoFromNum (servo_num, default_pos [servo_num]);
    }
}

/**
The \b loop function is called continually until the program exits.  It
performs actions based on @ref Arduino. Recieves two unsigned 8-bit
integers, a servo id and a servo angle, then calls @ref setServoFromID().  If
the @ref DUMP_SIGNAL is recieved at any time, calls @ref dump() and continues.
If the @ref CANCEL_SIGNAL is recieved, it does nothing and continues. If, at
the beginning of the function, there is no pending input, it transmits @ref
WAIT_RESPONSE, then does nothing until input is available.
**/
void loop ()
{
    static int servo_id, servo_angle;

    // indicate that we are waiting for input
    if (! Serial.available ())
    {
        serialPrint ("WAIT\n");
        Serial.write (WAIT_RESPONSE);
    }

    servo_id = serialGetByte ();
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
    serialPrintIntPretty ("recieved servo id: ", servo_id, "\n");

    servo_angle = serialGetByte ();
    if (servo_angle < 0)
    {
        return;
    }
    if (servo_angle == DUMP_SIGNAL)
    {
        dump ();
        return;
    }
    serialPrintIntPretty ("recieved servo value: ", servo_angle, "\n");

    setServoFromID (servo_id, servo_angle);
}

/**
Prints a string to the serial port if @ref VERBOSE is enabled.
**/
void serialPrint (const char* s)
{
    if (VERBOSE)
    {
        Serial.print (s);
    }
}

/**
Prints an integer to the serial port as a string if @ref VERBOSE is enabled.
**/
void serialPrintInt (uint8_t i)
{
    if (VERBOSE)
    {
        // this writes an integer to the string @ref buf
        snprintf (buf, BUFSIZE, "%d", i);
        Serial.write (buf);
    }
}

/**
Prints an integer as a string surrounded by two strings to the serial port if
@ref VERBOSE is enabled.
**/
void serialPrintIntPretty (const char* pre, uint8_t i, const char* post)
{
    if (VERBOSE)
    {
        Serial.write (pre);
        serialPrintInt (i);
        Serial.write (post);
    }
}

/**
Retrieves a byte from the serial port.
@return If a normal value is recieved, returns that value as an integer.
If @ref CANCEL_SIGNAL is recieved, returns -1.
**/
int serialGetByte ()
{
    int i;

    serialWait ();
    i = Serial.read ();
    if (i == CANCEL_SIGNAL)
    {
        serialPrint ("CANCEL_SIGNAL\n");
        return -1;
    }

    return i;
}

/**
Calculates modified angles for a servo. The function scales angles from 0-180
to some minimum and maximum callibration, then stores them for later retrieval.
@param servo_num The servo to calculate angles for.
@see getAdjustedAngle(), adjusted_angles
**/
void setAdjustedAngles (uint8_t servo_num)
{
    // doubles used for conversion
    double dangle, dmin, dmax;

    for (uint8_t angle = 0; angle <= 180; angle++)
    {
        // arduino has its own map function, but it's broken on some systems
        // conversion to double avoids integer rounding errors
        dangle = angle;
        dmin = limit [servo_num] [MIN_LIM];
        dmax = limit [servo_num] [MAX_LIM];
        dangle = dmin + ((dangle * (dmax - dmin)) / 180);
        adjusted_angles [servo_num] [angle] = dangle;
    }
}

/**
Retrieves modified angles for a servo.
@return The adjusted angle.
@param servo_num The servo number to use.
@param servo_angle The angle to retrieve.
@see setAdjustedAngles(), adjusted_angles
**/
uint8_t getAdjustedAngle (uint8_t servo_num, uint8_t servo_angle)
{
    // the bitmask was set before we were using uint8_t
    // todo- test and remove bitmask
    return adjusted_angles [servo_num] [servo_angle] & 0xff;
}

/**
Writes an angle to a pin associated with a servo number. Servos are controlled
by sending varying pulse widths over their signal wire. The adjusted angle will
be sent to the servo. Updates @ref current_pos with the non-adjusted angle.  If
the servo number is invalid, the function does nothing and exits, but if the
angle is out of range, it is changed to a valid value.
@param servo_num The servo to write to.
@param servo_angle The angle to write after adjusting. It may be reversed.
@see getAdjustedAngle(), reverse
**/
void setServoFromNum (uint8_t servo_num, uint8_t servo_angle)
{
    if (servo_num >= SERVOS || servo_num < 0)
    {
        serialPrint ("servo number out of range.\n");
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

    current_pos [servo_num] = servo_angle;

    if (reverse [servo_num])
    {
        servo_angle = 180 - servo_angle;
        serialPrintIntPretty ("reversing angle, now is: ", servo_angle, "\n");
    }

    servo_angle = getAdjustedAngle (servo_num, servo_angle);
    serialPrintIntPretty ("readjusted value: ", servo_angle, "\n");

    servo [servo_num].write (servo_angle);
}

/**
Writes an angle to a pin associated with a servo ID.
@see setServoFromNum(), getServoFromID()
**/
void setServoFromID (int servo_id, uint8_t servo_angle)
{
    int servo_num;

    servo_num = getServoFromID (servo_id);
    if (servo_num < 0)
    {
        serialPrint ("servo number out of range.\n");
        return;
    }

    setServoFromNum (servo_num, servo_angle);
}

/**
Writes various information about the board to the serial port. Triggered on
reception of @ref DUMP_RESPONSE_LEN. Transmits the ID of the board, the number
of servos, each servo ID, and the current position of the servos.
@see @ref Arduino, DUMP_START_RESPONSE, DUMP_END_RESPONSE, current_pos, getIDFromServo()
**/
void dump ()
{
    int i = 0;

    // if you change this function, please verify DUMP_RESPONSE_LEN
    if (BUFSIZE < DUMP_RESPONSE_LEN)
    {
        return;
    }

    buf [i++] = DUMP_START_RESPONSE;
    buf [i++] = BOARD_ID;
    buf [i++] = SERVOS;
    for (int servo_num = 0; servo_num < SERVOS; servo_num++)
    {
        buf [i++] = getIDFromServo (servo_num);
        buf [i++] = current_pos [servo_num];
    }
    buf [i++] = DUMP_END_RESPONSE;

    Serial.write (buf, i);
}

/**
Waits for serial input to become available then returns.
**/
void serialWait ()
{
    while (!Serial.available ());
}
