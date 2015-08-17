/**@file
This is the common code for all Arduino servo boards.

The program is written to be a simple module that can pass values to servos.
Each board needs its own settings file. Because of limitations of the
Arduino command line interface, these are included as \b settings.h. The
Makefile manages which configuration is currently located in the same directory
as this file and named \b settings.h. A better build system in the future could
use the tools \b avrdude or \b ino. Other Arduino boards in the project, such
as flex sensor boards, use an identical folder structure and include process.

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

#include <stdint.h>

/**
The representation of the servos.  Used to keep track of pin assignments and
send output.
@see getServoIndexFromID(), Arduino
**/
Servo servo [SERVOS];

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
    START_RESPONSE = 252, /// Beginning of some multi-byte response.
    END_RESPONSE = 251, /// End of some response.
    DUMP_RESPONSE_LEN = SERVOS * 2 + 4, /// The length of the response to the dump signal.
};

/**
The \b setup function is called at the beginning of the program. In it, we
assign a pin to each @ref servo.
**/
void setup ()
{
    int angle, pin;

    Serial.begin(SERIAL_BAUDRATE);

    serialPrint ("starting...\n");
    for (uint8_t servo_index = 0; servo_index < SERVOS; servo_index++)
    {
// simulations don't have pins
#ifndef SIMULATION
        serialPrintIntPretty ("assinging servo: ", servo_index, "\n");
        pin = getServoPinFromIndex (servo_index);
        if (pin < 0)
        {
            serialPrint ("invalid servo index.\n");
        }
        else
        {
            serialPrintIntPretty ("pin: ", pin, "\n");
            servo [servo_index].attach (pin);
            pinMode (pin, OUTPUT);
        }
#endif

        serialPrintIntPretty ("default: ", default_pos [servo_index], "\n");
        setServoFromIndex (servo_index, default_pos [servo_index]);
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
        Serial.print (i, DEC);
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
Writes an angle to a pin associated with a servo index. Servos are controlled
by sending varying pulse widths over their signal wire. The adjusted angle will
be sent to the servo. Updates @ref current_pos with the non-adjusted angle.  If
the servo index is invalid, the function does nothing and exits, but if the
angle is out of range, it is changed to a valid value.
@param servo_index The servo to write to.
@param servo_angle The angle to write after adjusting. It may be reversed.
@see reverse
**/
void setServoFromIndex (uint8_t servo_index, uint8_t servo_angle)
{
    if (servo_index >= SERVOS || servo_index < 0)
    {
        serialPrint ("servo index out of range.\n");
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
        servo_angle = map (servo_angle, 180, 0,
            limit [servo_index] [MIN_LIM], limit [servo_index] [MAX_LIM]);
    }
    else
    {
        servo_angle = map (servo_angle, 0, 180,
            limit [servo_index] [MIN_LIM], limit [servo_index] [MAX_LIM]);
    }

    serialPrintIntPretty ("readjusted value: ", servo_angle, "\n");

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
        serialPrint ("servo ID not valid.\n");
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
