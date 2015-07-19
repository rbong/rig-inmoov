/**@file
This is an example \b settings.h for @ref servo.ino.

Every Arduino board on the bot is to be a simple servo controller that accepts
commands from serial input, and so they are all to share the code located in
@ref servo.ino.  Each board needs its own header like this one to define custom
information about the board and each servo on the board. This is such a
settings file and will be documented as an example.
**/

/**
Number of servos to assign.
**/
#define SERVOS 6
/**
Size of the print buffer (must be large enough to hold @ref DUMP_RESPONSE_LEN).
**/
#define BUFSIZE 16

/**
This is the callibration for each servo- its minimum then maximum angle (for
positional servos) or its minimum then maximum speed (for continuous rotation
servos). Its index corresponds with @ref servo.
**/
uint8_t limit [SERVOS] [2] = {
    { 15, 114 }, { 10, 123 }, { 17, 100 },
    { 25, 122 }, { 22, 112 }, { 82, 160 },
};
/**
The default position to set each servo to. Its index corresponds with @ref
servo.
**/
uint8_t default_pos [SERVOS] = { 0, 0, 0, 0, 0, 0 };
/**
Indicates whether to reverse the angles for each servo, for servos that turn
the wrong way. Its index corresponds with @ref servo.
**/
uint8_t reverse [SERVOS] = { 0, 0, 0, 0, 0, 1 };
/**
Pin value to begin assigning pins at. It may be necessary to eventually
implement a similar pin retrieval system to @ref getServoFromID().
**/
uint8_t pin_offset = 2;

enum
{ 
    /// Whether to print debug output to the serial port.
    VERBOSE = 0,
    /// The unique identification for this board.
    BOARD_ID = 181,
};

/**
This function gets array index of a servo using its ID. If the servo does not
belong to this board, it returns -1. It is recommended that boards with complex
ID assignments use a switch statement or a similar solution to save resources.
@see @ref Values
**/
int getServoFromID (uint8_t servo_id)
{
    if (servo_id >= SERVOS || servo_id < 0)
    {
        return -1;
    }
    return servo_id;
}
