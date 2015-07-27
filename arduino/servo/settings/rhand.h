/**@file
This is an example \b settings.h for @ref servo.ino.

Every Arduino board on the bot is to be a simple servo controller that accepts
commands from serial input, and so they are all to share the code located in
@ref servo.ino. Each board needs its own header like this one to define custom
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
#define BUFSIZE 32

/**
This is the callibration for each servo- its minimum then maximum angle (for
positional servos) or its minimum then maximum speed (for continuous rotation
servos). Its index corresponds with @ref servo.
**/
uint8_t limit [SERVOS] [2] = {
    { 15, 55 }, { 45, 120 }, { 35, 110 },
    { 15, 105 }, { 15, 120 }, { 35, 140 },
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
uint8_t reverse [SERVOS] = { 1, 0, 0, 0, 0, 1 };
/**
Pin value to begin assigning pins at. Unique to this settings file.
@see getServoPinFromNum()
**/
uint8_t pin_offset = 2;

enum
{
    /// Whether to print debug output to the serial port.
    VERBOSE = 0,
    /// The unique identification for this board.
    BOARD_ID = 181,
    /// The baudrate to use for this board.
    SERIAL_BAUDRATE = 9600,
};

/**
This function gets array index of a servo using its ID. If the servo does not
belong to this board, it returns -1. It is recommended that boards with complex
ID assignments use a switch statement or a similar solution to save resources.
This function must correspond with getServoIDFromIndex().
@see @ref Values
**/
int getServoIndexFromID (uint8_t servo_id)
{
    if (servo_id >= SERVOS || servo_id < 0)
    {
        return -1;
    }
    return servo_id;
}

/**
This function gets servo ID using its array index. If the servo index is out of
bounds, it returns -1. This must correspond with getServoIndexFromID().
**/
int getServoIDFromIndex (uint8_t servo_index)
{
    if (servo_index >= SERVOS || servo_index < 0)
    {
        return -1;
    }
    return servo_index;
}

/**
This function returns the desired pin of a servo given its index number,
allows complex pin assignment. If the servo idex is not valid, returns -1.
**/
int getServoPinFromIndex (uint8_t servo_index)
{
    if (servo_index >= SERVOS || servo_index < 0)
    {
        return -1;
    }
    return servo_index + pin_offset;
}
