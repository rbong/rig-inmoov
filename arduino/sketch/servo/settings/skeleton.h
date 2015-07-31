#define SERVOS 1
#define BUFSIZE 1

uint8_t limit [SERVOS] [2] = { { 0, 180 }, };
uint8_t default_pos [SERVOS] = { 0, };
uint8_t reverse [SERVOS] = { 0, };

enum
{ 
    VERBOSE = 0,
    BOARD_ID = 0,
};

int getServoFromID (uint8_t servo_id)
{
    if (servo_id >= SERVOS || servo_id < 0)
    {
        return -1;
    }
    return servo_id;
}

int getIDFromServo (uint8_t servo_index)
{
    if (servo_index >= SERVOS || servo_index < 0)
    {
        return -1;
    }
    return servo_index;
}

int getServoPinFromIndex (uint8_t servo_index)
{
    if (servo_index >= SERVOS || servo_index < 0)
    {
        return -1;
    }
    return servo_index;
}
