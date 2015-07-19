#define SERVOS 1
#define BUFSIZE 1

uint8_t limit [SERVOS] [2] = {
    { 0, 0 },
};
uint8_t default_pos [SERVOS] = { 0 };
uint8_t reverse [SERVOS] = { 0 };
uint8_t pin_offset = 2;

enum
{ 
    VERBOSE = 0,
    BOARD_ID = 181,
};

int getServoFromID (uint8_t servo_id)
{
    if (servo_id >= SERVOS || servo_id < 0)
    {
        return -1;
    }
    return servo_id;
}
