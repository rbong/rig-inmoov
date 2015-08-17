#define SERVOS 6
#define BUFSIZE 32

uint8_t limit [SERVOS] [2] = {
    { 15, 55 }, { 0, 180 }, { 0, 180 },
    { 0, 180 }, { 0, 180 }, { 0, 180 },
};

uint8_t default_pos [SERVOS] = { 0, 90, 90, 90, 90, 90 };
uint8_t reverse [SERVOS] = { 1, 0, 0, 0, 0, 0 };
uint8_t pin_offset = 2;

enum
{
    VERBOSE = 0,
    BOARD_ID = 181,
    DEBUG_RX_PIN = 1,
    DEBUG_TX_PIN = 0,
    CMD_RX_PIN = 1,
    CMD_TX_PIN = 0,
    SERIAL_BAUDRATE = 9600,
    DEBUG_SERIAL_BAUDRATE = 9600,
};

int getServoIndexFromID (uint8_t servo_id)
{
    if (servo_id >= SERVOS || servo_id < 0)
    {
        return -1;
    }
    return servo_id;
}

int getServoIDFromIndex (uint8_t servo_index)
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
    return servo_index + pin_offset;
}
