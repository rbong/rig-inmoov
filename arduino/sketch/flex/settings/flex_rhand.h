#include <stdint.h>

#define FLEXS 5
#define BUFSIZE 32

int limit [FLEXS] [2] = {
    { 579, 713 }, { 519, 743 }, { 588, 793 }, { 610, 811 }, { 591, 806 },
};
uint8_t reverse [FLEXS] = { 1, 1, 1, 1, 1, };
uint8_t pin_offset = 14;
// used for IDs
uint8_t flex_offset = 6;

enum
{
    VERBOSE = 1,
    NOISE_CONTROL = 0,
    BOARD_ID = 182,
    SERIAL_BAUDRATE = 9600,
    SOFT_SERIAL_BAUDRATE = 9600,
    DELAY_MS = 0,
    DEBUG_RX_PIN = 0,
    DEBUG_TX_PIN = 1,
    CMD_RX_PIN = 10,
    CMD_TX_PIN = 11,
    DEBUG_SERIAL_BAUDRATE = 9600,
    CMD_SERIAL_BAUDRATE = 9600,
};

int getFlexIDFromIndex (uint8_t flex_index)
{
    if (flex_index > FLEXS)
    {
        return -1;
    }
    return flex_index + flex_offset;
}

int getFlexPinFromIndex (uint8_t flex_index)
{
    if (flex_index > FLEXS)
    {
        return -1;
    }
    return flex_index + pin_offset;
}

int getServoIDFromFlexIndex (uint8_t flex_index)
{
    if (flex_index > FLEXS)
    {
        return -1;
    }
    return flex_index + 1;
}

int getServoIndexFromID (uint8_t servo_id)
{
    if (servo_id >= 6 || servo_id < 0)
    {
        return -1;
    }
    return servo_id;
}
