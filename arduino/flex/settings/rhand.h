#define FLEXS 5
#define BUFSIZE 32

int limit [FLEXS] [2] = {
    { 540, 645 }, { 494, 659 }, { 545, 693 }, { 549, 687 }, { 533, 699 },
};
uint8_t reverse [FLEXS] = { 1, 1, 1, 1, 1, };
uint8_t RX_pin = 10;
uint8_t TX_pin = 11;
uint8_t pin_offset = 14;
// used for IDs
uint8_t flex_offset = 6;

enum
{
    VERBOSE = 0,
    NOISE_CONTROL = 1,
    BOARD_ID = 182,
    SERIAL_BAUDRATE = 9600,
    SOFT_SERIAL_BAUDRATE = 9600,
    DELAY_MS = 25,
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
