#include "settings.h"

int current_pos [6] = { 0 };

enum
{
    CANCEL_SIGNAL = 255,
    MIN_SIGNAL = 250,
    MAX_SIGNAL = 249,
    START_RESPONSE = 252,
    END_RESPONSE = 251,
    CALIB_RESPONSE_LEN = FLEXS * 2,
    MIN_LIM = 0,
    MAX_LIM = 1,
};

void setup ()
{
    serialSetup ();
}

void loop ()
{
    static int flex_amount, servo_id, servo_angle;

    if (serialCmdAvailable ())
    {
        switch (serialCmdRead ())
        {
            case MIN_SIGNAL:
                for (int flex_index = 0; flex_index < FLEXS; flex_index++)
                {
                    serialDebugPrintIntPretty ("Flex: ", getFlexIDFromIndex (flex_index), "\n");
                    limit [flex_index] [MIN_LIM] = analogGetInt (flex_index);
                    serialDebugPrintIntPretty ("Min: ", limit [flex_index] [MIN_LIM], "\n");
                }
                break;
            case MAX_SIGNAL:
                for (int flex_index = 0; flex_index < FLEXS; flex_index++)
                {
                    serialDebugPrintIntPretty ("Flex: ", getFlexIDFromIndex (flex_index), "\n");
                    limit [flex_index] [MAX_LIM] = analogGetInt (flex_index);
                    serialDebugPrintIntPretty ("Max: ", limit [flex_index] [MIN_LIM], "\n");
                }
                break;
            default:
                serialDebugPrint ("Unknown signal.");
                break;
        }
    }

    serialCmdWrite (CANCEL_SIGNAL);
    for (int flex_index = 0; flex_index < FLEXS; flex_index++)
    {
        flex_amount = analogGetInt (flex_index);
        if (flex_amount < 0)
        {
            continue;
        }

        servo_angle = getAdjustedFlex (flex_index, flex_amount);
        if (servo_angle < 0)
        {
            continue;
        }

        servo_id = getServoIDFromFlexIndex (flex_index);
        if (servo_id < 0)
        {
            continue;
        }

        softSerialServoCmd (servo_id, servo_angle);

        serialDebugPrintIntPretty ("flex sensor: ", flex_index, "\n");
        serialDebugPrintIntPretty ("flex pin: ", getFlexPinFromIndex (flex_index), "\n");
        serialDebugPrintIntPretty ("flex amount: ", flex_amount, "\n");
        serialDebugPrintIntPretty ("servo: ", servo_id, "\n");
        serialDebugPrintIntPretty ("servo angle: ", servo_angle, "\n");
    }
    delay (DELAY_MS);
}

int analogGetInt (uint8_t flex_index)
{
    int pin, flex_amount;

    pin = getFlexPinFromIndex (flex_index);
    if (pin < 0)
    {
        return -1;
    }

    flex_amount = analogRead (pin);
    return flex_amount;
}

int getAdjustedFlex (uint8_t flex_index, int flex_amount)
{
    int flex_min, flex_max;

    if ((flex_min = getFlexMin (flex_index)) < 0)
    {
        return -1;
    }

    if ((flex_max = getFlexMax (flex_index)) < 0)
    {
        return -1;
    }

    if (flex_amount > flex_max)
    {
        flex_amount = flex_max;
    }
    else if (flex_amount < flex_min)
    {
        flex_amount = flex_min;
    }

    if (reverse [flex_index])
    {
        return map (flex_amount, flex_min, flex_max, 0, 180);
    }
    return map (flex_amount, flex_min, flex_max, 180, 0);
}

int getFlexMax (uint8_t flex_index)
{
    if (flex_index > FLEXS)
    {
        return -1;
    }

    return limit [flex_index] [MAX_LIM];
}

int getFlexMin (uint8_t flex_index)
{
    if (flex_index > FLEXS)
    {
        return -1;
    }

    return limit [flex_index] [MIN_LIM];
}

void softSerialServoCmd (uint8_t servo_id, int servo_angle)
{
    int servo_index = getServoIndexFromID (servo_id), difference;

    if (servo_index < 0)
    {
        return;
    }

    if (NOISE_CONTROL)
    {
        difference = current_pos [servo_index] - servo_angle;
        if ((difference * difference) < 2) 
        {
            return;
        }

        current_pos [servo_index] = servo_angle;
    }

    serialCmdWrite (servo_id);
    serialCmdWrite (servo_angle);
}
