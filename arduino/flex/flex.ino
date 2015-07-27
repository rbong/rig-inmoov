#include <SoftwareSerial.h>

#include "settings.h"

SoftwareSerial soft_serial (RX_pin, TX_pin);
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
    int angle, pin;

    Serial.begin (SERIAL_BAUDRATE);
    soft_serial.begin (SOFT_SERIAL_BAUDRATE);
}

void loop ()
{
    static int flex_amount, servo_id, servo_angle;

    if (Serial.available ())
    {
        switch (Serial.read ())
        {
            case MIN_SIGNAL:
                for (int flex_index = 0; flex_index < FLEXS; flex_index++)
                {
                    serialPrintIntPretty ("Flex: ", getFlexIDFromIndex (flex_index), "\n");
                    limit [flex_index] [MIN_LIM] = analogGetInt (flex_index);
                    serialPrintIntPretty ("Min: ", limit [flex_index] [MIN_LIM], "\n");
                }
            case MAX_SIGNAL:
                for (int flex_index = 0; flex_index < FLEXS; flex_index++)
                {
                    serialPrintIntPretty ("Flex: ", getFlexIDFromIndex (flex_index), "\n");
                    limit [flex_index] [MAX_LIM] = analogGetInt (flex_index);
                    serialPrintIntPretty ("Max: ", limit [flex_index] [MIN_LIM], "\n");
                }
        }
    }

    soft_serial.write (CANCEL_SIGNAL);
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

        serialPrintIntPretty ("flex sensor: ", flex_index, "\n");
        serialPrintIntPretty ("flex pin: ", getFlexPinFromIndex (flex_index), "\n");
        serialPrintIntPretty ("flex amount: ", flex_amount, "\n");
        serialPrintIntPretty ("servo: ", servo_id, "\n");
        serialPrintIntPretty ("servo angle: ", servo_angle, "\n");
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

    soft_serial.write (servo_id);
    soft_serial.write (servo_angle);
}

void serialPrint (const char* s)
{
    if (VERBOSE)
    {
        Serial.print (s);
    }
}

void serialPrintInt (int i)
{
    if (VERBOSE)
    {
        Serial.print (i, DEC);
    }
}

void serialPrintIntPretty (const char* pre, int i, const char* post)
{
    if (VERBOSE)
    {
        Serial.write (pre);
        serialPrintInt (i);
        Serial.write (post);
    }
}
