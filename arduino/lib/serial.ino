/**@file
This is the serial communication shared code.

In the near future, this will be shared between Arduino programs for common
serial communication tasks.
**/

#include <SoftwareSerial.h>

SoftwareSerial cmd_serial (CMD_RX_PIN, CMD_TX_PIN);

void serialSetup ()
{
    Serial.begin (DEBUG_SERIAL_BAUDRATE);

    cmd_serial.begin (CMD_SERIAL_BAUDRATE);

    cmd_serial.listen ();
}

bool serialCmdAvailable ()
{
    return cmd_serial.available ();
}

void serialCmdWait ()
{
    while (!cmd_serial.available ());
}

void serialCmdWrite (uint8_t c)
{
    cmd_serial.write (c);
}

void serialDebugWrite (uint8_t c)
{
    Serial.write (c);
}

uint8_t serialCmdRead ()
{
    return cmd_serial.read ();
}

int serialCmdGetByte ()
{
    int i;

    serialCmdWait ();
    i = serialCmdRead ();
    if (i == CANCEL_SIGNAL)
    {
        serialDebugPrint ("CANCEL_SIGNAL\n");
        return -1;
    }

    return i;
}

uint8_t serialDebugRead ()
{
    return Serial.read ();
}

int serialDebugAvailable ()
{
    return Serial.available ();
}

void serialDebugPrint (const char* s)
{
    if (VERBOSE)
    {
        Serial.print (s);
    }
}

void serialDebugPrintInt (int i)
{
    if (VERBOSE)
    {
        Serial.print (i, DEC);
    }
}

void serialDebugPrintIntPretty (const char* pre, int i, const char* post)
{
    if (VERBOSE)
    {
        Serial.print (pre);
        serialDebugPrintInt (i);
        Serial.print (post);
    }
}
