#include <SoftwareSerial.h>

SoftwareSerial debug_serial (DEBUG_RX_PIN, DEBUG_TX_PIN);
SoftwareSerial cmd_serial (CMD_RX_PIN, CMD_TX_PIN);

void serialSetup ()
{
    if (VERBOSE)
    {
        debug_serial.begin (DEBUG_SERIAL_BAUDRATE);
    }

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

void serialDebugPrint (const char* s)
{
    if (VERBOSE)
    {
        debug_serial.print (s);
    }
}

void serialDebugPrintInt (int i)
{
    if (VERBOSE)
    {
        debug_serial.print (i, DEC);
    }
}

void serialDebugPrintIntPretty (const char* pre, int i, const char* post)
{
    if (VERBOSE)
    {
        debug_serial.print (pre);
        serialDebugPrintInt (i);
        debug_serial.print (post);
    }
}
