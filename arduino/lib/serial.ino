/**@file
This is the serial communication shared code.

Serial communication is broken up into two serial ports; a command port, and a
debug port. On boards that only recieve commands (servo controllers), the
command port is the port on which the board recieves commands and prints
responses, and the debug port is the port on which it prints information if the
@ref VERBOSE constant is true.

On boards that control other boards (flex sensors), the command port is the
port on which it transmits commands to and listens for responses from the child
board, and the debug port is the port on which it recieves any commands and
prints information.

To use the USB serial port on an Arduino board, simply set the board's
corresponding RX and TX pins for output (usually 0 and 1).
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
