"""! Documentation for the servo_demo.py script.
An example script for controlling the right hand.
This script requires python with a library called pyserial.
After instally python 3.4, pyserial can be installed by calling

@code{.sh}
pip install pyserial
@endcode

This may vary from system to system.
See the <a href="http://www.python.org/">python</a> and
<a href="http://pyserial.sourceforge.net/">pyserial</a> websites for help.

To run the script, execute

@code{.sh}
python -i servo_demo.py
@endcode

or in python

@code{.py}
import servo_demo
@endcode

This will allow you to call the functions from a command interpreter.
It is recommended you experiment with all functions and view the source to
understand how you might produce commands. Also try demo() while connected.

The movements included are quickly written and for demo purposes only.

@see servo.ino Values

@var ser
The serial connection for input and output. Must be initialized.
@see connect()
import pdb; pdb.set_trace()  # XXX BREAKPOINT
@var RHAND_ID
The identification byte of the right hand board.
"""

import serial
from time import sleep

# Serial signals and responses
CANCEL_SIGNAL = 255
DUMP_SIGNAL = 253
START_RESPONSE = 252
END_RESPONSE = 251
RHAND_ID = 181

global ser
ser = serial.Serial ()

def connect (port):
    """! Sets the global @ref ser variable."""
    global ser
    ser = serial.Serial (port, 9600, timeout=1)

def sweep (initial, servos, starts, ends, steps):
    """! Produces complex command chains.

    The output returned from the function is formatted in such a way that the
    commands can be transmitted to the serial port at variable speeds.

    The parameters must all be lists of the same length whose indexes
    correspond with \b servos.

    @param initial  An array of initial commands to append to.
    @param servos   The servos to control.
    @param starts   The current positions of the servos.
    @param ends     The desired end positions.
    @param steps    The values to use to decrement/increment.
                    Converted to absolute value.

    @returns Upon success, returns the generated list of commands.
    The commands are surrounded by the @ref CANCEL_SIGNAL for syncing.
    The servos will always reach their exact final destination, unless if
    \b steps at the index of the servo is 0.
    Upon error, raises <b>'Invalid sweep command'</b>.
    This indicates non-matching list lengths.

    @par Example
    @code sweep ([], [1,2], [180,0], [90,180], [90,45]) @endcode
    Returns
    @code
    [CANCEL_SIGNAL, 1, 180, 2, 0, 1, 90, 2, 45, 2, 90, 2, 135, 2, 180, CANCEL_SIGNAL]
    @endcode
    The protocol this follows is described in servo.ino.
    By intertwining commands in this way, a delay can be inserted between
    bytes and fluid motion is still preserved.
    """

    # current angles of the servos
    pos = list (starts)

    # length of the servo list
    ln = len (servos)

    if ln != len (starts) or ln != len (ends) or ln != len (steps):
        raise NameError ('Invalid sweep command')

    initial.append (CANCEL_SIGNAL)

    # the done list indicates which servos have completed their gestures
    done = [0] * ln
    while 1:
        i = 0
        while i < ln:
            if done [i]:
                i += 1
                continue

            print ('servo:')
            print (servos [i])
            print ('pos:')
            print (pos [i])
            initial.append (servos [i])
            initial.append (pos [i])

            # servo has reached its final position, indicate completion
            if pos [i] == ends [i] or steps [i] == 0:
                done [i] = 1
            # the starting position preceeds the ending position, increment
            elif starts [i] < ends [i]:
                pos [i] += abs (steps [i])
                # ensure that the servo reaches its exact destination
                if pos [i] > ends [i]:
                    pos [i] = ends [i]
            # the ending position preceeds the starting position, decrement
            else:
                pos [i] -= abs (steps [i])
                # ensure that the servo reaches its exact destination
                if pos [i] < ends [i]:
                    pos [i] = ends [i]
            i += 1

        if not (0 in done):
            break

    initial.append (CANCEL_SIGNAL)
    return initial

def unsweep (servos, ends, steps):
    """! Produces a command chain that resets all servos to 0.
    @see sweep()
    """
    starts = list (ends)
    ends = list ([0 for x in servos])
    return sweep ([], servos, starts, ends, steps)

def servowrite (commands, delay):
    """! Writes commands to the serial port.

    If \b delay is 0, immediately writes a list of ints to @ref ser as bytes.
    Otherwise, writes a list of ints to @ref ser as bytes one at a time.

    Inserting a delay controls the speed of the movements.

    @see connect()
    """
    if delay == 0:
        ser.write (commands)
    else:
        for n in commands:
            ser.write ([n])
            sleep (delay)

def gesture (initial, servos, starts, ends, steps, delay):
    """!
    Sets the servos to a position, waits for input, then resets them to 0.
    @see sweep() servowrite()
    """
    initial = sweep (initial, servos, starts, ends, steps)
    print (initial)
    servowrite (initial, delay)
    input ()
    initial = unsweep (servos, ends, steps)
    print (initial)
    servowrite (initial, delay)

def reset ():
    """! Immediately sets the servos to 0."""
    servowrite ([CANCEL_SIGNAL,1,0,2,0,3,0,4,0,5,0], 0)

def cmd ():
    """! Sends the input to the serial port until a blank line is given.
    Must enter integers between 0-255.
    """
    while 1:
        s = (input ())
        if s == '':
            break
        servowrite ([int (s)], 0)

def dump (servos=6):
    """! Retrieves information about the board.
    The command sends the @ref DUMP_SIGNAL to the serial port and reads back
    the response. The response is described in servo.ino.

    When you write your own similar function, timing is important.

    We may need to increase the response delay on the Arduino if you are unable
    to recieve a response. View the source of this function to understand how
    it works, but keep in mind it is untested. Also keep in mind that the
    function is very non-general as it always reads the same amount of bytes
    instead of looking for the end signal.

    @note While this function could be used to change dynamically between
    gestures instead of resetting the servos to 0, this function did not exist
    when this script was first written.
    """

    # ensure there is no pending serial input
    ser.flushInput ()

    # indicate to the board that we with to retrieve information
    servowrite ([DUMP_SIGNAL], 0)

    # read 10 bytes- the begin byte, the id of the board, the number of servos,
    # the positions of the servos (6 servos on the hand), and the end byte
    response = ser.read (servos * 2 + 4)
    # convert the list of bytes into a list of ints
    response = list (response)

    # terminate if the start or end byte is not correct
    if not (response [0] == START_RESPONSE and response [-1] == END_RESPONSE):
        return

    print ('id: ' + str (response [1]))
    if response [1] == RHAND_ID:
        print ('this board is the right hand.')
    print ('servos: ' + str (response [2]))
    servos = response [2]
    i = 0
    while i < servos * 2:
        print ('servo ' + str (response [i + 3]) + ': ' +  str (response [i + 4]))
        i += 2

# movements and gestures

def peace (delay=0):
    """! Makes a peace sign. @see gesture()"""
    gesture ([CANCEL_SIGNAL,2,0,3,0], [1,4,5], [0] * 3, [180] * 3, [10] * 3, delay)

def ok (delay=0.01):
    """! Makes an ok sign. @see gesture()"""
    gesture ([CANCEL_SIGNAL,3,0,4,0,5,0], [1,2], [0,0], [180,160], [10,10], delay)

def grab (delay=0.02):
    """! Makes a fist. @see gesture()"""
    gesture ([], [1,2,3,4,5], [0] * 5, [180] * 5, [60] * 5, delay)

def rockon (delay=0.01):
    """! Makes a rock on sign. @see gesture()"""
    gesture ([], [3,4], [0,0], [180,180], [10,10], 0.01)

def wiggle (n=90, delay=0.01, wiggles=1):
    """! Wiggles the fingers. @see sweep()"""
    th = 40
    step = n
    servowrite (sweep ([CANCEL_SIGNAL,4,0,3,0,2,0,1,th], [5], [0], [n], [step]), delay)
    while wiggles > 0:
        servowrite (sweep ([], [5,4], [n,0], [0,n], [step,step]), delay)
        servowrite (sweep ([], [4,3], [n,0], [0,n], [step,step]), delay)
        servowrite (sweep ([], [3,2], [n,0], [0,n], [step,step]), delay)
        servowrite (sweep ([], [2,1], [n,th], [0,n+th], [step] * 2), delay)
        servowrite (sweep ([], [1], [n+th], [0], [step]), delay)
        servowrite (sweep ([], [2,1], [0,th+n], [n,th], [step] * 2), delay)
        servowrite (sweep ([], [3,2], [0,n], [n,0], [step,step]), delay)
        servowrite (sweep ([], [4,3], [0,n], [n,0], [step,step]), delay)
        servowrite (sweep ([], [5,4], [0,n], [n,0], [step,step]), delay)
        wiggles -= 1
    servowrite (sweep ([], [5], [n], [0], [step]), delay)

def count ():
    """! Counts to 5 on the fingers. @see sweep()"""
    servowrite (sweep ([CANCEL_SIGNAL,2,0], [1,3,4,5], [0] * 4, [180] * 4, [10] * 4), 0.005)
    servowrite (sweep ([], [3], [180], [0], [10]), 0.02)
    servowrite (sweep ([], [4], [180], [0], [10]), 0.02)
    servowrite (sweep ([], [5], [180], [0], [10]), 0.02)
    servowrite (sweep ([], [1], [180], [0], [10]), 0.02)

def demo ():
    """! Performs all movements."""
    peace ()
    sleep (1)
    ok ()
    sleep (1)
    grab ()
    sleep (1)
    wiggle ()
    sleep (1)
    count ()
    sleep (1)
    rockon ()
