#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py

Extends the communication framework from Studio 12 with:
  - Magic number + checksum framing for reliable packet delivery
  - A command-line interface that reads user commands while displaying
    live Arduino data

Packet framing format (103 bytes total):
  MAGIC (2 B) | TPacket (100 B) | CHECKSUM (1 B)

Usage:
  source env/bin/activate
  python3 pi_sensor.py
"""

from second_terminal import relay # new

import struct
import serial
import time
import sys
import select
import re

import alex_camera as cam


# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None


def openSerial():
    """Open the serial port and wait for the Arduino to boot."""
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)
    print("Ready.\n")


def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()


# ----------------------------------------------------------------
# TPACKET CONSTANTS
# (must match sensor_miniproject_template.ino)
# ----------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP       = 0
# TODO (Activity 2): define your own command type for the color sensor here.
# It must match the value you add to TCommandType in packets.h.
COMMAND_COLOR       = 2
COMMAND_FORWARD     = 3
COMMAND_LEFT        = 4
COMMAND_BACKWARD    = 5
COMMAND_RIGHT       = 6
COMMAND_STOP        = 7
COMMAND_INCREASE    = 8
COMMAND_DECREASE    = 9
COMMAND_BASE        = 10
COMMAND_SHOULDER    = 11
COMMAND_ELBOW       = 12
COMMAND_GRIPPER     = 13
COMMAND_VELOCITY    = 14
COMMAND_HOME        = 15

RESP_OK         = 0
RESP_STATUS     = 1
# TODO (Activity 2): define your own response type for the color sensor here.
# It must match the value you add to TResponseType in packets.h.
RESP_COLOR      = 2
# Movement Response
RESP_FORWARD    = 3
RESP_LEFT       = 4
RESP_BACKWARD   = 5
RESP_RIGHT      = 6
RESP_STOP       = 7
# Speed Response
RESP_INCREASE   = 8
RESP_DECREASE   = 9
RESP_BASE       = 10
RESP_SHOULDER   = 11
RESP_ELBOW      = 12
RESP_GRIPPER    = 13
RESP_VELOCITY   = 14
RESP_HOME       = 15

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

# ----------------------------------------------------------------
# RELIABLE FRAMING: magic number + XOR checksum
# ----------------------------------------------------------------

MAGIC = b'\xDE\xAD'          # 2-byte magic number (0xDEAD)
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # 2 + 100 + 1 = 103


def computeChecksum(data: bytes) -> int:
    """Return the XOR of all bytes in data."""
    result = 0
    for b in data:
        result ^= b
    return result


def packFrame(packetType, command, data=b'', params=None):
    """
    Build a framed packet: MAGIC | TPacket bytes | checksum.

    Args:
        packetType: integer packet type constant
        command:    integer command / response constant
        data:       bytes for the data field (padded/truncated to MAX_STR_LEN)
        params:     list of PARAMS_COUNT uint32 values (default: all zeros)

    Returns:
        bytes of length FRAME_SIZE (103)
    """
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 100-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:])
    }


def receiveFrame():
    """
    Read bytes from the serial port until a valid framed packet is found.

    Synchronises to the magic number, reads the TPacket body, then
    validates the checksum.  Discards corrupt or out-of-sync bytes.

    Returns:
        A packet dict (see unpackTPacket), or None on timeout / error.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        # Read and discard bytes until we see the first magic byte.
        b = _ser.read(1)
        if not b:
            return None          # timeout
        if b[0] != MAGIC_HI:
            continue

        # Read the second magic byte.
        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            # Not the magic number; keep searching (don't skip the byte
            # we just read in case it is the first byte of another frame).
            continue

        # Magic matched; now read the TPacket body.
        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        # Read and verify the checksum.
        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected:
            # Checksum mismatch: corrupted packet, try to resync.
            continue

        return unpackTPacket(raw)


def sendCommand(commandType, data=b'', params=None):
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING


def isEstopActive():
    """Return True if the E-Stop is currently active (system stopped)."""
    return _estop_state == STATE_STOPPED


# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    """Print a received TPacket in human-readable form.

    The 'data' field carries an optional debug string from the Arduino.
    When non-empty, it is printed automatically so you can embed debug
    messages in any outgoing TPacket on the Arduino side (set pkt.data to
    a null-terminated string up to 31 characters before calling sendFrame).
    This works like Serial.print(), but sends output to the Pi terminal.
    """
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            else:
                print("Status: STOPPED")
            
            # TODO (Activity 2): add an elif branch here to handle your color
            # response.  Display the three channel frequencies in Hz, e.g.:
            #   R: <params[0]> Hz, G: <params[1]> Hz, B: <params[2]> Hz
        elif cmd == RESP_COLOR:
            red = pkt['params'][0]
            green = pkt['params'][1]
            blue = pkt['params'][2]
            print(f"R: {red} Hz, G: {green} Hz, B: {blue} Hz")

        elif cmd == RESP_FORWARD:
            speed = pkt['params'][0]
            print(f"Moving forward by {speed}")

        elif cmd == RESP_LEFT:
            speed = pkt['params'][0]
            print(f"Turning left by {speed}")

        elif cmd == RESP_BACKWARD:
            speed = pkt['params'][0]
            print(f"Moving backward by {speed}")

        elif cmd == RESP_RIGHT:
            speed = pkt['params'][0]
            print(f"Turning right by {speed}")

        elif cmd == RESP_STOP:
            print("Robot has stopped")

        elif cmd == RESP_INCREASE or cmd == RESP_DECREASE:
            speed = pkt['params'][0]
            print(f"Current speed {speed}")

        elif cmd == RESP_BASE:
            print(f"Base servo  -> {pkt['params'][0]} deg")
        elif cmd == RESP_SHOULDER:
            print(f"Shoulder    -> {pkt['params'][0]} deg")
        elif cmd == RESP_ELBOW:
            print(f"Elbow       -> {pkt['params'][0]} deg")
        elif cmd == RESP_GRIPPER:
            print(f"Gripper     -> {pkt['params'][0]} deg")
        elif cmd == RESP_VELOCITY:
            print(f"Arm speed   -> {pkt['params'][0]} ms/deg")
        elif cmd == RESP_HOME:
            print("Homing all servos to 90°")
        else:
            print(f"Response: unknown command {cmd}")

        # Print the optional debug string from the data field.
        # On the Arduino side, fill pkt.data before calling sendFrame() to
        # send debug messages to this terminal (similar to Serial.print()).
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"Arduino debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Arduino: {msg}")
    else:
        print(f"Packet: type={ptype}, cmd={cmd}")


# ----------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ----------------------------------------------------------------

def handleColorCommand():
    """
    TODO (Activity 2): request a color reading from the Arduino and display it.

    Check the E-Stop state first; if stopped, refuse with a clear message.
    Otherwise, send your color command to the Arduino.
    """

    if isEstopActive():
        print("Refused: E-Stop is active.")
        return

    print("sending colour command...")
    sendCommand(COMMAND_COLOR)

# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------

# TODO (Activity 3): import the camera library provided (alex_camera.py).

# TODO (Activity 3): open the camera (cameraOpen()) before first use.
# frames remaining before further captures are refused
_camera = None
_frames_remaining = 10   


def handleCameraCommand():
    """
    TODO (Activity 3): capture and display a greyscale frame.

    Gate on E-Stop state and the remaining frame count.
    Use captureGreyscaleFrame() and renderGreyscaleFrame() from alex_camera.
    """
    if isEstopActive():
        print("Refused: E-Stop is active.")
        return
    global _frames_remaining, _camera
    if _camera is None:
        _camera = cam.cameraOpen()
    
    if _frames_remaining <= 0 :
        print("no frames remaining")
    else:
        frame = cam.captureGreyscaleFrame(_camera)
        cam.renderGreyscaleFrame(frame)
        _frames_remaining -= 1
        print(f"remaining frames left: {_frames_remaining}")


# ----------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ----------------------------------------------------------------

# TODO (Activity 4): import from lidar.alex_lidar and lidar_example_cli_plot
#   (lidar_example_cli_plot.py is in the same folder; alex_lidar.py is in lidar/).

def handleLidarCommand():
    from lidar_example_cli_plot import plot_single_scan
    """
    TODO (Activity 4): perform a single LIDAR scan and render it.

    Gate on E-Stop state, then use the LIDAR library to capture one scan
    and the CLI plot helpers to display it.
    """
    # TODO
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    import lidar.alex_lidar as alex_lidar
    from lidar_example_cli_plot import plot_single_scan
    print("Performing LIDAR scan...")

    plot_single_scan()
    
# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

# User input -> action mapping:
#   e  send a software E-Stop command to the Arduino (pre-wired)
#   c  request color reading from the Arduino        (Activity 2 - implement yourself)
#   p  capture and display a camera frame            (Activity 3 - implement yourself)
#   l  perform a single LIDAR scan                   (Activity 4 - implement yourself)

def handleMovementCommand(movement):
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    print("sending movement command...")
    if movement == 'w':
        sendCommand(COMMAND_FORWARD)
    elif movement == 'a':
        sendCommand(COMMAND_LEFT)
    elif movement == 's':
        sendCommand(COMMAND_BACKWARD)
    elif movement == 'd':
        sendCommand(COMMAND_RIGHT)
    elif movement == 'x':
        sendCommand(COMMAND_STOP)

def handleSpeedCommand(speed):
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    print("changing speed...")
    if speed == '+':
        sendCommand(COMMAND_INCREASE)
    elif speed == '-':
        sendCommand(COMMAND_DECREASE)

_ARM_COMMANDS = {
    'B': (COMMAND_BASE,     'base'),
    'S': (COMMAND_SHOULDER, 'shoulder'),
    'E': (COMMAND_ELBOW,    'elbow'),
    'G': (COMMAND_GRIPPER,  'gripper'),
    'V': (COMMAND_VELOCITY, 'velocity'),
}

def handleArmCommand(letter, value):
    """Send a servo/velocity command with the numeric value in params[0]."""
    if isEstopActive():
        print("Refused: E-Stop is active.")
        return
    cmd_type, label = _ARM_COMMANDS[letter]
    print(f"Sending {label} command: {value}")
    params = [value] + [0] * (PARAMS_COUNT - 1)
    sendCommand(cmd_type, params=params)

def handleUserInput(line):
    """
    Dispatch a single line of user input.

    The 'e' case is pre-wired to send a software E-Stop command.
    TODO (Activities 2, 3 & 4): add 'c' (color), 'p' (camera) and 'l' (LIDAR).
    """
    m = re.fullmatch(r'([BSEGVbsegv])(\d{3})', line)
    if m:
        letter = m.group(1).upper()
        value  = int(m.group(2))
        handleArmCommand(letter, value)
        return
    if line == 'e':
        print("Sending E-Stop command...")
        sendCommand(COMMAND_ESTOP, data=b'This is a debug message')
    # TODO (Activity 2): add an elif branch for 'c' (color sensor) that calls handleColorCommand().
    elif line == 'c':
        handleColorCommand()
    # TODO (Activities 3 & 4): add elif branches for 'p' (camera) and 'l' (LIDAR).
    elif line == 'l':
        handleLidarCommand()
    elif line == 'p':
        handleCameraCommand()
    elif line == 'w' or line == 'a' or line == 's' or line == 'd' or line == 'x':
        handleMovementCommand(line)
    elif line == '+' or line == '-':
        handleSpeedCommand(line)
    elif line == 'h':
        if isEstopActive():
            print("Refused: E-Stop is active.")
        else:
            print("Homing arm...")
            sendCommand(COMMAND_HOME)
    else:
        print(f"Unknown input: '{line}'. Valid: e, c, p, l, w, a, s, d, x, +, -")


def runCommandInterface():
    """
    Main command loop.

    Uses select.select() to simultaneously receive packets from the Arduino
    and read typed user input from stdin without either blocking the other.
    """
    print("Robot interface ready.")
    print("Sensor: Type e / c / p / l and press Enter for E-stop / Color / Picture / Lidar.")
    print("Movement: Type w / a / s / d / x and press Enter to move Forward / Left / Backward / Right / Stop.")
    print("Speed change: Type + / - and press Enter to Increase / Decrease Speed. Curr = 165 | Max = 255 | Min = 0.")
    print("Press Ctrl+C to exit.\n")

    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)
                relay.onPacketReceived(packFrame(pkt['packetType'], pkt['command'], pkt['data'], pkt['params'])) # new

        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if not line:
                time.sleep(0.05)
                continue
            handleUserInput(line)

        relay.checkSecondTerminal(_ser) # new
        time.sleep(0.05)


# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    relay.start() # new
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        relay.shutdown() # new
        # TODO (Activities 3 & 4): close the camera and disconnect the LIDAR here if you opened them.
        closeSerial()

