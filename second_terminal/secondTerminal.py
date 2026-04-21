#!/usr/bin/env python3
"""
Studio 16: Robot Integration
second_terminal.py  -  Second operator terminal.

This terminal connects to pi_sensor.py over TCP.  It:
  - Displays every TPacket forwarded from the robot (via pi_sensor.py).
  - Sends a software E-Stop command when you type 'e'.

Architecture
------------
   [Arduino] <--USB serial--> [pi_sensor.py] <--TCP--> [second_terminal.py]
                                (TCP server,               (TCP client,
                                 port 65432)                localhost:65432)

Run pi_sensor.py FIRST (it starts the TCP server), then run this script.
Both scripts run on the same Raspberry Pi.

IMPORTANT: Update the TPacket constants below to match your pi_sensor.py.
---------------------------------------------------------------------------
The packet constants (PACKET_TYPE_*, COMMAND_*, RESP_*, STATE_*, sizes) are
duplicated here from pi_sensor.py.  They MUST stay in sync with your
pi_sensor.py (and with the Arduino sketch).  Update them whenever you change
your protocol.

Tip: consider abstracting all TPacket constants into a shared file (e.g.
packets.py) that both pi_sensor.py and second_terminal.py import, so there
is only one place to update them.  You do not have to do this now, but it
avoids hard-to-find bugs caused by constants getting out of sync.

Commands
--------
  e   Send a software E-Stop to the robot (same as pressing the button).
  q   Quit.

Usage
-----
    source env/bin/activate
    python3 second_terminal/second_terminal.py

Press Ctrl+C to exit.
"""

import re
import select
import struct
import sys
import time

# net_utils is imported with an absolute import because this script is designed
# to be run directly (python3 second_terminal/second_terminal.py), which adds
# this file's directory to sys.path automatically.
from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame


# ---------------------------------------------------------------------------
# Connection settings
# ---------------------------------------------------------------------------
# Both scripts run on the same Pi, so the host is 'localhost'.
# Change PI_HOST to the Pi's IP address if you run this from a different machine.
PI_HOST = 'localhost'
PI_PORT = 65432


# ---------------------------------------------------------------------------
# TPacket constants
# ---------------------------------------------------------------------------
# IMPORTANT: keep these in sync with your pi_sensor.py and the Arduino sketch.

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP = 0

COMMAND_COLOR = 2 # new
COMMAND_FORWARD = 3 # new
COMMAND_LEFT = 4  # new
COMMAND_BACKWARD = 5 # new
COMMAND_RIGHT = 6 # new
COMMAND_STOP = 7 # new
COMMAND_INCREASE  = 8 # new
COMMAND_DECREASE  = 9 # new
COMMAND_BASE      = 10
COMMAND_SHOULDER  = 11
COMMAND_ELBOW     = 12
COMMAND_GRIPPER   = 13
COMMAND_VELOCITY  = 14
COMMAND_HOME      = 15

RESP_OK       = 0
RESP_STATUS   = 1
RESP_COLOR    = 2 # new
RESP_FORWARD  = 3 # new
RESP_LEFT     = 4 # new
RESP_BACKWARD = 5 # new
RESP_RIGHT    = 6 # new
RESP_STOP     = 7 # new
RESP_INCREASE = 8 # new
RESP_DECREASE = 9 # new
RESP_BASE     = 10
RESP_SHOULDER = 11
RESP_ELBOW    = 12
RESP_GRIPPER  = 13
RESP_VELOCITY = 14
RESP_HOME     = 15

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16
TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)   # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC      = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # = 103


# ---------------------------------------------------------------------------
# TPacket helpers
# ---------------------------------------------------------------------------

def _computeChecksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def _packFrame(packetType, command, data=b'', params=None):
    """Pack a TPacket into a 103-byte framed byte string."""
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded  = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    return MAGIC + packet_bytes + bytes([_computeChecksum(packet_bytes)])


def _unpackFrame(frame: bytes):
    """Validate checksum and unpack a 103-byte frame.  Returns None if corrupt."""
    if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
        return None
    raw = frame[2:2 + TPACKET_SIZE]
    if frame[-1] != _computeChecksum(raw):
        return None
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


# ---------------------------------------------------------------------------
# Packet display
# ---------------------------------------------------------------------------

_estop_active = False


def _printPacket(pkt):
    """Pretty-print a TPacket forwarded from the robot."""
    global _estop_active

    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("[robot] OK")
        elif cmd == RESP_STATUS:
            state         = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")
        elif cmd == RESP_COLOR:
            print("[robot] ACK: COLOR")
        elif cmd == RESP_FORWARD:
            print("[robot] ACK: FORWARD")
        elif cmd == RESP_LEFT:
            print("[robot] ACK: LEFT")
        elif cmd == RESP_BACKWARD:
            print("[robot] ACK: BACKWARD")
        elif cmd == RESP_RIGHT:
            print("[robot] ACK: RIGHT")
        elif cmd == RESP_STOP:
            print("[robot] ACK: STOP")
        elif cmd == RESP_INCREASE:
            speed = pkt['params'][0]
            print(f"[robot] ACK: INCREASE  (speed={speed})" if speed else "[robot] ACK: INCREASE")
        elif cmd == RESP_DECREASE:
            speed = pkt['params'][0]
            print(f"[robot] ACK: DECREASE  (speed={speed})" if speed else "[robot] ACK: DECREASE")
        elif cmd == RESP_BASE:
            print(f"[robot] Base servo   -> {pkt['params'][0]} deg")
        elif cmd == RESP_SHOULDER:
            print(f"[robot] Shoulder     -> {pkt['params'][0]} deg")
        elif cmd == RESP_ELBOW:
            print(f"[robot] Elbow        -> {pkt['params'][0]} deg")
        elif cmd == RESP_GRIPPER:
            print(f"[robot] Gripper      -> {pkt['params'][0]} deg")
        elif cmd == RESP_VELOCITY:
            print(f"[robot] Arm speed    -> {pkt['params'][0]} ms/deg")
        elif cmd == RESP_HOME:
            print("[robot] Homing all servos to 90°")
        else:
            print(f"[robot] Response: unknown command {cmd}")
        # Print any debug string embedded in the data field.
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"[robot] Debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"[robot] Message: {msg}")

    else:
        print(f"[robot] Packet: type={ptype}, cmd={cmd}")


# ---------------------------------------------------------------------------
# Input handling
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Arm command table  (matches pi_sensor.py _ARM_COMMANDS)
# ---------------------------------------------------------------------------
_ARM_COMMANDS = {
    'B': (COMMAND_BASE,     'base'),
    'S': (COMMAND_SHOULDER, 'shoulder'),
    'E': (COMMAND_ELBOW,    'elbow'),
    'G': (COMMAND_GRIPPER,  'gripper'),
    'V': (COMMAND_VELOCITY, 'velocity'),
}


# ---------------------------------------------------------------------------
# Input handling
# ---------------------------------------------------------------------------

def _handleInput(line: str, client: TCPClient):
    """Handle one line of keyboard input."""
    line = line.strip()
    if not line:
        return

    # --- Arm servo / velocity:  letter + 3 digits  e.g. B090, S180, V010 ---
    m = re.fullmatch(r'([BSEGVbsegv])(\d{3})', line)
    if m:
        letter    = m.group(1).upper()
        value     = int(m.group(2))
        cmd_type, label = _ARM_COMMANDS[letter]
        params    = [value] + [0] * (PARAMS_COUNT - 1)
        frame     = _packFrame(PACKET_TYPE_COMMAND, cmd_type, params=params)
        sendTPacketFrame(client.sock, frame)
        print(f"[second_terminal] Sent: {label.upper()} {value}")
        return

    # Single-character commands (case-insensitive)
    key = line.lower()

    if key == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: E-STOP")

    elif key == 'w':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_FORWARD)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: FORWARD")

    elif key == 'a':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_LEFT)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: LEFT")

    elif key == 's':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_BACKWARD)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: BACKWARD")

    elif key == 'd':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_RIGHT)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: RIGHT")

    elif key == 'x':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_STOP)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: STOP")

    elif key == 'c':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_COLOR)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: COLOR")

    elif key == '+':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_INCREASE)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: INCREASE")

    elif key == '-':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_DECREASE)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: DECREASE")

    elif key == 'h':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_HOME)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: HOME")

    elif key == 'q':
        print("[second_terminal] Quitting.")
        raise KeyboardInterrupt

    else:
        print(f"[second_terminal] Unknown: '{line}'.")
        print("  Movement : w=Forward  s=Backward  a=Left  d=Right  x=Stop")
        print("  Speed    : +=Increase  -=Decrease")
        print("  Arm      : B/S/E/G/V + 3-digit angle  e.g. B090  S045  V010")
        print("  Other    : c=Color  h=Home  e=E-Stop  q=Quit")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run():
    client = TCPClient(host=PI_HOST, port=PI_PORT)
    print(f"[second_terminal] Connecting to pi_sensor.py at {PI_HOST}:{PI_PORT}...")

    if not client.connect(timeout=10.0):
        print("[second_terminal] Could not connect.")
        print("  Make sure pi_sensor.py is running and waiting for a"
              " second terminal connection.")
        sys.exit(1)

    print("[second_terminal] Connected!")
    print("[second_terminal] Movement : w=Forward  s=Backward  a=Left  d=Right  x=Stop")
    print("[second_terminal] Speed    : +=Increase  -=Decrease")
    print("[second_terminal] Arm      : B/S/E/G/V + 3-digit angle  e.g. B090  S045  V010")
    print("[second_terminal] Other    : c=Color  h=Home  e=E-Stop  q=Quit")
    print("[second_terminal] Incoming robot packets will be printed below.\n")

    try:
        while True:
            # Check for forwarded TPackets from pi_sensor.py (non-blocking).
            if client.hasData():
                frame = recvTPacketFrame(client.sock)
                if frame is None:
                    print("[second_terminal] Connection to pi_sensor.py closed.")
                    break
                pkt = _unpackFrame(frame)
                if pkt:
                    _printPacket(pkt)

            # Check for keyboard input (non-blocking via select).
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                line = sys.stdin.readline()
                _handleInput(line, client)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[second_terminal] Exiting.")
    finally:
        client.close()


if __name__ == '__main__':
    run()