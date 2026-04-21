#!/usr/bin/env python3
"""
streamer.py - TCP map streamer for the SLAM system.

Streams the live occupancy map and robot pose to a remote client (e.g. a
laptop running client_gui.py) over a plain TCP socket.

Each frame sent to the client is exactly PAYLOAD_SIZE bytes:

  Bytes 0-63   : UTF-8 pose string "x_mm,y_mm,theta_deg", space-padded to 64 B
  Bytes 64-...  : raw occupancy map (MAP_SIZE_PIXELS^2 bytes, flat row-major)

The server accepts one client at a time.  If the client disconnects it waits
for the next connection.  The thread exits automatically when pss.stopped is set.

Call start_map_streamer(pss) once after the ProcessSharedState has been
created.  The streamer thread is a daemon so it will not prevent the process
from exiting.
"""

import socket
import threading
import time
import zlib
import struct

from settings import MAP_SIZE_PIXELS, MAP_UPDATE_HZ

# Total bytes sent per frame.
_MAP_BYTES = MAP_SIZE_PIXELS * MAP_SIZE_PIXELS
_POSE_BYTES = 64
PAYLOAD_SIZE = _POSE_BYTES + _MAP_BYTES

# How long accept() blocks before re-checking pss.stopped.
_ACCEPT_TIMEOUT_S = 1.0


def start_map_streamer(pss, host: str = '0.0.0.0', port: int = 5005) -> None:
    """Start the map-streamer background thread.

    Parameters
    ----------
    pss  : ProcessSharedState instance (must already be initialised).
    host : address to bind on (default '0.0.0.0' = all interfaces).
    port : TCP port to listen on (default 5005).
    """
    thread = threading.Thread(
        target=_streamer_thread,
        args=(pss, host, port),
        name='map-streamer',
        daemon=True,
    )
    thread.start()
    print(f'[streamer] listening on {host}:{port}')


# ---------------------------------------------------------------------------
# Internal implementation
# ---------------------------------------------------------------------------

def _streamer_thread(pss, host: str, port: int) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Non-blocking accept loop: time out every second to check pss.stopped.
    server.settimeout(_ACCEPT_TIMEOUT_S)
    server.bind((host, port))
    server.listen(1)

    interval = 0.5 / MAP_UPDATE_HZ

    try:
        while not pss.stopped.value:
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue  # re-check pss.stopped

            print(f'[streamer] client connected from {addr}')
            try:
                _serve_client(conn, pss, interval)
            except (ConnectionResetError, BrokenPipeError):
                pass
            except Exception as exc:
                print(f'[streamer] client error: {exc}')
            finally:
                conn.close()
                print(f'[streamer] client {addr} disconnected')
    finally:
        server.close()
        print('[streamer] server closed')


def _serve_client(conn: socket.socket, pss, interval: float) -> None:
    """Send frames to a connected client until it disconnects or SLAM stops."""
    while not pss.stopped.value:
        # Read map snapshot from shared memory.
        map_data = bytes(pss.shm.buf[:_MAP_BYTES])
        compressed_map = zlib.compress(map_data, level=1)

        # Encode pose as a fixed-width ASCII field.
        pose_str = f'{pss.x_mm.value},{pss.y_mm.value},{pss.theta_deg.value}'
        pose_bytes = pose_str.encode('utf-8')[:_POSE_BYTES]
        pose_padded = pose_bytes.ljust(_POSE_BYTES, b' ')

        # Pack the size of the compressed map into a 4-byte integer
        size_bytes = struct.pack('<I', len(compressed_map))

        # Send: [64B Pose] + [4B Map Size] + [Compressed Map]
        conn.sendall(pose_padded + size_bytes + compressed_map)

        time.sleep(interval)
