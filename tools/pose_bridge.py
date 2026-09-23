#!/usr/bin/env python3
"""Bridge the physics pose feed (UDP) to the browser viewer (WebSocket).

A browser cannot open a UDP socket, so something has to sit in the middle. This
does only that: decode, validate, forward as JSON. It holds no state the viewer
depends on, so it can be restarted at any time.

    ./tools/pose_bridge.py                 # UDP 9100 -> ws://127.0.0.1:9101
    ./tools/pose_bridge.py --serve         # also serve viewer/web on :9102

Then open http://127.0.0.1:9102/ (with --serve) or viewer/web/index.html.

The packet layout is defined in physics/include/fdt/pose_packet.hpp and pinned
by static_asserts there and tests in test_pose_publisher.cpp. The magic number
and version are checked here: a stray packet from one of SITL's nearby ports
must be rejected, not rendered as a pose.
"""
from __future__ import annotations

import argparse
import asyncio
import contextlib
import http.server
import json
import socket
import struct
import threading
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
WEB_ROOT = REPO_ROOT / "viewer" / "web"

# physics/include/fdt/pose_packet.hpp -- 120 bytes, little-endian, no padding.
POSE = struct.Struct("<IHH d 3d 4d 3d 4f f f")
POSE_MAGIC = 0x50544446  # 'FDTP'
POSE_VERSION = 1
FLAG_ARMED = 1 << 0
FLAG_IN_CONTACT = 1 << 1

assert POSE.size == 120, f"pose struct is {POSE.size} bytes, expected 120"


def decode(data: bytes) -> dict | None:
    """-> a dict for the viewer, or None if this is not one of our packets."""
    if len(data) != POSE.size:
        return None
    fields = POSE.unpack(data)
    magic, version, flags = fields[0], fields[1], fields[2]
    if magic != POSE_MAGIC or version != POSE_VERSION:
        return None

    t = fields[3]
    pos = fields[4:7]
    quat = fields[7:11]      # w, x, y, z -- scalar first
    vel = fields[11:14]
    rpm = fields[14:18]
    battery_v, throttle = fields[18], fields[19]

    return {
        "t": t,
        # Sent in world NED. The viewer converts to its own Y-up frame; doing
        # it there keeps this bridge a dumb pipe.
        "pos_ned": list(pos),
        "quat_wxyz": list(quat),
        "vel_ned": list(vel),
        "rpm": list(rpm),
        "battery_v": battery_v,
        "throttle": throttle,
        "armed": bool(flags & FLAG_ARMED),
        "in_contact": bool(flags & FLAG_IN_CONTACT),
    }


class Stats:
    def __init__(self):
        self.received = 0
        self.rejected = 0
        self.forwarded = 0


async def run(udp_port: int, ws_host: str, ws_port: int, stats: Stats) -> None:
    import websockets

    clients: set = set()

    async def handler(ws):
        clients.add(ws)
        print(f"viewer connected ({len(clients)} total)")
        try:
            await ws.wait_closed()
        finally:
            clients.discard(ws)
            print(f"viewer disconnected ({len(clients)} left)")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", udp_port))
    sock.setblocking(False)
    loop = asyncio.get_running_loop()

    async with websockets.serve(handler, ws_host, ws_port):
        print(f"pose bridge: udp/{udp_port} -> ws://{ws_host}:{ws_port}")
        while True:
            data = await loop.sock_recv(sock, 2048)
            stats.received += 1
            pose = decode(data)
            if pose is None:
                stats.rejected += 1
                if stats.rejected in (1, 10, 100):
                    print(f"ignored {stats.rejected} packet(s) that are not ours "
                          f"(wrong magic, version or size)")
                continue
            if not clients:
                continue
            message = json.dumps(pose)
            await asyncio.gather(*(c.send(message) for c in list(clients)),
                                 return_exceptions=True)
            stats.forwarded += 1


def serve_web(port: int) -> None:
    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(WEB_ROOT), **kwargs)

        def log_message(self, *args):  # silence per-request logging
            pass

    server = http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"viewer:      http://127.0.0.1:{port}/")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--udp-port", type=int, default=9100)
    parser.add_argument("--ws-host", default="127.0.0.1")
    parser.add_argument("--ws-port", type=int, default=9101)
    parser.add_argument("--serve", action="store_true", help=f"serve {WEB_ROOT} over HTTP")
    parser.add_argument("--http-port", type=int, default=9102)
    args = parser.parse_args()

    if args.serve:
        if not WEB_ROOT.is_dir():
            print(f"no such directory: {WEB_ROOT}")
            return 2
        serve_web(args.http_port)

    stats = Stats()
    try:
        asyncio.run(run(args.udp_port, args.ws_host, args.ws_port, stats))
    except KeyboardInterrupt:
        print(f"\nreceived {stats.received}, forwarded {stats.forwarded}, "
              f"rejected {stats.rejected}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
