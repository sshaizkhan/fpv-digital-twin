#!/usr/bin/env python3
"""Verify a running Betaflight SITL is the firmware we think it is.

Speaks MSP over the TCP port the Configurator uses, so a pass here is direct
evidence the Configurator will connect too. Also checks the reported version
against config/quad.yaml, so SITL cannot silently drift from the real FC.

    ./tools/run_sitl.sh &
    ./tools/check_sitl.py

Exit 0 on success, 1 on mismatch, 2 if SITL is not reachable.
"""
# Works on the system python 3.9 as well as the 3.13 venv, so a quick SITL
# sanity check never depends on having activated anything.
from __future__ import annotations

import argparse
import re
import socket
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3


def msp_request(cmd: int) -> bytes:
    """MSPv1 request frame: '$M<', length, command, checksum (XOR)."""
    return b"$M<" + bytes([0, cmd, 0 ^ cmd])


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("SITL closed the connection")
        buf += chunk
    return buf


def msp_call(sock: socket.socket, cmd: int) -> bytes:
    sock.sendall(msp_request(cmd))
    header = _recv_exact(sock, 5)
    if header[:3] != b"$M>":
        raise ValueError(f"not an MSP reply: {header!r}")
    length, reply_cmd = header[3], header[4]
    payload = _recv_exact(sock, length)
    _recv_exact(sock, 1)  # checksum
    if reply_cmd != cmd:
        raise ValueError(f"asked for command {cmd}, got {reply_cmd}")
    return payload


def expected_version() -> str | None:
    """betaflight_version from config/quad.yaml, without a YAML dependency."""
    text = (REPO_ROOT / "config" / "quad.yaml").read_text()
    match = re.search(r'^\s*betaflight_version:\s*"([^"]+)"', text, re.MULTILINE)
    return match.group(1) if match else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761, help="UART1 = BASE_PORT(5760) + 1")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    try:
        sock = socket.create_connection((args.host, args.port), timeout=args.timeout)
    except OSError as exc:
        print(f"cannot reach SITL at {args.host}:{args.port}: {exc}", file=sys.stderr)
        print("is ./tools/run_sitl.sh running?", file=sys.stderr)
        return 2

    with sock:
        api = msp_call(sock, MSP_API_VERSION)
        variant = msp_call(sock, MSP_FC_VARIANT).decode(errors="replace")
        version = msp_call(sock, MSP_FC_VERSION)

    reported = f"{version[0]}.{version[1]}.{version[2]}"
    print(f"MSP {args.host}:{args.port}  variant={variant}  version={reported}  api={api[1]}.{api[2]}")

    ok = True
    if variant != "BTFL":
        print(f"FAIL: expected variant BTFL, got {variant!r}", file=sys.stderr)
        ok = False

    want = expected_version()
    if want is None:
        print("WARN: config/quad.yaml has no betaflight_version to compare against", file=sys.stderr)
    elif reported != want:
        print(f"FAIL: SITL runs {reported}, but config/quad.yaml pins {want}", file=sys.stderr)
        print("      SITL must match the firmware on the real quad.", file=sys.stderr)
        ok = False
    else:
        print(f"OK: matches config/quad.yaml ({want}) -- the Configurator will connect here")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
