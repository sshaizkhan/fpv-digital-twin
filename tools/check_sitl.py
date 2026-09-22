#!/usr/bin/env python3
"""Verify a running Betaflight SITL is the firmware we think it is.

Speaks MSP over the TCP port the Configurator uses, so a pass here is direct
evidence the Configurator will connect too. Also checks the reported version
against config/quad.yaml, so SITL cannot silently drift from the real FC.

    ./tools/run_sitl.sh &
    ./tools/check_sitl.py

Exit 0 on success, 1 on mismatch (wrong variant/version, or no pin to compare
against), 2 if SITL is not reachable or is not answering MSP.
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


class MspError(Exception):
    """SITL is reachable on the port but is not speaking MSP as expected."""


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except OSError as exc:  # includes socket.timeout
            raise MspError(f"no MSP reply within the timeout: {exc}") from exc
        if not chunk:
            raise MspError("SITL closed the connection")
        buf += chunk
    return buf


def msp_call(sock: socket.socket, cmd: int) -> bytes:
    sock.sendall(msp_request(cmd))
    header = _recv_exact(sock, 5)
    if header[:3] == b"$M!":
        raise MspError(f"SITL rejected command {cmd}")
    if header[:3] != b"$M>":
        raise MspError(f"not an MSP reply: {header!r}")
    length, reply_cmd = header[3], header[4]
    payload = _recv_exact(sock, length)
    _recv_exact(sock, 1)  # checksum
    if reply_cmd != cmd:
        raise MspError(f"asked for command {cmd}, got {reply_cmd}")
    return payload


def expected_version() -> str | None:
    """betaflight_version from config/quad.yaml, without a YAML dependency.

    Quotes are optional in YAML, so accept both forms -- requiring them made a
    legal `betaflight_version: 4.5.1` look like "no pin configured".
    """
    try:
        text = (REPO_ROOT / "config" / "quad.yaml").read_text()
    except OSError:
        return None
    match = re.search(
        r'^[ \t]*betaflight_version:[ \t]*(?:"([^"]+)"|\'([^\']+)\'|([^\s#]+))[ \t]*(?:#.*)?$',
        text,
        re.MULTILINE,
    )
    if not match:
        return None
    return next(g for g in match.groups() if g is not None)


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

    try:
        with sock:
            api = msp_call(sock, MSP_API_VERSION)
            variant = msp_call(sock, MSP_FC_VARIANT).decode(errors="replace")
            version = msp_call(sock, MSP_FC_VERSION)
    except MspError as exc:
        print(f"SITL at {args.host}:{args.port} is not answering MSP: {exc}", file=sys.stderr)
        print("is it still booting? give it a moment and retry.", file=sys.stderr)
        return 2

    if len(version) < 3 or len(api) < 3:
        print(f"short MSP reply: version={version!r} api={api!r}", file=sys.stderr)
        return 2

    reported = f"{version[0]}.{version[1]}.{version[2]}"
    print(f"MSP {args.host}:{args.port}  variant={variant}  version={reported}  api={api[1]}.{api[2]}")

    ok = True
    if variant != "BTFL":
        print(f"FAIL: expected variant BTFL, got {variant!r}", file=sys.stderr)
        ok = False

    want = expected_version()
    if want is None:
        # This check exists to catch SITL drifting from the real FC. If the pin
        # it compares against is missing or unreadable, the check is not
        # passing -- it is not running, and must not report success.
        print("FAIL: no readable betaflight_version in config/quad.yaml", file=sys.stderr)
        print("      there is nothing to compare SITL against.", file=sys.stderr)
        ok = False
    elif reported != want:
        print(f"FAIL: SITL runs {reported}, but config/quad.yaml pins {want}", file=sys.stderr)
        print("      SITL must match the firmware on the real quad.", file=sys.stderr)
        ok = False
    else:
        print(f"OK: matches config/quad.yaml ({want}) -- the Configurator will connect here")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
