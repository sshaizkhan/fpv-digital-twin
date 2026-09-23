#!/usr/bin/env python3
"""Replay the real flight controller's `diff all` into a running Betaflight SITL.

SITL exposes UART1 as a TCP socket, and that is the same CLI the Configurator's
CLI tab drives. Replaying the diff through it is more repeatable than pasting by
hand, it is versioned with the repo, and it re-applies itself whenever the
container is recreated instead of depending on .sitl-state/eeprom.bin surviving.

Some of the diff CANNOT apply to SITL, and this script reports every such line
rather than silently swallowing it -- those rejections are exactly the places
where the sim differs from your real quad:

  * motor_pwm_protocol  DSHOT is not compiled into SITL at all (USE_DSHOT is
                        defined only when !SITL, common_pre.h:52-54), so the
                        protocol is overridden to PWM. Left as DSHOT300 the
                        motor output stays DISABLED and SITL never emits a
                        motor packet.
  * dshot_bidir         meaningless without DSHOT; it is what gives the real
                        quad RPM telemetry, which the sim gets from its own
                        motor model instead.
  * board_name etc.     SITL is not a SPEEDYBEEF405V4 and has none of its pins.

    ./tools/run_sitl.sh &
    ./tools/load_config_sitl.py                 # apply, save, verify
    ./tools/load_config_sitl.py --dry-run       # show the plan, send nothing

`save` reboots the FC, and in SITL a reboot exits the process -- which stops the
container. Pass --restart-container to have that handled.
"""
from __future__ import annotations

import argparse
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Betaflight prints "###ERROR IN <cmd>: <detail>" (cli.c:431-439).
ERROR_MARKER = "###ERROR IN"

# Lines that describe the physical board. SITL has none of this hardware, and
# board_name in particular can trigger a config reset mid-replay.
SKIP_PREFIXES = (
    "board_name",
    "manufacturer_id",
    "mcu_id",
    "signature",
    "resource",
    "timer",
    "dma",
)

# setting name -> (value SITL can use, why)
OVERRIDES = {
    "motor_pwm_protocol": ("PWM", "SITL has no DSHOT (common_pre.h:52-54); DSHOT leaves motor output DISABLED"),
}

# Commands that do not exist in a SITL build. Confirmed by sending them once
# and getting "UNKNOWN COMMAND" / "INVALID NAME" back. They are skipped rather
# than sent so that a rejection in the report means something genuinely
# unexpected, instead of being lost in a wall of known-benign noise.
ABSENT_COMMANDS = {
    "beeper": "no beeper hardware in a SITL build",
    "beacon": "no beeper hardware in a SITL build",
}
ABSENT_SETTINGS = {
    "dshot_bidir": "no DSHOT in SITL, so no RPM telemetry -- the sim gets rotor speed from its own motor model",
    "dyn_notch_count": "dynamic notch filter not compiled into SITL",
    "dyn_notch_q": "dynamic notch filter not compiled into SITL",
}
ABSENT_SETTING_PREFIXES = {
    "osd_": "no OSD in a SITL build",
}

# Settings that DO exist in SITL but describe the physical sensors of the real
# board. Applying them to SITL's exact virtual sensors injects an error rather
# than removing one.
BOARD_SPECIFIC_SETTINGS = {
    "acc_calibration": ("the real FC's accelerometer trim. SITL's virtual accel is exact, "
                        "so this biases every reading -- it made a level quad read "
                        "[-55, -161, 248] instead of [0, 0, 256]"),
    "acc_trim_roll": "accelerometer trim for the real board",
    "acc_trim_pitch": "accelerometer trim for the real board",
}

# A few settings read back after the reboot to prove the config actually stuck.
VERIFY = ["p_pitch", "i_pitch", "d_pitch", "p_roll", "yaw_rc_rate", "motor_pwm_protocol"]


class Cli:
    """Line-oriented client for SITL's CLI over TCP."""

    def __init__(self, host: str, port: int, timeout: float = 4.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def read_until_idle(self, idle: float = 0.35, overall: float = 4.0) -> str:
        """Read until the FC goes quiet. The CLI has no reliable end marker.

        Silence only counts as "done" once something has actually arrived --
        otherwise the first read after connecting races the FC's reply and
        returns empty, which looks exactly like a missing CLI.
        """
        self.sock.settimeout(idle)
        chunks: list[bytes] = []
        deadline = time.monotonic() + overall
        while time.monotonic() < deadline:
            try:
                data = self.sock.recv(8192)
            except socket.timeout:
                if chunks:
                    break  # quiet after a reply: done
                continue   # nothing yet: keep waiting until `overall`
            except OSError:
                break
            if not data:
                break
            chunks.append(data)
        return b"".join(chunks).decode(errors="replace")

    def send(self, line: str) -> str:
        self.sock.sendall(line.encode() + b"\r\n")
        return self.read_until_idle()


def classify(line: str) -> tuple[str, str, str]:
    """-> (action, line_to_send, note). action in send|skip|override|save."""
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return "skip", "", ""
    if stripped == "diff all":
        return "skip", "", "the echoed command, not a setting"
    if stripped == "save":
        return "save", "", "handled separately so the reboot is expected"

    head = stripped.split()[0]
    if head in SKIP_PREFIXES:
        return "skip", "", f"{head}: describes the real board, meaningless in SITL"
    if head in ABSENT_COMMANDS:
        return "skip", "", f"{head}: {ABSENT_COMMANDS[head]}"

    match = re.match(r"^set\s+([A-Za-z0-9_]+)\s*=\s*(.+)$", stripped)
    if match:
        name, value = match.group(1), match.group(2).strip()
        if name in ABSENT_SETTINGS:
            return "skip", "", f"{name}: {ABSENT_SETTINGS[name]}"
        if name in BOARD_SPECIFIC_SETTINGS:
            why = BOARD_SPECIFIC_SETTINGS[name]
            return "skip", "", f"{name}: {why if isinstance(why, str) else why[0]}"
        for prefix, why in ABSENT_SETTING_PREFIXES.items():
            if name.startswith(prefix):
                return "skip", "", f"{name}: {why}"
        if name in OVERRIDES:
            new_value, why = OVERRIDES[name]
            if new_value.lower() == value.lower():
                return "send", stripped, ""
            return "override", f"set {name} = {new_value}", f"{value} -> {new_value}: {why}"
    return "send", stripped, ""


def msp_answers(host: str, port: int, timeout: float = 2.0) -> bool:
    """Readiness probe that does NOT disturb the FC.

    Probing with `#` would work, but it ENTERS CLI MODE -- and a FC sitting in
    the CLI answers no MSP at all, so the readiness check would itself be what
    leaves SITL unusable. Ask for MSP_FC_VARIANT instead.
    """
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            sock.sendall(b"$M<" + bytes([0, 2, 2]))  # MSP_FC_VARIANT
            header = sock.recv(5)
            return len(header) == 5 and header[:3] == b"$M>"
    except OSError:
        return False


def restart_container(name: str, host: str, port: int, wait: float) -> bool:
    """Restart the container, then WAIT FOR THE CLI rather than guessing.

    A fixed sleep was the source of a false "motor_pwm_protocol = DISABLED"
    reading: the verify connected before the FC had finished loading its
    config and read a default.
    """
    print(f"\nrestarting container '{name}' (SITL exits on reboot)...")
    subprocess.run(["docker", "restart", name], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    deadline = time.monotonic() + max(wait, 30.0)
    while time.monotonic() < deadline:
        time.sleep(1.0)
        if msp_answers(host, port):
            time.sleep(1.0)  # let the config finish settling
            return True
    print("  SITL did not come back answering MSP", file=sys.stderr)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--diff", default=str(REPO_ROOT / "config" / "diff_all.txt"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5761)
    parser.add_argument("--dry-run", action="store_true", help="print the plan, send nothing")
    parser.add_argument("--no-save", action="store_true", help="apply but do not save or reboot")
    parser.add_argument("--restart-container", metavar="NAME", default=None,
                        help="docker container to restart after the save-triggered reboot")
    parser.add_argument("--restart-wait", type=float, default=8.0)
    args = parser.parse_args()

    diff_path = Path(args.diff)
    if not diff_path.is_file():
        print(f"no such file: {diff_path}", file=sys.stderr)
        return 2

    plan = [(raw, *classify(raw)) for raw in diff_path.read_text().splitlines()]
    to_send = [p for p in plan if p[1] in ("send", "override")]
    skipped = [p for p in plan if p[1] == "skip" and p[3]]
    overridden = [p for p in plan if p[1] == "override"]

    print(f"{diff_path.name}: {len(to_send)} lines to apply, "
          f"{len(overridden)} overridden, {len(skipped)} skipped as board-specific\n")

    if overridden:
        print("OVERRIDDEN -- the sim will differ from your quad here:")
        for raw, _, sent, note in overridden:
            print(f"  {raw.strip()}\n      -> {sent}   ({note})")
        print()
    if skipped:
        print("SKIPPED:")
        for raw, _, _, note in skipped:
            print(f"  {raw.strip():<40} {note}")
        print()

    if args.dry_run:
        print("dry run: nothing was sent.")
        return 0

    # Wait for the FC to finish booting before touching it. The TCP port binds
    # early, so a bare connect() succeeds long before MSP is answering and the
    # CLI would just time out with a confusing "is something else on this
    # port?".
    waited = 0.0
    while not msp_answers(args.host, args.port) and waited < 30.0:
        time.sleep(1.0)
        waited += 1.0
    if waited >= 30.0:
        print(f"SITL at {args.host}:{args.port} never started answering MSP.", file=sys.stderr)
        print("is ./tools/run_sitl.sh running?", file=sys.stderr)
        return 2
    if waited:
        print(f"(waited {waited:.0f}s for SITL to finish booting)\n")

    try:
        cli = Cli(args.host, args.port)
    except OSError as exc:
        print(f"cannot reach SITL at {args.host}:{args.port}: {exc}", file=sys.stderr)
        return 2

    banner = cli.send("#")
    if "CLI" not in banner and "#" not in banner:
        print("did not get a CLI prompt; is something else on this port?", file=sys.stderr)
        cli.close()
        return 2

    rejected: list[tuple[str, str]] = []
    for _, action, sent, _ in plan:
        if action not in ("send", "override"):
            continue
        reply = cli.send(sent)
        if ERROR_MARKER in reply:
            detail = next((ln.strip() for ln in reply.splitlines() if ERROR_MARKER in ln), reply.strip())
            rejected.append((sent, detail))

    print(f"applied {len(to_send)} lines, {len(rejected)} rejected by Betaflight")
    if rejected:
        print("\nREJECTED -- these did not take:")
        for sent, detail in rejected:
            print(f"  {sent}\n      {detail}")

    if args.no_save:
        print("\n--no-save: config applied to RAM only, not written to eeprom.")
        cli.close()
        return 1 if rejected else 0

    # The diff opens a command batch (`batch start`) and never closes it -- the
    # real FC's `save` ends it implicitly. Close it explicitly: inside an open
    # batch the CLI rejects other commands as UNKNOWN COMMAND, which is
    # baffling to debug later.
    batch_reply = cli.send("batch end")
    if "batch ended" not in batch_reply.lower() and ERROR_MARKER in batch_reply:
        print("note: `batch end` was rejected; the diff may not have opened one")

    print("\nsaving (this reboots the FC, which exits the SITL process)...")
    cli.send("save")
    cli.close()

    if args.restart_container:
        if not restart_container(args.restart_container, args.host, args.port, args.restart_wait):
            return 2
    else:
        print("SITL has exited. Restart it, then re-run with --no-save to verify,")
        print("or pass --restart-container fdt-sitl next time.")
        return 1 if rejected else 0

    # --- verify the config survived the reboot ---
    print("\nverifying against the FC:")
    try:
        cli = Cli(args.host, args.port)
    except OSError as exc:
        print(f"  cannot reconnect: {exc}", file=sys.stderr)
        return 2

    cli.send("#")
    wanted = {}
    for raw, action, sent, _ in plan:
        m = re.match(r"^set\s+([A-Za-z0-9_]+)\s*=\s*(.+)$", sent) if action in ("send", "override") else None
        if m:
            wanted[m.group(1)] = m.group(2).strip()

    def read_setting(name: str) -> str | None:
        """Read one setting, insisting the reply is actually for this command.

        The CLI has no request ids, so a laggy reply can otherwise be paired
        with the wrong `get`.
        """
        for _ in range(3):
            reply = cli.send(f"get {name}")
            if f"get {name}" not in reply:
                continue
            for line in reply.splitlines():
                if line.strip().startswith(f"{name} ="):
                    return line.split("=", 1)[1].strip()
        return None

    mismatches = 0
    for name in VERIFY:
        got = read_setting(name)
        expect = wanted.get(name)
        if got is None:
            print(f"  {name:<22} (no reading)")
            mismatches += 1
        elif expect is not None and got.lower() != expect.lower():
            print(f"  {name:<22} {got:<12} MISMATCH, expected {expect}")
            mismatches += 1
        else:
            print(f"  {name:<22} {got}")
    cli.close()

    # Verifying had to enter CLI mode, and in SITL leaving it (`exit`) reboots
    # just as `save` does. While the FC sits in CLI it will not answer MSP, so
    # the physics side and the Configurator would both find it dead. Restart
    # once more to leave it clean -- the config is already saved, so this
    # costs nothing.
    print("\nleaving SITL out of CLI mode so it answers MSP...")
    if not restart_container(args.restart_container, args.host, args.port, args.restart_wait):
        return 2

    if rejected or mismatches:
        print(f"\n{len(rejected)} rejected, {mismatches} mismatched.")
        return 1
    print("\nSITL is running your tune, and is answering MSP.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
