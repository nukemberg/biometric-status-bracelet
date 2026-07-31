#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5"]
# ///
"""Capture serial output from the bracelet.

Opening a serial port normally asserts DTR/RTS, which drives the ESP32's auto-reset
circuit and reboots the board. That makes every naive capture start from a cold boot,
so nothing settled state ever appears: the GSR tonic EMA needs ~45 s to converge and
the resonator bank ~10 s. `--no-reset` tries to suppress that.

Examples:
    tools/capture.py --seconds 60
    tools/capture.py --only Jitter --seconds 30
    tools/capture.py --list

`--no-reset` is best-effort and does NOT work on the CP210x/CH340 adapters used here:
the auto-reset fires at the driver level before pyserial can lower the lines. The
reliable way to observe settled state is to hold the port open --

    tools/capture.py --seconds 0 --out /tmp/session.log --quiet &

-- which resets the board once at start and then logs continuously; read the tail of
that file whenever you need current values. Note the port is then busy, so stop the
logger before flashing.
"""

from __future__ import annotations

import argparse
import glob
import subprocess
import sys
import time

import serial

DEFAULT_BAUD = 115200

# Printed by the FastLED driver during setup(), so their presence in a capture means
# the board rebooted when we opened the port.
BOOT_MARKERS = ("ChannelManager:", "RMT Memory", "[RMT TX]")


def find_ports() -> list[str]:
    return sorted(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*"))


def pick_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = find_ports()
    if not ports:
        sys.exit("no USB serial port found; pass --port")
    if len(ports) > 1:
        sys.exit(f"multiple ports found, pass --port: {', '.join(ports)}")
    return ports[0]


def open_serial(port: str, baud: int, no_reset: bool) -> serial.Serial:
    """Open the port, optionally without triggering the ESP32 auto-reset.

    Two things drive the reset: the DTR/RTS transition when the port is opened, and
    HUPCL asserting DTR again when it is closed. We clear HUPCL via stty first, then
    set both lines low *before* open() so pyserial applies them as it configures the
    port rather than after.
    """
    if no_reset:
        # Best-effort; a driver that ignores this just means we get a reboot.
        subprocess.run(["stty", "-f", port, "-hupcl"], check=False,
                       capture_output=True)
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 1
        ser.dtr = False
        ser.rts = False
        ser.open()
        return ser
    return serial.Serial(port, baud, timeout=1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--seconds", type=float, default=30.0,
                    help="capture duration; 0 runs until interrupted, which is how "
                         "to keep the port open and observe settled state")
    ap.add_argument("--out", help="write lines to this file as well as stdout")
    ap.add_argument("--only", action="append", default=[],
                    help="keep only lines containing this substring (repeatable)")
    ap.add_argument("--no-reset", action="store_true",
                    help="try not to reboot the board on open")
    ap.add_argument("--quiet", action="store_true", help="do not echo to stdout")
    ap.add_argument("--list", action="store_true", help="list candidate ports and exit")
    args = ap.parse_args()

    if args.list:
        for p in find_ports():
            print(p)
        return 0

    port = pick_port(args.port)
    ser = open_serial(port, args.baud, args.no_reset)

    # Discard whatever was already buffered so the capture starts clean.
    time.sleep(0.3)
    ser.reset_input_buffer()

    out = open(args.out, "w") if args.out else None
    lines = 0
    rebooted = False
    started = time.monotonic()

    duration = "until interrupted" if args.seconds == 0 else f"{args.seconds:g}s"
    print(f"# {port} @ {args.baud}, {duration}"
          f"{', no-reset' if args.no_reset else ''}", file=sys.stderr)

    try:
        while args.seconds == 0 or time.monotonic() - started < args.seconds:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").rstrip("\r\n")
            if not line:
                continue
            if any(m in line for m in BOOT_MARKERS):
                rebooted = True
            if args.only and not any(k in line for k in args.only):
                continue
            lines += 1
            if not args.quiet:
                print(line, flush=True)
            if out:
                out.write(line + "\n")
                out.flush()   # so the file can be tailed while the logger runs
    except KeyboardInterrupt:
        print("\n# interrupted", file=sys.stderr)
    finally:
        ser.close()
        if out:
            out.close()

    elapsed = time.monotonic() - started
    print(f"# {lines} lines in {elapsed:.1f}s", file=sys.stderr)
    if rebooted:
        print("# NOTE: board rebooted on open — this capture starts from cold boot,"
              " so GSR tonic (45s) and the resonator bank (10s) are still settling",
              file=sys.stderr)
    elif args.no_reset:
        print("# no reboot detected — capture is of already-running state",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
