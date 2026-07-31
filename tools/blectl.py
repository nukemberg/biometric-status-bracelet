#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["bleak>=0.22"]
# ///
"""Read the bracelet over BLE, without a USB tether.

Motivation: capturing over USB reboots the board on every connect, so nothing settled
is ever observed, and it requires the wearer to sit next to the laptop. BLE avoids
both.

Examples:
    tools/blectl.py selftest
    tools/blectl.py scan
    tools/blectl.py info
    tools/blectl.py monitor --seconds 30
    tools/blectl.py monitor --csv vitals.csv
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import bracelet_protocol as proto  # noqa: E402
from bleak import BleakClient, BleakScanner  # noqa: E402

DEVICE_NAME = "Bracelet"


async def find_device(name: str, timeout: float):
    """Prefer matching on the service UUID.

    macOS CoreBluetooth caches GAP names aggressively and will happily report a stale
    one from whatever the board ran previously, so the advertised service is the more
    trustworthy identifier.
    """
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: proto.SVC_BRACELET.lower()
        in [u.lower() for u in ad.service_uuids],
        timeout=timeout,
    )
    if dev is None:
        dev = await BleakScanner.find_device_by_name(name, timeout=timeout / 2)
    return dev


async def cmd_scan(args) -> int:
    print(f"scanning {args.timeout:g}s for the bracelet service...")
    seen: dict[str, tuple] = {}

    def cb(d, ad):
        if proto.SVC_BRACELET.lower() in [u.lower() for u in ad.service_uuids]:
            seen[d.address] = (ad.local_name or d.name, ad.rssi)

    async with BleakScanner(cb):
        await asyncio.sleep(args.timeout)

    if not seen:
        print("no bracelet found", file=sys.stderr)
        return 1
    for addr, (name, rssi) in seen.items():
        print(f"{addr}  {name!r}  rssi {rssi}")
    return 0


async def cmd_info(args) -> int:
    dev = await find_device(args.name, args.timeout)
    if dev is None:
        print("bracelet not found", file=sys.stderr)
        return 1
    async with BleakClient(dev) as c:
        raw = await c.read_gatt_char(proto.CHR_INFO)
        info = raw.decode(errors="replace")
        print(f"address:  {dev.address}")
        print(f"info:     {info}")
        for field in info.split(";"):
            if field.startswith("proto="):
                dev_proto = int(field.split("=", 1)[1])
                if dev_proto != proto.PROTOCOL_VERSION:
                    print(
                        f"\nWARNING: device speaks protocol v{dev_proto}, this tool "
                        f"speaks v{proto.PROTOCOL_VERSION}. Decodes will be rejected.",
                        file=sys.stderr,
                    )
                    return 2
    return 0


async def cmd_monitor(args) -> int:
    dev = await find_device(args.name, args.timeout)
    if dev is None:
        print("bracelet not found", file=sys.stderr)
        return 1
    print(f"# {dev.address}", file=sys.stderr)

    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("time_s,bpm,confidence,arousal,perfusion,gsr_raw,pulse_raw,"
                  "temp_c,gsr_tonic,worn,pulse_trusted,mode\n")

    count = 0
    errors = 0
    started = time.monotonic()

    def on_vitals(_handle, data: bytearray):
        nonlocal count, errors
        try:
            v = proto.decode_vitals(bytes(data))
        except proto.ProtocolError as e:
            errors += 1
            print(f"  {e}", file=sys.stderr)
            return
        count += 1
        t = time.monotonic() - started
        if not args.quiet:
            print(f"{t:7.2f}  {v.format()}", flush=True)
        if csv:
            csv.write(
                f"{t:.3f},{v.bpm:.1f},{v.confidence:.3f},{v.arousal:.3f},"
                f"{v.perfusion:.2f},{v.gsr_raw},{v.pulse_raw},{v.temp_c:.2f},"
                f"{v.gsr_tonic:.0f},{int(v.worn)},{int(v.pulse_trusted)},{v.mode}\n"
            )
            csv.flush()

    try:
        async with BleakClient(dev) as c:
            await c.start_notify(proto.CHR_VITALS, on_vitals)
            if args.seconds > 0:
                await asyncio.sleep(args.seconds)
            else:
                while True:
                    await asyncio.sleep(3600)
            await c.stop_notify(proto.CHR_VITALS)
    except KeyboardInterrupt:
        pass
    finally:
        if csv:
            csv.close()

    elapsed = time.monotonic() - started
    rate = count / elapsed if elapsed > 0 else 0
    print(f"# {count} notifications in {elapsed:.1f}s = {rate:.2f} Hz"
          f"{f', {errors} decode errors' if errors else ''}", file=sys.stderr)
    return 1 if errors and not count else 0


def cmd_selftest(args) -> int:
    return 1 if proto.selftest() else 0


def main() -> int:
    # Shared options live on a parent parser so they work after the subcommand,
    # which is where anyone would naturally type them.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--name", default=DEVICE_NAME, help="advertised name fallback")
    common.add_argument("--timeout", type=float, default=15.0, help="scan timeout")

    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[common],
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("selftest", help="decode the C++ golden fixtures, no device needed")
    sub.add_parser("scan", parents=[common], help="list bracelets in range")
    sub.add_parser("info", parents=[common], help="read the Info characteristic")

    m = sub.add_parser("monitor", parents=[common], help="stream decoded vitals")
    m.add_argument("--seconds", type=float, default=0.0, help="0 = until interrupted")
    m.add_argument("--csv", help="also write decoded vitals here")
    m.add_argument("--quiet", action="store_true")

    args = ap.parse_args()

    if args.cmd == "selftest":
        return cmd_selftest(args)
    handler = {"scan": cmd_scan, "info": cmd_info, "monitor": cmd_monitor}[args.cmd]
    try:
        return asyncio.run(handler(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
