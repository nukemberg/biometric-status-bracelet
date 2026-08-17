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
    tools/blectl.py stream --seconds 60 --csv signals.csv
    tools/blectl.py spectrum --seconds 30
    tools/blectl.py cmd mode 1
    tools/blectl.py cmd brightness 200
    tools/blectl.py cmd recalibrate
    tools/blectl.py config get
    tools/blectl.py config set pi_trust_min 0.45
    tools/blectl.py config reset
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


async def cmd_stream(args) -> int:
    """25 Hz signals. Opt-in, so the stream mask is set on connect and cleared after."""
    dev = await find_device(args.name, args.timeout)
    if dev is None:
        print("bracelet not found", file=sys.stderr)
        return 1

    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("timestamp_ms,pulse_filtered,gsr_phasic,pulse_raw,gsr_raw\n")

    samples = 0
    packets = 0
    errors = 0
    last_ts = None
    gaps = 0

    def on_signals(_h, data: bytearray):
        nonlocal samples, packets, errors, last_ts, gaps
        try:
            ts, batch = proto.decode_signals(bytes(data))
        except proto.ProtocolError as e:
            errors += 1
            print(f"  {e}", file=sys.stderr)
            return
        packets += 1
        # 25 Hz means 40 ms per sample; a batch of five should advance 200 ms. Anything
        # much larger is a dropped sample, which must not pass as continuous data.
        if last_ts is not None and ts - last_ts > 260:
            gaps += 1
            print(f"  gap: {ts - last_ts} ms since previous batch", file=sys.stderr)
        last_ts = ts
        for i, smp in enumerate(batch):
            samples += 1
            t = ts + i * 40
            if not args.quiet:
                print(f"{t:9d}  pulse {smp.pulse_filtered:9.1f}  "
                      f"gsr {smp.gsr_phasic:8.1f}  ir {smp.pulse_raw:7d} "
                      f"{smp.gsr_raw:5d}", flush=True)
            if csv:
                csv.write(f"{t},{smp.pulse_filtered:.1f},{smp.gsr_phasic:.1f},"
                          f"{smp.pulse_raw},{smp.gsr_raw}\n")
        if csv:
            csv.flush()

    started = time.monotonic()
    try:
        async with BleakClient(dev) as c:
            await c.write_gatt_char(
                proto.CHR_CONTROL,
                proto.encode_control(proto.CMD_SET_STREAMS, proto.STREAM_SIGNALS),
                response=True,
            )
            await c.start_notify(proto.CHR_SIGNALS, on_signals)
            await asyncio.sleep(args.seconds if args.seconds > 0 else 3600)
            await c.stop_notify(proto.CHR_SIGNALS)
            await c.write_gatt_char(
                proto.CHR_CONTROL,
                proto.encode_control(proto.CMD_SET_STREAMS, 0), response=True)
    except KeyboardInterrupt:
        pass
    finally:
        if csv:
            csv.close()

    elapsed = time.monotonic() - started
    print(f"# {samples} samples in {packets} packets over {elapsed:.1f}s = "
          f"{samples/elapsed:.1f} Hz (expect ~25)"
          f"{f', {gaps} gaps' if gaps else ''}"
          f"{f', {errors} decode errors' if errors else ''}", file=sys.stderr)
    return 0


async def cmd_spectrum(args) -> int:
    """48-bin resonator power. This is the view that shows harmonic capture live."""
    dev = await find_device(args.name, args.timeout)
    if dev is None:
        print("bracelet not found", file=sys.stderr)
        return 1

    count = 0
    csv = open(args.csv, "w") if args.csv else None
    if csv:
        # One row per spectrum: time, peak bpm, peak bin, then the 48 raw bin powers
        # (0-255, peak-normalised -- see ble_protocol.h). Not physical units, but
        # consistent frame to frame, which is what matters for watching a harmonic
        # gain on a fundamental over time.
        csv.write("time_s,peak_bpm,peak_bin," +
                  ",".join(f"bin{i}" for i in range(proto.SPECTRUM_BINS)) + "\n")
    started = time.monotonic()

    def on_spectrum(_h, data: bytearray):
        nonlocal count
        try:
            sp = proto.decode_spectrum(bytes(data))
        except proto.ProtocolError as e:
            print(f"  {e}", file=sys.stderr)
            return
        count += 1
        if csv:
            t = time.monotonic() - started
            row = [f"{t:.2f}", f"{sp.peak_bpm:.1f}", str(sp.peak_bin)]
            row += [str(v) for v in sp.bins]
            # Pad to the fixed column count if the device ever reports fewer bins,
            # so every row has the same width regardless of what any one packet said.
            while len(row) < 3 + proto.SPECTRUM_BINS:
                row.append("")
            csv.write(",".join(row) + "\n")
            csv.flush()
        if args.bars:
            print(f"\npeak {sp.peak_bpm:.1f} BPM (bin {sp.peak_bin})")
            for i, v in enumerate(sp.bins):
                if v < args.floor:
                    continue
                bar = "#" * int(v / 255 * 50)
                mark = " <-- peak" if i == sp.peak_bin else ""
                print(f"  {sp.bpm_of_bin(i):6.1f} {v:4d} {bar}{mark}")
        else:
            top = sorted(range(len(sp.bins)), key=lambda i: sp.bins[i], reverse=True)[:4]
            desc = "  ".join(f"{sp.bpm_of_bin(i):.0f}={sp.bins[i]}" for i in top)
            print(f"peak {sp.peak_bpm:6.1f}  |  {desc}", flush=True)

    try:
        async with BleakClient(dev) as c:
            await c.write_gatt_char(
                proto.CHR_CONTROL,
                proto.encode_control(proto.CMD_SET_STREAMS, proto.STREAM_SPECTRUM),
                response=True,
            )
            await c.start_notify(proto.CHR_SPECTRUM, on_spectrum)
            await asyncio.sleep(args.seconds if args.seconds > 0 else 3600)
            await c.stop_notify(proto.CHR_SPECTRUM)
            await c.write_gatt_char(
                proto.CHR_CONTROL,
                proto.encode_control(proto.CMD_SET_STREAMS, 0), response=True)
    except KeyboardInterrupt:
        pass
    if csv:
        csv.close()
    print(f"# {count} spectra", file=sys.stderr)
    return 0


CMDS = {
    "mode": (proto.CMD_SET_MODE, True),
    "brightness": (proto.CMD_SET_BRIGHTNESS, True),
    "recalibrate": (proto.CMD_RECALIBRATE_GSR, False),
    "reset-bank": (proto.CMD_RESET_BANK, False),
    "reset-config": (proto.CMD_RESET_CONFIG, False),
}


async def cmd_cmd(args) -> int:
    if args.command not in CMDS:
        print(f"unknown command; known: {', '.join(CMDS)}", file=sys.stderr)
        return 2
    code, needs_arg = CMDS[args.command]
    if needs_arg and args.value is None:
        print(f"{args.command} needs a value", file=sys.stderr)
        return 2
    dev = await find_device(args.name, args.timeout)
    if dev is None:
        print("bracelet not found", file=sys.stderr)
        return 1
    payload = proto.encode_control(code, args.value if needs_arg else None)
    async with BleakClient(dev) as c:
        await c.write_gatt_char(proto.CHR_CONTROL, payload, response=True)
    print(f"sent {args.command}" + (f" {args.value}" if needs_arg else ""))
    return 0


async def cmd_config(args) -> int:
    """Read all tunables, set one by name, or reset to compiled defaults.

    The config characteristic (-pmw) carries the same [paramId][f32] record in both
    directions: a write is one record, a read is all of them. A set takes effect on
    the next DSP tick (~40 ms) and a read packs live state, so `set` then `get`
    round-trips immediately; the debounced NVS save only affects reboot persistence.
    """
    if args.action == "reset":
        dev = await find_device(args.name, args.timeout)
        if dev is None:
            print("bracelet not found", file=sys.stderr)
            return 1
        async with BleakClient(dev) as c:
            await c.write_gatt_char(
                proto.CHR_CONTROL,
                proto.encode_control(proto.CMD_RESET_CONFIG), response=True)
        print("reset config to compiled defaults")
        return 0

    if args.action == "set":
        try:
            payload = proto.encode_config_write(args.param, args.value)
        except proto.ProtocolError as e:
            print(e, file=sys.stderr)
            return 2
        dev = await find_device(args.name, args.timeout)
        if dev is None:
            print("bracelet not found", file=sys.stderr)
            return 1
        async with BleakClient(dev) as c:
            await c.write_gatt_char(proto.CHR_CONFIG, payload, response=True)
        print(f"set {args.param} = {args.value}")
        return 0

    # get
    dev = await find_device(args.name, args.timeout)
    if dev is None:
        print("bracelet not found", file=sys.stderr)
        return 1
    async with BleakClient(dev) as c:
        raw = await c.read_gatt_char(proto.CHR_CONFIG)
    try:
        cfg = proto.decode_config_read(bytes(raw))
    except proto.ProtocolError as e:
        print(e, file=sys.stderr)
        return 1
    if args.csv:
        print("name,value")
        for name, value in cfg.items():
            print(f"{name},{value}")
    else:
        w = max(len(n) for n in cfg)
        for name, value in cfg.items():
            print(f"{name:<{w}} = {value:g}")
    return 0


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

    st = sub.add_parser("stream", parents=[common], help="25 Hz filtered signals")
    st.add_argument("--seconds", type=float, default=0.0)
    st.add_argument("--csv", help="write samples here")
    st.add_argument("--quiet", action="store_true")

    sp = sub.add_parser("spectrum", parents=[common], help="48-bin resonator power")
    sp.add_argument("--seconds", type=float, default=0.0)
    sp.add_argument("--bars", action="store_true", help="full bar chart per spectrum")
    sp.add_argument("--floor", type=int, default=20, help="hide bins below this")
    sp.add_argument("--csv", help="write a 48-column CSV, one row per spectrum")

    cm = sub.add_parser("cmd", parents=[common], help="send a control command")
    cm.add_argument("command", choices=sorted(CMDS))
    cm.add_argument("value", nargs="?", type=int)

    cf = sub.add_parser("config", help="read/set tunables over BLE")
    cf_sub = cf.add_subparsers(dest="action", required=True)
    cf_get = cf_sub.add_parser("get", parents=[common], help="read all tunables")
    cf_get.add_argument("--csv", action="store_true", help="write name,value rows")
    cf_set = cf_sub.add_parser(
        "set", parents=[common],
        help=f"set one tunable (one of: {', '.join(sorted(proto.CONFIG_PARAMS))})")
    cf_set.add_argument(
        "param",
        help=f"parameter name (one of: {', '.join(sorted(proto.CONFIG_PARAMS))})")
    cf_set.add_argument("value", type=float)
    cf_sub.add_parser("reset", parents=[common], help="restore compiled defaults")

    args = ap.parse_args()

    if args.cmd == "selftest":
        return cmd_selftest(args)
    handler = {"scan": cmd_scan, "info": cmd_info, "monitor": cmd_monitor,
               "stream": cmd_stream, "spectrum": cmd_spectrum, "cmd": cmd_cmd,
               "config": cmd_config}[args.cmd]
    try:
        return asyncio.run(handler(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
