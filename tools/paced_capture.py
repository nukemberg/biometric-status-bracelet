#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5"]
# ///
"""Record a raw_streamer capture while pacing the wearer's breathing.

Breath detection needs ground truth, and a capture of someone breathing however they
felt like is not ground truth. This drives a metronome in the terminal and writes the
cue timings into the SAME csv as comment rows, so the analysis never has to align two
files recorded against two clocks.

    tools/paced_capture.py --out samples/breath_paced_01.csv

Requires firmware/raw_streamer (Timestamp_ms,RawIr,RawGSR,IrNew) -- main_armband emits
its own log format and is rejected on sight.

Markers look like:

    # MARK dev_ms=48212 host_s=61.004 stage=paced6 idx=1 event=in rate=6.0

`dev_ms` is the device timestamp of the last row read before the cue fired, so every
marker is anchored on the device's clock rather than the host's. Consumers that split
on ',' see a single field and skip the line (tools/dsp_v2_sim.py already does), so a
paced capture stays a valid ordinary capture.

The default plan runs ~6 min:

    settle 60s  -> paced 6/min 60s -> rest 30s -> paced 12/min 60s
                -> rest 30s -> hold 30s -> free 60s

Two paced rates, because a tracker that merely latched onto one frequency passes a
single-rate test. The hold stage is the falsifier: a genuine respiratory component in
the GSR phasic signal has to disappear while the breath does.

A `sigh` stage cues single deep breaths spaced far apart instead of a rhythm --
`sigh:1:240` is one every 60 s for four minutes. Steady breathing and isolated deep
breaths are different provocations: paced captures 01 and 02 found no GSR response to
the first, while DESIGN.md section 5 reports a clear one to the second. The two stage
types exist to keep that distinction measurable rather than argued.

Sit still. Motion is the confound that already produced a false harmonic lock in the
PPG work (dd9b2d8), and it will happily fake a 0.2 Hz component too.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import capture  # noqa: E402  (sibling script; reuse its port handling)

DEFAULT_PLAN = "settle:60,paced:6:60,rest:30,paced:12:60,rest:30,hold:30,free:60"
EXPECTED_HEADER = "Timestamp_ms,RawIr,RawGSR,IrNew"

# How long to wait for the board to identify itself before giving up. The ESP32
# reboots when the port opens, and its setup() does I2C bring-up first, so the header
# does not arrive instantly.
HEADER_TIMEOUT_S = 8.0

CUE_HZ = 20.0
BAR_CELLS = 14

# A sigh cue is a deep breath in, then all the way out, then back to normal. The
# response being provoked is an SCR with a 1-3 s latency, so the interval that
# follows has to stay quiet long enough to see it rise and recover.
SIGH_IN_S = 3.0
SIGH_OUT_S = 4.0

STATIC_CUES = {
    "settle": "sit still, breathe freely (DSP settling)",
    "rest":   "rest, breathe freely",
    "free":   "breathe freely",
    "hold":   "exhale, then HOLD -- release if uncomfortable",
}


@dataclass
class Stage:
    kind: str
    dur: float
    rate: float = 0.0    # breaths/min, paced stages only

    @property
    def name(self) -> str:
        if self.kind == "paced":
            return f"paced{self.rate:g}"
        if self.kind == "sigh":
            return f"sigh{self.rate:g}"
        return self.kind


def parse_plan(spec: str) -> list[Stage]:
    stages: list[Stage] = []
    for tok in spec.split(","):
        parts = tok.strip().split(":")
        kind = parts[0]
        try:
            if kind in ("paced", "sigh"):
                stages.append(Stage(kind, float(parts[2]), float(parts[1])))
            elif kind in STATIC_CUES:
                stages.append(Stage(kind, float(parts[1])))
            else:
                sys.exit(f"unknown stage '{kind}' in --plan")
        except (IndexError, ValueError):
            sys.exit(f"bad stage spec '{tok}'; want kind:seconds, paced:rate:seconds "
                     f"or sigh:per_minute:seconds")
    if not stages:
        sys.exit("--plan is empty")
    return stages


def build_events(stages: list[Stage]) -> list[tuple[float, int, str]]:
    """(offset_seconds, stage_index, event) for every cue, in time order.

    Paced stages emit an in/out event every half period. A stage_start is emitted
    for all stages including paced ones, so a consumer can find stage boundaries
    without having to reconstruct them from the breath events.
    """
    events: list[tuple[float, int, str]] = []
    t = 0.0
    for i, st in enumerate(stages):
        events.append((t, i, "stage_start"))
        if st.kind == "paced":
            half = 30.0 / st.rate          # period = 60/rate; half of it per phase
            k = 0
            while t + k * half < t + st.dur - 1e-6:
                events.append((t + k * half, i, "in" if k % 2 == 0 else "out"))
                k += 1
        elif st.kind == "sigh":
            # One cue per interval and nothing in between: a sigh is an isolated
            # event whose response is scored against the quiet that precedes it,
            # not a rhythm to fold. The refractory note in DESIGN.md is why the
            # default spacing is a whole minute.
            gap = 60.0 / st.rate
            k = 0
            while t + k * gap < t + st.dur - 1e-6:
                events.append((t + k * gap, i, "sigh"))
                k += 1
        t += st.dur
    events.append((t, len(stages) - 1, "end"))
    return events


def render_cue(stages: list[Stage], idx: int, in_stage: float, remaining: float) -> str:
    st = stages[idx]
    tail = f"stage {idx + 1}/{len(stages)}  {int(remaining) // 60}:{int(remaining) % 60:02d} left"

    if st.kind == "sigh":
        gap = 60.0 / st.rate
        since = in_stage % gap
        if since < SIGH_IN_S:
            frac = since / SIGH_IN_S
            bar = "█" * int(round(frac * BAR_CELLS))
            body = f"DEEP BREATH IN  {bar:<{BAR_CELLS}}"
        elif since < SIGH_IN_S + SIGH_OUT_S:
            frac = 1.0 - (since - SIGH_IN_S) / SIGH_OUT_S
            bar = "█" * int(round(frac * BAR_CELLS))
            body = f"all the way OUT {bar:<{BAR_CELLS}}"
        else:
            body = f"relax, breathe normally -- next in {gap - since:4.0f}s"
        return f"[   sigh x{st.rate:g}/m  ]  {body:<42}  {tail}"

    if st.kind != "paced":
        return f"[ {st.name:^11} ]  {STATIC_CUES[st.kind]:<42}  {tail}"

    period = 60.0 / st.rate
    phase = (in_stage % period) / period
    inhaling = phase < 0.5
    frac = phase * 2.0 if inhaling else 1.0 - (phase - 0.5) * 2.0
    filled = int(round(frac * BAR_CELLS))
    bar = "█" * filled + "░" * (BAR_CELLS - filled)
    left = (0.5 - phase if inhaling else 1.0 - phase) * period
    word = "IN " if inhaling else "OUT"
    return f"[ {st.rate:g} br/min ]  {word} {bar} {left:4.1f}s{'':>16}  {tail}"


def wait_for_header(ser, out, quiet: bool) -> None:
    """Confirm this is a raw_streamer capture before we waste the wearer's 6 minutes.

    A three-column analog capture or a main_armband log would produce a full run and
    a file that only fails much later, in the analysis.
    """
    deadline = time.monotonic() + HEADER_TIMEOUT_S
    while time.monotonic() < deadline:
        line = ser.readline().decode("utf-8", "replace").strip()
        if not line:
            continue
        if line == EXPECTED_HEADER:
            out.write(line + "\n")
            return
        if line.startswith("#"):
            if not quiet:
                print(f"# device: {line}", file=sys.stderr)
            continue
        parts = line.split(",")
        # Already-running board (--no-reset): no header will come, so accept a
        # well-formed data row as proof of the right firmware instead.
        if len(parts) == 4 and all(p.strip().lstrip("-").isdigit() for p in parts):
            out.write(EXPECTED_HEADER + "\n")
            out.write(line + "\n")
            return
        if len(parts) == 3:
            sys.exit("device is emitting a 3-column analog capture; flash "
                     "firmware/raw_streamer")
    sys.exit(f"no '{EXPECTED_HEADER}' header within {HEADER_TIMEOUT_S:g}s; "
             "is firmware/raw_streamer flashed?")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="csv to write")
    ap.add_argument("--port", help="serial device (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=capture.DEFAULT_BAUD)
    ap.add_argument("--plan", default=DEFAULT_PLAN,
                    help=f"stage list (default: {DEFAULT_PLAN})")
    ap.add_argument("--countdown", type=float, default=5.0,
                    help="seconds between opening the port and the first cue")
    ap.add_argument("--no-reset", action="store_true",
                    help="try not to reboot the board on open")
    ap.add_argument("--no-beep", action="store_true", help="visual cues only")
    ap.add_argument("--quiet", action="store_true", help="no live cue line")
    ap.add_argument("--list", action="store_true", help="list candidate ports and exit")
    args = ap.parse_args()

    if args.list:
        for p in capture.find_ports():
            print(p)
        return 0

    stages = parse_plan(args.plan)
    events = build_events(stages)
    total = sum(s.dur for s in stages)

    port = capture.pick_port(args.port)
    ser = capture.open_serial(port, args.baud, args.no_reset)
    # capture.open_serial uses a 1 s timeout, which would stall the metronome for a
    # whole second whenever the stream hiccups. The cue loop needs to come back around
    # at CUE_HZ regardless of what the device is doing.
    ser.timeout = 0.02

    out = open(args.out, "w")
    print(f"# {port} @ {args.baud} -> {args.out}", file=sys.stderr)
    print(f"# plan: {args.plan}  ({total:.0f}s total)", file=sys.stderr)

    wait_for_header(ser, out, args.quiet)

    for k in range(int(args.countdown), 0, -1):
        print(f"\rstarting in {k}... ", end="", file=sys.stderr, flush=True)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            ser.readline()          # keep the port drained; pre-start rows are noise
    print("\r" + " " * 30 + "\r", end="", file=sys.stderr, flush=True)

    started = time.monotonic()
    dev_ms = 0                      # last device timestamp seen
    rows = 0
    next_event = 0
    next_cue = 0.0
    stage_starts = [0.0]
    for st in stages[:-1]:
        stage_starts.append(stage_starts[-1] + st.dur)

    def mark(idx: int, event: str, host_s: float) -> None:
        st = stages[idx]
        out.write(f"# MARK dev_ms={dev_ms} host_s={host_s:.3f} stage={st.name} "
                  f"idx={idx} event={event} rate={st.rate:g}\n")

    try:
        while True:
            raw = ser.readline()
            if raw:
                line = raw.decode("utf-8", "replace").rstrip("\r\n")
                if line:
                    out.write(line + "\n")
                    rows += 1
                    head = line.split(",", 1)[0]
                    if head.isdigit():
                        dev_ms = int(head)

            now = time.monotonic() - started

            while next_event < len(events) and events[next_event][0] <= now:
                at, idx, ev = events[next_event]
                mark(idx, ev, at)
                if ev == "stage_start" and not args.quiet:
                    st = stages[idx]
                    if st.kind == "paced":
                        label = f"{st.rate:g} br/min"
                    elif st.kind == "sigh":
                        label = f"one deep breath every {60.0 / st.rate:g}s"
                    else:
                        label = STATIC_CUES[st.kind]
                    print(f"\r{'':<78}\r# {int(at) // 60}:{int(at) % 60:02d}  "
                          f"{st.name} -- {label} ({st.dur:g}s)",
                          file=sys.stderr, flush=True)
                if ev in ("in", "out", "sigh", "stage_start") and not args.no_beep:
                    print("\a", end="", file=sys.stderr, flush=True)
                next_event += 1

            if now >= total:
                break

            if not args.quiet and now >= next_cue:
                next_cue = now + 1.0 / CUE_HZ
                idx = max(i for i, s in enumerate(stage_starts) if s <= now)
                cue = render_cue(stages, idx, now - stage_starts[idx], total - now)
                print(f"\r{cue:<100}", end="", file=sys.stderr, flush=True)
    except KeyboardInterrupt:
        print("\n# interrupted", file=sys.stderr)
    finally:
        print("", file=sys.stderr)
        ser.close()
        out.close()

    elapsed = time.monotonic() - started
    print(f"# {rows} rows in {elapsed:.1f}s -> {args.out}", file=sys.stderr)
    if rows and elapsed > 0:
        print(f"# {rows / elapsed:.0f} rows/s (expect ~500)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
