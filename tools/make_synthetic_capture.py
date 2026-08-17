#!/usr/bin/env python3
"""Generate a deterministic synthetic raw_streamer capture.

Why this exists: swapping the analog PPG sensor for the MAX30102 retired every
capture in samples/ as a regression case (DESIGN.md section 5), and the C++/Python
parity harness needs *some* input to run against. Those are two different jobs:

  - Signal quality -- does the tracker read the right BPM off a real wrist -- needs
    real captures with ground truth. Nothing here substitutes for that.
  - Implementation parity -- do libraries/BraceletDSP and tools/dsp_v2_sim.py compute
    the same numbers from the same input -- needs only an input that exercises every
    branch. Synthetic data does that job completely, and does it without hardware.

So this file is explicitly NOT a substitute for a wrist capture. It is a fixture that
keeps `just test-parity` meaningful in CI and on a machine with no bracelet attached.

Deterministic by construction: a fixed-seed LCG rather than random, so the file is
byte-identical on every machine and a parity failure is always a code change.

Usage:
    uv run tools/make_synthetic_capture.py > samples/synthetic.csv
    uv run tools/make_synthetic_capture.py --seconds 120 --bpm 72
"""

import argparse
import math

FS_RAW = 500.0
DECIM = 20          # PPG sample every 20th row, i.e. 25 Hz
IR_DC = 80000.0     # plausible MAX30102 wrist DC
IR_AC = 900.0       # ~1.1 % perfusion index
GSR_DC = 1300.0


class Lcg:
    """Numerical Recipes LCG. Deterministic across platforms and Python versions,
    which `random` with a seed is not guaranteed to be across major releases."""

    def __init__(self, seed=12345):
        self.s = seed & 0xFFFFFFFF

    def unit(self):
        """Uniform in [-1, 1)."""
        self.s = (1664525 * self.s + 1013904223) & 0xFFFFFFFF
        return self.s / 2147483648.0 - 1.0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--bpm", type=float, default=64.0,
                    help="synthetic heart rate; the tracker should recover this")
    ap.add_argument("--seed", type=int, default=12345)
    args = ap.parse_args()

    rng = Lcg(args.seed)
    n_rows = int(args.seconds * FS_RAW)
    w = 2.0 * math.pi * (args.bpm / 60.0)

    print("# synthetic capture -- parity fixture, NOT a signal-quality reference")
    print(f"# bpm={args.bpm} seconds={args.seconds} seed={args.seed}")
    print("Timestamp_ms,RawIr,RawGSR,IrNew")

    for i in range(n_rows):
        t = i / FS_RAW
        ts_ms = int(t * 1000.0)

        # GSR: slow downward tonic drift (habituation, ~0.9 counts/s as measured on
        # bio2.log), one SCR-shaped bump per 20 s, plus quantisation-scale dither and
        # a 50 Hz mains component so the 20-sample boxcar notch has something to null.
        drift = -0.9 * t
        scr = -25.0 * math.exp(-((t % 20.0) - 5.0) ** 2 / 2.0) if t > 5.0 else 0.0
        mains = 14.0 * math.sin(2.0 * math.pi * 50.0 * t)
        gsr = GSR_DC + drift + scr + mains + 1.5 * rng.unit()

        # PPG only lands on decimation boundaries. Cardiac fundamental plus a second
        # harmonic at 40 %, which is what makes harmonic capture (DESIGN.md 2.3)
        # reachable in this fixture rather than being a purely theoretical branch.
        if i % DECIM == 0:
            cardiac = IR_AC * (math.sin(w * t) + 0.40 * math.sin(2.0 * w * t))
            ir = int(IR_DC + cardiac + 60.0 * rng.unit())
            ir_new = 1
        else:
            ir = 0
            ir_new = 0

        print(f"{ts_ms},{ir},{int(gsr)},{ir_new}")


if __name__ == "__main__":
    main()
