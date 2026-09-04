#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy"]
# ///
"""Regression test: the arousal output must report a real, cued SCR.

samples/breath_sigh_01.csv is seven deep breaths cued 60 s apart, each of which
provokes a skin-conductance response. The GSR front end sees all seven -- the
phasic component responds 7/7 at median per-epoch z=6.9, level with the ppg_riiv
positive control -- so any of them missing from the published arousal is a defect
in the normalizer, not an absent signal.

Bead -0b6: with FLOOR_MIN=3.0 / RANGE_GAIN=2.5 the tracker reported 4 of the 7
as exactly 0.000, because FLOOR_MIN set the dead-zone threshold above the drive a
real SCR produces (1.45-8.01 counts/s here).

Three assertions, in the order they matter:

  responds   every cue produces a reading above AROUSAL_FLOOR. This is the one
             that was failing.
  graded     no cue pins the bar at 1.0. A tracker that answers "everything is
             maximal" carries no more information than one that answers nothing.
  quiet      the 90th percentile away from any cue stays low, so sensitivity was
             not bought by turning the whole trace into noise.

Run: tools/gsr_scr_test.py   (or `just test-gsr`)
"""

from __future__ import annotations

import re
import sys

import numpy as np

sys.path.insert(0, "tools")
import dsp_v2_sim as D  # noqa: E402

CAPTURE = "samples/breath_sigh_01.csv"

# A second fixture, for the opposite failure. breath_paced_01.csv was recorded during
# paced breathing but contains real contact/motion disturbances (the largest at
# t~17 s), and the wearer was at rest throughout -- so time spent pinned at the top of
# the scale there is artifact, not arousal. -0b6's fix widened the output range, and
# this budget is what stops a future sensitivity tweak from buying responsiveness by
# letting artifacts saturate the bar again.
ARTIFACT_CAPTURE = "samples/breath_paced_01.csv"
ARTIFACT_PIN_MAX_PCT = 3.5     # measured 4.28 % at SLOPE_CLAMP=60, 3.10 % at 15

DECIM = 20

# Response window after a cue. SCR latency is 1-3 s and recovery runs 10-20 s, so
# this brackets the rise without reaching into the next cue 60 s later.
RESP_LO_MS, RESP_HI_MS = 1000.0, 12000.0
WARMUP_MS = 60000.0          # one tonic time constant plus margin

AROUSAL_FLOOR = 0.02         # below this the bar is visually indistinguishable from 0
PIN_CEILING = 0.98
QUIET_P90_MAX = 0.15


def load(path: str):
    t, g, marks = [], [], []
    for line in open(path):
        m = re.match(r"^# MARK (.*)$", line.strip())
        if m:
            marks.append(dict(kv.split("=", 1) for kv in m.group(1).split()))
            continue
        p = line.strip().split(",")
        if len(p) != 4:
            continue
        try:
            t.append(float(p[0]))
            g.append(float(p[2]))
        except ValueError:
            continue
    t, g = np.array(t), np.array(g)
    n = (len(g) // DECIM) * DECIM
    return (t[:n].reshape(-1, DECIM).mean(axis=1),
            g[:n].reshape(-1, DECIM).mean(axis=1), marks)


def main() -> int:
    dt, dec, marks = load(CAPTURE)
    cues = [float(k["dev_ms"]) for k in marks if k["event"] == "sigh"]
    if not cues:
        sys.exit(f"{CAPTURE}: no sigh cues; wrong fixture")

    gt = D.GsrTracker()
    ar = np.empty_like(dec)
    for i, v in enumerate(dec):
        gt.update(float(v))
        ar[i] = gt.arousal

    resp = np.zeros(len(dt), bool)
    peaks = []
    for c in cues:
        w = (dt >= c + RESP_LO_MS) & (dt <= c + RESP_HI_MS)
        resp |= w
        peaks.append(ar[w].max())
    quiet = ar[(dt > dt[0] + WARMUP_MS) & ~resp]
    q90 = float(np.percentile(quiet, 90))

    print(f"FLOOR_MIN={D.FLOOR_MIN} RANGE_GAIN={D.RANGE_GAIN}")
    print(f"{len(cues)} cues: " + " ".join(f"{p:.2f}" for p in peaks))
    print(f"quiet p90 = {q90:.3f}")

    fails = []
    dead = [i for i, p in enumerate(peaks) if p <= AROUSAL_FLOOR]
    if dead:
        fails.append(f"responds: cues {dead} produced arousal <= {AROUSAL_FLOOR} "
                     f"({len(dead)}/{len(peaks)} real SCRs unreported)")
    pinned = [i for i, p in enumerate(peaks) if p >= PIN_CEILING]
    if pinned:
        fails.append(f"graded: cues {pinned} pinned at >= {PIN_CEILING}")
    if q90 > QUIET_P90_MAX:
        fails.append(f"quiet: p90 {q90:.3f} exceeds {QUIET_P90_MAX}")

    # Artifact budget, on the other fixture.
    adt, adec, _ = load(ARTIFACT_CAPTURE)
    agt = D.GsrTracker()
    aar = np.empty_like(adec)
    for i, v in enumerate(adec):
        agt.update(float(v))
        aar[i] = agt.arousal
    warm = aar[adt > adt[0] + WARMUP_MS]
    pinned_pct = 100.0 * float((warm > 0.9).mean())
    print(f"SLOPE_CLAMP={D.SLOPE_CLAMP}  "
          f"{ARTIFACT_CAPTURE.split('/')[-1]} time above 0.9 = {pinned_pct:.2f} %")
    if pinned_pct > ARTIFACT_PIN_MAX_PCT:
        fails.append(f"artifacts: {pinned_pct:.2f} % of a resting session pinned "
                     f"above 0.9, budget is {ARTIFACT_PIN_MAX_PCT} %")

    if fails:
        for f in fails:
            print(f"FAIL {f}")
        return 1
    print("GSR SCR OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
