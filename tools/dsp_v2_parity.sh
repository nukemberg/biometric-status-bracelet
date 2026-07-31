#!/usr/bin/env bash
# Build and run the host parity harness, then diff it against the Python reference.
# Both must agree to within float rounding, otherwise the offline validation in
# tools/dsp_v2_sim.py says nothing about what the firmware will do.
#
# Usage: tools/dsp_v2_parity.sh [capture.log]
set -euo pipefail

cd "$(dirname "$0")/.."
LOG="${1:-samples/bio2.log}"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

c++ -O2 -std=c++17 -Itools/hoststub -o "$OUT/parity" tools/dsp_v2_parity.cpp
"$OUT/parity" "$LOG" > "$OUT/cpp.csv"

uv run python tools/dsp_v2_sim.py "$LOG" --rate 1 \
  | grep -v '^#' | tail -n +2 > "$OUT/py.csv"

python3 - "$OUT/cpp.csv" "$OUT/py.csv" <<'PY'
import sys
cpp = [l.split(",") for l in open(sys.argv[1]).read().split()]
py  = [l.split(",") for l in open(sys.argv[2]).read().split()]
n = min(len(cpp), len(py))
if n == 0:
    sys.exit("no rows to compare")
cols = ["time", "bpm", "conf", "phase", "arousal", "tonic"]
t0 = float(cpp[0][0])
worst = [0.0] * len(cols)
worst_settled = [0.0] * len(cols)
for a, b in zip(cpp[:n], py[:n]):
    settled = float(a[0]) - t0 >= 45.0   # one tonic time constant
    for i in range(len(cols)):
        d = abs(float(a[i]) - float(b[i]))
        worst[i] = max(worst[i], d)
        if settled:
            worst_settled[i] = max(worst_settled[i], d)
print(f"compared {n} rows")
for name, w, ws in zip(cols, worst, worst_settled):
    print(f"  max |cpp - py| {name:8s} {w:.4f}   (after warm-up: {ws:.4f})")

# The C++ runs float32 and the sim float64. `phasic = smooth - tonic` cancels two
# ~1300-count values, so the two diverge slightly while the 45 s tonic EMA is still
# settling; after that they track to well under one LED step. Hence the two-tier
# check rather than one loose bound.
tol = {"time": 0.11, "bpm": 0.5, "conf": 0.02, "phase": 0.05,
       "arousal": 0.03, "tonic": 0.5}
tol_settled = dict(tol, arousal=0.01)
bad = [c for c, w in zip(cols, worst) if w > tol[c]]
bad += [c + "(settled)" for c, w in zip(cols, worst_settled) if w > tol_settled[c]]
if bad:
    sys.exit(f"PARITY FAIL: {', '.join(bad)} exceed tolerance")
print("PARITY OK")
PY
