#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""Score candidate breath signals against a paced capture's ground truth.

Answers the open question in bead -lc9: is respiration better recovered from the GSR
phasic path or from the PPG? Runs every candidate over the same file and reports, per
stage, what frequency it found and how confidently -- so the choice is made on measured
separation rather than on which mechanism sounds more plausible.

    tools/breath_analysis.py samples/breath_paced_01.csv
    tools/breath_analysis.py samples/breath_paced_01.csv --plot breath.png

Input is a tools/paced_capture.py recording: a raw_streamer csv carrying `# MARK` rows.
A capture with no marks is rejected -- without them there is nothing to score against.

Candidates
----------
gsr_phasic  GsrTracker's own phasic component (smooth - tonic), i.e. exactly the signal
            the arousal path already sees. If breath is here, it is the thing currently
            saturating arousal once per breath, and a notch belongs in this path.
gsr_bp      Raw GSR band-passed straight to 0.05-0.6 Hz. Upper bound on what any GSR
            method could recover: if this is weak, gsr_phasic cannot be better.
ppg_riiv    Respiratory-induced intensity variation -- the IR *baseline* wobble as
            venous return changes with intrathoracic pressure. Needs no beat detection,
            so it survives the low perfusion this hardware actually delivers.
ppg_rsa     Respiratory sinus arrhythmia -- breath-rate modulation of the beat-to-beat
            interval. The textbook route, and the one that depends most on PPG quality.

Scoring
-------
Per stage, a single-window periodogram over the 0.05-0.6 Hz band. Two numbers:

  peak    the strongest frequency in band, in breaths/min
  snr     power in a +/-0.02 Hz notch around the *expected* rate, over the median
          in-band power. Paced stages only -- it is the number that says whether the
          right answer stands out, independent of whether the peak happened to land
          on it.

The hold stage is the falsifier and is scored inverted: a candidate that still reports
a confident breath while the wearer is not breathing is measuring something else.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys

import numpy as np
from scipy import signal

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dsp_v2_sim  # noqa: E402  (sibling script; reuse the real GsrTracker)

FS = 25.0           # DSP rate after 20:1 decimation, matching DSP_HZ
DECIM = 20
TONIC_TAU = 45.0
PHASIC_TAU = 0.7

BAND_LO, BAND_HI = 0.05, 0.60     # 3 - 36 breaths/min
NOTCH_HZ = 0.02                   # half-width of the "expected rate" window

MARK_RE = re.compile(r"^# MARK (.*)$")


def ema_alpha(tau_s: float, fs: float = FS) -> float:
    return 1.0 - math.exp(-1.0 / (tau_s * fs))


def ema(x: np.ndarray, alpha: float) -> np.ndarray:
    """Same one-pole the firmware runs, applied over a whole array."""
    y = np.empty_like(x)
    acc = x[0]
    for i, v in enumerate(x):
        acc += alpha * (v - acc)
        y[i] = acc
    return y


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------

def load(path: str):
    t, ir, gsr, new, marks = [], [], [], [], []
    with open(path) as fh:
        for line in fh:
            line = line.rstrip("\n")
            m = MARK_RE.match(line)
            if m:
                kv = dict(p.split("=", 1) for p in m.group(1).split())
                marks.append(kv)
                continue
            parts = line.split(",")
            if len(parts) != 4:
                continue
            try:
                t.append(float(parts[0])); ir.append(float(parts[1]))
                gsr.append(float(parts[2])); new.append(int(parts[3]))
            except ValueError:
                continue     # header
    if not t:
        sys.exit(f"{path}: no usable rows")
    if not marks:
        sys.exit(f"{path}: no '# MARK' rows -- recapture with tools/paced_capture.py")
    return (np.array(t), np.array(ir), np.array(gsr), np.array(new), marks)


def stages_from_marks(marks: list[dict]) -> list[dict]:
    """Collapse the mark stream into [{name, rate, t0_ms, t1_ms}] in device time."""
    out: list[dict] = []
    for kv in marks:
        ev = kv["event"]
        dev = float(kv["dev_ms"])
        if ev == "stage_start":
            if out:
                out[-1]["t1"] = dev
            out.append({"name": kv["stage"], "rate": float(kv["rate"]),
                        "t0": dev, "t1": dev})
        elif ev == "end" and out:
            out[-1]["t1"] = dev
    return [s for s in out if s["t1"] > s["t0"]]


# ---------------------------------------------------------------------------
# candidate signal construction, all resampled onto one uniform 25 Hz grid
# ---------------------------------------------------------------------------

def uniform_grid(t_ms: np.ndarray) -> np.ndarray:
    return np.arange(t_ms[0], t_ms[-1], 1000.0 / FS)


def gsr_channels(t_ms: np.ndarray, gsr: np.ndarray, grid: np.ndarray):
    """Decimate GSR the way the firmware does, then build both GSR candidates."""
    n = (len(gsr) // DECIM) * DECIM
    dec = gsr[:n].reshape(-1, DECIM).mean(axis=1)
    dec_t = t_ms[:n].reshape(-1, DECIM).mean(axis=1)
    # The capture's own clock is not perfectly uniform (dropped rows, USB jitter), and
    # a periodogram assumes it is. Resample once, here, rather than letting each
    # candidate quietly disagree about the time base.
    x = np.interp(grid, dec_t, dec)

    smooth = ema(x, ema_alpha(PHASIC_TAU))
    tonic = ema(smooth, ema_alpha(TONIC_TAU))
    phasic = smooth - tonic

    sos = signal.butter(2, [BAND_LO, BAND_HI], btype="band", fs=FS, output="sos")
    bp = signal.sosfiltfilt(sos, x - x.mean())

    # The published arousal, produced by the real tracker rather than a paraphrase of
    # it. This is the signal the bead is actually about: whatever is riding the breath
    # cycle has to be visible HERE to be the thing saturating the bar.
    g = dsp_v2_sim.GsrTracker()
    arousal = np.empty_like(x)
    for i, v in enumerate(x):
        g.update(float(v))
        arousal[i] = g.arousal
    return phasic, bp, arousal


def ppg_channels(t_ms: np.ndarray, ir: np.ndarray, new: np.ndarray, grid: np.ndarray):
    """RIIV (IR baseline wobble) and RSA (beat-interval modulation)."""
    sel = new == 1
    ppg_t, ppg_x = t_ms[sel], ir[sel]
    if len(ppg_t) < 100:
        return None, None, 0.0
    x = np.interp(grid, ppg_t, ppg_x)

    sos_b = signal.butter(2, [BAND_LO, BAND_HI], btype="band", fs=FS, output="sos")
    riiv = signal.sosfiltfilt(sos_b, x - x.mean())

    # Beats for RSA. Band-pass to the cardiac range first; peak spacing is bounded
    # below by BPM_MAX (190) so a dicrotic notch cannot be counted as its own beat.
    sos_c = signal.butter(2, [0.7, 3.5], btype="band", fs=FS, output="sos")
    card = signal.sosfiltfilt(sos_c, x - x.mean())
    peaks, _ = signal.find_peaks(card, distance=int(FS * 60.0 / 190.0),
                                 prominence=np.std(card) * 0.5)
    if len(peaks) < 30:
        return riiv, None, 0.0
    beat_t = grid[peaks]
    ibi = np.diff(beat_t) / 1000.0                      # seconds
    ok = (ibi > 60.0 / 190.0) & (ibi < 60.0 / 40.0)     # drop obvious misdetections
    if ok.sum() < 20:
        return riiv, None, len(peaks) and ok.mean()
    inst_hr = 60.0 / ibi[ok]
    hr_t = beat_t[1:][ok]
    rsa = np.interp(grid, hr_t, inst_hr)
    rsa = signal.sosfiltfilt(sos_b, rsa - rsa.mean())
    return riiv, rsa, float(ok.mean())


# ---------------------------------------------------------------------------
# scoring
# ---------------------------------------------------------------------------

def score(seg: np.ndarray, expect_hz: float | None):
    """Single-window periodogram over the breath band. Returns (peak_brmin, snr)."""
    if len(seg) < int(FS * 15):
        return None, None
    seg = signal.detrend(seg)
    f, p = signal.periodogram(seg, fs=FS, window="hann")
    band = (f >= BAND_LO) & (f <= BAND_HI)
    if not band.any() or p[band].max() <= 0:
        return None, None
    fb, pb = f[band], p[band]
    peak = fb[np.argmax(pb)] * 60.0
    if expect_hz is None:
        # Unpaced stage: report how much the peak itself stands out, so the hold
        # stage still gets a number to be judged on.
        snr = pb.max() / (np.median(pb) + 1e-30)
    else:
        notch = np.abs(fb - expect_hz) <= NOTCH_HZ
        snr = (pb[notch].max() if notch.any() else 0.0) / (np.median(pb) + 1e-30)
    return peak, snr


def lockin(seg: np.ndarray, f_hz: float) -> float:
    """Amplitude at exactly f_hz as a fraction of the segment's in-band RMS.

    A pure sinusoid at f_hz scores 1.0; broadband noise of the same energy scores
    near zero. This is the test that matters here, because the peak-picking above is
    hostage to whichever artifact happened to land in the window -- a lock-in asks
    the narrower question "is the paced rate present", which is the question the
    ground truth can actually answer.
    """
    if len(seg) < int(FS * 15):
        return float("nan")
    sos = signal.butter(2, [BAND_LO, BAND_HI], btype="band", fs=FS, output="sos")
    x = signal.sosfiltfilt(sos, signal.detrend(seg))
    rms = np.sqrt(np.mean(x ** 2))
    if rms <= 0:
        return float("nan")
    t = np.arange(len(x)) / FS
    w = np.hanning(len(x))
    # Hann halves coherent gain; divide it back out so the 1.0-for-a-sinusoid
    # normalisation holds.
    amp = 2.0 * np.abs(np.sum(x * w * np.exp(-2j * np.pi * f_hz * t))) / (np.sum(w))
    return float(amp / (math.sqrt(2.0) * rms))


def fold(x: np.ndarray, grid: np.ndarray, t0_ms: float, period_s: float,
         n_bins: int = 50):
    """Average the signal over breath cycles, aligned on the inhale cues.

    Cycle folding is the test the other two cannot be: drift and one-off contact
    artifacts are uncorrelated with breath phase, so averaging over N cycles pushes
    them down by sqrt(N) while anything genuinely locked to the breath survives at
    full amplitude. Everything above was measuring the artifacts.

    Returns (mean_cycle, sem_of_mean, n_cycles).
    """
    step = period_s * 1000.0
    cycles = []
    t = t0_ms
    phase_bins = (np.arange(n_bins) + 0.5) / n_bins
    while t + step <= grid[-1]:
        want = t + phase_bins * step
        if want[0] >= grid[0]:
            cycles.append(np.interp(want, grid, x))
        t += step
    if len(cycles) < 3:
        return None, None, len(cycles)
    c = np.array(cycles)
    # Each cycle carries the slow trend it happened to sit on; that offset is not
    # breath, so remove it per cycle before averaging.
    c = c - c.mean(axis=1, keepdims=True)
    return c.mean(axis=0), c.std(axis=0, ddof=1) / math.sqrt(len(c)), len(c)


def fold_stat(mean_cycle, sem) -> float:
    """Peak-to-peak of the folded cycle in units of its own standard error.

    Reads as a z-score: below ~3 the folded shape is not distinguishable from what
    averaging unrelated noise would produce.
    """
    return float((mean_cycle.max() - mean_cycle.min()) / (sem.mean() + 1e-30))


def erp(x: np.ndarray, grid: np.ndarray, cues: list[float],
        pre_s: float = 5.0, post_s: float = 25.0):
    """Event-related average around isolated cues (sighs), baseline-corrected.

    Folding is the wrong tool for a sigh: there is no cycle, just a provocation and a
    response that rises over seconds and recovers. Each epoch is referenced to the
    quiet window before its own cue, so a slow drift passing through cannot masquerade
    as a response.

    Returns (t_seconds, mean, sem, n).
    """
    step = 1000.0 / FS
    t_rel = np.arange(-pre_s * 1000.0, post_s * 1000.0, step)
    epochs = []
    for c in cues:
        want = c + t_rel
        if want[0] < grid[0] or want[-1] > grid[-1]:
            continue
        e = np.interp(want, grid, x)
        e -= e[t_rel < 0].mean()
        epochs.append(e)
    if len(epochs) < 3:
        return t_rel / 1000.0, None, None, len(epochs)
    a = np.array(epochs)
    return (t_rel / 1000.0, a.mean(axis=0),
            a.std(axis=0, ddof=1) / math.sqrt(len(a)), len(a))


def erp_per_epoch(x: np.ndarray, grid: np.ndarray, cues: list[float],
                  pre_s: float = 5.0, win: tuple[float, float] = (1.0, 12.0)):
    """Per-epoch response amplitude, scored without assuming latency alignment.

    Averaging waveforms (erp() above) assumes every trial responds at the same delay.
    SCR latency jitters by seconds between trials, so the average smears and cancels
    exactly the peak it is meant to measure. Here each epoch is scored on its own:
    peak excursion inside the response window, in units of that epoch's own pre-cue
    standard deviation. Then a sign test over epochs asks whether the response is
    consistently positive -- which needs no latency assumption at all.

    Returns (per_epoch_z, n_positive, n).
    """
    step = 1000.0 / FS
    t_rel = np.arange(-pre_s * 1000.0, win[1] * 1000.0, step)
    zs = []
    for c in cues:
        want = c + t_rel
        if want[0] < grid[0] or want[-1] > grid[-1]:
            continue
        e = np.interp(want, grid, x)
        base = e[t_rel < 0]
        sd = base.std(ddof=1)
        if sd <= 0:
            continue
        resp = e[(t_rel >= win[0] * 1000.0)]
        zs.append(float((resp.max() - base.mean()) / sd))
    if not zs:
        return None, 0, 0
    return np.array(zs), int(sum(z > 0 for z in zs)), len(zs)


CANDIDATES = ("arousal", "gsr_phasic", "gsr_bp", "ppg_riiv", "ppg_rsa")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--plot", help="write a PNG of every candidate with stage bands")
    ap.add_argument("--zoom", metavar="STAGE",
                    help="write <plot>.zoom.png of one stage with the in/out cues "
                         "drawn on, for eyeballing what the numbers claim")
    args = ap.parse_args()

    t_ms, ir, gsr, new, marks = load(args.capture)
    stages = stages_from_marks(marks)
    grid = uniform_grid(t_ms)

    phasic, bp, arousal = gsr_channels(t_ms, gsr, grid)
    riiv, rsa, beat_ok = ppg_channels(t_ms, ir, new, grid)

    sig = {"arousal": arousal, "gsr_phasic": phasic, "gsr_bp": bp,
           "ppg_riiv": riiv, "ppg_rsa": rsa}

    dur = (t_ms[-1] - t_ms[0]) / 1000.0
    print(f"# {args.capture}: {len(t_ms)} rows, {dur:.0f}s, "
          f"{len(t_ms) / dur:.0f} rows/s, {int(new.sum())} PPG samples "
          f"({new.sum() / dur:.1f} Hz)")
    if rsa is None:
        print("# ppg_rsa unavailable: too few clean beats detected")
    else:
        print(f"# ppg_rsa: {beat_ok * 100:.0f}% of detected beat intervals plausible")
    print()

    head = f"{'stage':<10}{'expect':>8}" + "".join(
        f"{c:>22}" for c in CANDIDATES)
    print(head)
    print(f"{'':<10}{'br/min':>8}" + "".join(f"{'peak / snr':>22}" for _ in CANDIDATES))
    print("-" * len(head))

    for st in stages:
        m = (grid >= st["t0"]) & (grid <= st["t1"])
        expect_hz = (st["rate"] / 60.0
                     if st["rate"] and not st["name"].startswith("sigh") else None)
        row = f"{st['name']:<10}{(st['rate'] or 0):>8.0f}"
        for c in CANDIDATES:
            x = sig[c]
            if x is None:
                row += f"{'--':>22}"
                continue
            peak, snr = score(x[m], expect_hz)
            row += ("--" if peak is None else
                    f"{peak:8.1f} /{snr:7.1f}").rjust(22)
        print(row)

    print()
    print("snr on paced rows is power at the paced rate over median in-band power;")
    print("on unpaced rows it is the peak's own prominence. hold: lower is better.")

    # ---- lock-in contrast --------------------------------------------------
    # The verdict table. Every stage is probed at BOTH paced frequencies, so each
    # candidate is judged on whether 0.10 Hz rises only while 6/min was paced and
    # 0.20 Hz only while 12/min was -- a pattern no artifact reproduces by accident.
    probes = sorted({s["rate"] for s in stages if s["rate"]})
    if probes:
        print()
        print("lock-in amplitude at each paced rate, as a fraction of in-band RMS")
        print("(1.0 = pure sinusoid at that rate). Diagonal should dominate:")
        print()
        head = f"{'stage':<10}" + "".join(
            f"{c + ' @' + str(int(r)):>16}" for c in CANDIDATES for r in probes)
        print(head)
        print("-" * len(head))
        for st in stages:
            m = (grid >= st["t0"]) & (grid <= st["t1"])
            row = f"{st['name']:<10}"
            for c in CANDIDATES:
                for r in probes:
                    x = sig[c]
                    if x is None:
                        row += f"{'--':>16}"
                    else:
                        row += f"{lockin(x[m], r / 60.0):>16.3f}"
            print(row)

    # ---- cue-locked cycle folding -----------------------------------------
    # A sigh stage carries a rate too, but it is a provocation interval and not a
    # rhythm -- it is scored by erp() below, never folded.
    paced = [s for s in stages if s["rate"] and not s["name"].startswith("sigh")]
    folds: dict[tuple[str, str], tuple] = {}
    if paced:
        print()
        print("cycle fold, aligned on the inhale cues: peak-to-peak of the averaged")
        print("breath cycle in units of its own standard error (z). 'null' folds the")
        print("same data at 1.37x the true period -- a control that must stay small.")
        print()
        head = f"{'stage':<10}{'n':>4}" + "".join(f"{c:>20}" for c in CANDIDATES)
        print(head)
        print(f"{'':<10}{'':>4}" + "".join(f"{'z / null':>20}" for _ in CANDIDATES))
        print("-" * len(head))
        for st in paced:
            period = 60.0 / st["rate"]
            cue0 = next((float(k["dev_ms"]) for k in marks
                         if k["stage"] == st["name"] and k["event"] == "in"), st["t0"])
            end = st["t1"]
            row_n = 0
            row = ""
            for c in CANDIDATES:
                x = sig[c]
                if x is None:
                    row += f"{'--':>20}"
                    continue
                m = (grid >= cue0) & (grid <= end)
                gsub, xsub = grid[m], x[m]
                mc, sem, n = fold(xsub, gsub, cue0, period)
                nc, nsem, _ = fold(xsub, gsub, cue0, period * 1.37)
                row_n = max(row_n, n)
                if mc is None:
                    row += f"{'--':>20}"
                else:
                    folds[(st["name"], c)] = (mc, sem, n, period)
                    z = fold_stat(mc, sem)
                    zn = fold_stat(nc, nsem) if nc is not None else float("nan")
                    row += f"{z:9.1f} /{zn:8.1f}"
            print(f"{st['name']:<10}{row_n:>4}" + row)

    # ---- sigh response -----------------------------------------------------
    sigh_stages = [s for s in stages if s["name"].startswith("sigh")]
    erps: dict[tuple[str, str], tuple] = {}
    if sigh_stages:
        print()
        print("sigh response: average of the epochs around each deep-breath cue,")
        print("each referenced to its own 5 s pre-cue baseline. peak is the largest")
        print("excursion within 25 s, z is that peak over its standard error.")
        print()
        head = f"{'stage':<10}{'n':>4}" + "".join(f"{c:>22}" for c in CANDIDATES)
        print(head)
        print(f"{'':<10}{'':>4}" + "".join(f"{'peak @ s / z':>22}" for _ in CANDIDATES))
        print("-" * len(head))
        for st in sigh_stages:
            cues = [float(k["dev_ms"]) for k in marks
                    if k["stage"] == st["name"] and k["event"] == "sigh"]
            row, row_n = "", 0
            for c in CANDIDATES:
                x = sig[c]
                if x is None:
                    row += f"{'--':>22}"
                    continue
                t_rel, mean, sem, n = erp(x, grid, cues)
                row_n = max(row_n, n)
                if mean is None:
                    row += f"{'--':>22}"
                    continue
                erps[(st["name"], c)] = (t_rel, mean, sem, n)
                i = int(np.argmax(np.abs(mean)))
                z = abs(mean[i]) / (sem[i] + 1e-30)
                row += f"{mean[i]:8.3f} @{t_rel[i]:5.1f} /{z:5.1f}"
            print(f"{st['name']:<10}{row_n:>4}" + row)

        print()
        print("per-epoch scoring, which makes no latency assumption: each epoch's")
        print("peak in +1..+12 s over its own pre-cue SD, then median across epochs")
        print("and how many of them were positive.")
        print()
        head = f"{'stage':<10}{'n':>4}" + "".join(f"{c:>22}" for c in CANDIDATES)
        print(head)
        print(f"{'':<10}{'':>4}" + "".join(f"{'median z / n pos':>22}"
                                           for _ in CANDIDATES))
        print("-" * len(head))
        for st in sigh_stages:
            cues = [float(k["dev_ms"]) for k in marks
                    if k["stage"] == st["name"] and k["event"] == "sigh"]
            row, row_n = "", 0
            for c in CANDIDATES:
                x = sig[c]
                if x is None:
                    row += f"{'--':>22}"
                    continue
                zs, npos, n = erp_per_epoch(x, grid, cues)
                row_n = max(row_n, n)
                row += (f"{'--':>22}" if zs is None
                        else f"{np.median(zs):14.1f} /{npos:3d}/{n:<3d}")
            print(f"{st['name']:<10}{row_n:>4}" + row)

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        avail = [c for c in CANDIDATES if sig[c] is not None]
        fig, axes = plt.subplots(len(avail), 1, figsize=(14, 2.4 * len(avail)),
                                 sharex=True)
        ts = (grid - grid[0]) / 1000.0
        for ax, c in zip(np.atleast_1d(axes), avail):
            ax.plot(ts, sig[c], lw=0.6)
            ax.set_ylabel(c, fontsize=9)
            for st in stages:
                x0 = (st["t0"] - grid[0]) / 1000.0
                x1 = (st["t1"] - grid[0]) / 1000.0
                ax.axvspan(x0, x1, alpha=0.08,
                           color="tab:red" if st["rate"] else "tab:gray")
                ax.text(x0 + 1, ax.get_ylim()[1], st["name"], fontsize=7, va="top")
        np.atleast_1d(axes)[-1].set_xlabel("seconds")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"\nwrote {args.plot}")

        if folds:
            names = sorted({k[0] for k in folds})
            fig, axes = plt.subplots(len(names), len(avail),
                                     figsize=(3.0 * len(avail), 2.6 * len(names)),
                                     squeeze=False)
            for r, sname in enumerate(names):
                for cix, c in enumerate(avail):
                    ax = axes[r][cix]
                    got = folds.get((sname, c))
                    if not got:
                        ax.axis("off")
                        continue
                    mc, sem, n, period = got
                    ph = (np.arange(len(mc)) + 0.5) / len(mc) * period
                    ax.plot(ph, mc, lw=1.2)
                    ax.fill_between(ph, mc - sem, mc + sem, alpha=0.25)
                    ax.axhline(0, color="k", lw=0.5)
                    ax.axvline(period / 2, color="tab:red", lw=0.7, ls="--")
                    if r == 0:
                        ax.set_title(c, fontsize=9)
                    if cix == 0:
                        ax.set_ylabel(f"{sname}\n(n={n})", fontsize=8)
                    ax.tick_params(labelsize=7)
            fig.suptitle("mean breath cycle +/- SEM; inhale cue at 0, "
                         "exhale cue dashed", fontsize=9)
            fig.tight_layout()
            fold_png = args.plot.replace(".png", "") + ".fold.png"
            fig.savefig(fold_png, dpi=110)
            print(f"wrote {fold_png}")

        if erps:
            names = sorted({k[0] for k in erps})
            fig, axes = plt.subplots(len(names), len(avail),
                                     figsize=(3.0 * len(avail), 2.6 * len(names)),
                                     squeeze=False)
            for r, sname in enumerate(names):
                for cix, c in enumerate(avail):
                    ax = axes[r][cix]
                    got = erps.get((sname, c))
                    if not got:
                        ax.axis("off")
                        continue
                    t_rel, mean, sem, n = got
                    ax.plot(t_rel, mean, lw=1.2)
                    ax.fill_between(t_rel, mean - sem, mean + sem, alpha=0.25)
                    ax.axhline(0, color="k", lw=0.5)
                    ax.axvline(0, color="tab:red", lw=0.8)
                    if r == 0:
                        ax.set_title(c, fontsize=9)
                    if cix == 0:
                        ax.set_ylabel(f"{sname}\n(n={n})", fontsize=8)
                    ax.tick_params(labelsize=7)
            fig.suptitle("mean response to a cued deep breath +/- SEM "
                         "(cue at t=0)", fontsize=9)
            fig.tight_layout()
            erp_png = args.plot.replace(".png", "") + ".sigh.png"
            fig.savefig(erp_png, dpi=110)
            print(f"wrote {erp_png}")

        if args.zoom:
            hit = [s for s in stages if s["name"] == args.zoom]
            if not hit:
                sys.exit(f"--zoom {args.zoom}: no such stage in this capture")
            st = hit[0]
            m = (grid >= st["t0"]) & (grid <= st["t1"])
            zt = (grid[m] - st["t0"]) / 1000.0
            cues = [(float(k["dev_ms"]) - st["t0"]) / 1000.0 for k in marks
                    if k["event"] in ("in", "out") and k["stage"] == st["name"]]
            fig, axes = plt.subplots(len(avail), 1, figsize=(14, 2.4 * len(avail)),
                                     sharex=True)
            for ax, c in zip(np.atleast_1d(axes), avail):
                ax.plot(zt, sig[c][m], lw=0.9)
                ax.set_ylabel(c, fontsize=9)
                for i, cue in enumerate(cues):
                    ax.axvline(cue, color="tab:red", lw=0.7, alpha=0.6,
                               ls="-" if i % 2 == 0 else "--")
            np.atleast_1d(axes)[-1].set_xlabel(
                f"seconds into {st['name']} (solid = inhale cue, dashed = exhale)")
            fig.tight_layout()
            out_png = args.plot.replace(".png", "") + ".zoom.png"
            fig.savefig(out_png, dpi=110)
            print(f"wrote {out_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
