#!/usr/bin/env python3
"""
Offline reference implementation of the DSP v2 pipeline.

This is a deliberate mirror of the C++ in firmware/dsp_v2/dsp_v2.ino: same sample
rates, same filter coefficients, same scalar arithmetic, same update order. It exists
so the firmware can be validated against recorded captures (bio2.log, heartbeat*.log)
before it is flashed, and so regressions are caught on the desktop.

Usage:
    uv run --with numpy,scipy tools/dsp_v2_sim.py bio2.log
    uv run --with numpy,scipy tools/dsp_v2_sim.py bio2.log --compare

Input format is the raw_streamer CSV: Timestamp_ms,RawPulse,RawGSR at 500 Hz.
"""

import argparse
import cmath
import math
import sys

# ---------------------------------------------------------------------------
# Configuration (keep in sync with firmware/dsp_v2/dsp_v2.ino)
# ---------------------------------------------------------------------------
FS_RAW = 500.0
DECIM = 20  # 500 Hz -> 25 Hz. The 40 ms boxcar nulls 25/50/75 Hz (mains).
FS = FS_RAW / DECIM

# Pulse chain
HP_HZ = 0.5  # baseline tracker
LP_HZ = 4.0  # two cascaded poles, kills the 10.2 Hz tremor artifact
ENERGY_TAU = 2.0

# Resonator bank
BPM_MIN = 40.0
BPM_MAX = 190.0
N_BINS = 48
BANK_TAU = 10.0
RENORM_INTERVAL = 256  # rotator renormalisation period, in samples

# BPM output shaping.
#
# Confidence on this hardware is weak and only mildly discriminative (measured
# separation between good and bad windows is ~1.3x), so it is used as a *weight* on
# how fast the BPM may move rather than as a hard accept/reject gate. Low confidence
# therefore parks the reading on its last good value instead of letting it jump,
# which is exactly the behaviour the LEDs want.
SLEW_BPM_PER_S = 8.0
CONF_REF = 0.18  # peak/total power ratio treated as "fully trusted"
CONF_GATE = 0.06  # below this the bank is treated as noise; BPM is frozen

# GSR chain
TONIC_TAU = 45.0
PHASIC_TAU = 0.7
SCR_ATTACK_TAU = 0.15
SCR_RELEASE_TAU = 3.0
SLOPE_CLAMP = 60.0  # counts/s; rejects contact/motion steps (measured up to 570)
#
# Auto-ranging is relative to *recent* SCR activity rather than to an absolute peak
# hold. A peak-hold envelope gets pinned by a single contact artifact and then
# crushes everything after it to zero (measured: envelope stuck at 2000+ counts/s
# against a median drive of 2.6). Normalising against a slow mean instead keeps the
# bar alive and reactive, at the cost of "resting" not reading as a low absolute
# value -- the right trade for an LED display, per the responsiveness-over-accuracy
# priority.
RANGE_TAU = 30.0
RANGE_GAIN = 2.5  # drive must reach 2.5x its recent mean to peg the bar
RANGE_FLOOR = 0.5  # counts/s, keeps a dead/disconnected sensor from being amplified
OUT_ATTACK_TAU = 0.10
OUT_RELEASE_TAU = 1.50


def ema_alpha(tau_s, fs=FS):
    """EMA coefficient for a given time constant."""
    return 1.0 - math.exp(-1.0 / (tau_s * fs))


def onepole_alpha(fc_hz, fs=FS):
    """EMA coefficient for a given -3 dB corner frequency."""
    return 1.0 - math.exp(-2.0 * math.pi * fc_hz / fs)


class PulseTracker:
    """Sliding-DFT resonator bank: yields BPM, confidence and beat phase together.

    Per-beat threshold detection fails on this signal (measured SNR is below 1 for
    most of bio2.log). A bank of complex one-pole resonators integrates over many
    cycles instead, so a missed or noisy beat degrades the estimate rather than
    stalling it -- and the winning bin's argument gives a continuous beat phase for
    free, which is what actually drives the LEDs.
    """

    def __init__(self):
        self.a_hp = onepole_alpha(HP_HZ)
        self.a_lp = onepole_alpha(LP_HZ)
        self.a_energy = ema_alpha(ENERGY_TAU)
        self.a_bank = ema_alpha(BANK_TAU)

        self.baseline = None
        self.lp1 = 0.0
        self.lp2 = 0.0
        self.energy = 0.0

        # Bin centres, linear in BPM.
        self.bpms = [
            BPM_MIN + (BPM_MAX - BPM_MIN) * k / (N_BINS - 1) for k in range(N_BINS)
        ]
        # Per-bin rotation step e^(-j*w_k); advanced recursively so the sample loop
        # never calls sin/cos.
        self.step = [cmath.exp(-1j * 2.0 * math.pi * (b / 60.0) / FS) for b in self.bpms]
        self.rot = [complex(1.0, 0.0)] * N_BINS
        self.res = [complex(0.0, 0.0)] * N_BINS
        self.n = 0

        self.bpm = 78.0
        self.confidence = 0.0
        self.phase = 0.0
        self.filtered = 0.0

    def update(self, x):
        """Feed one 25 Hz sample. Returns the filtered (band-limited) value."""
        if self.baseline is None:
            self.baseline = x
            self.lp1 = 0.0
            self.lp2 = 0.0

        self.baseline += self.a_hp * (x - self.baseline)
        ac = x - self.baseline
        self.lp1 += self.a_lp * (ac - self.lp1)
        self.lp2 += self.a_lp * (self.lp1 - self.lp2)
        y = self.lp2
        self.filtered = y

        self.energy += self.a_energy * (y * y - self.energy)

        for k in range(N_BINS):
            r = self.rot[k] * self.step[k]
            self.rot[k] = r
            self.res[k] += self.a_bank * (y * r - self.res[k])

        self.n += 1
        if self.n % RENORM_INTERVAL == 0:
            # Recursive rotation drifts off the unit circle; pull it back.
            for k in range(N_BINS):
                m = abs(self.rot[k])
                if m > 0.0:
                    self.rot[k] /= m

        return y

    def update_filtered(self, y):
        """Feed a sample that has ALREADY been through the baseline/lowpass chain --
        the pulse_filtered field of a BLE Signals packet, computed on-device by this
        same PulseTracker. Used to re-run the resonator bank offline against a live
        BLE capture, which has no raw 500 Hz signal to decimate and filter (the
        device only streams the already-decimated, already-filtered 25 Hz value).
        Bypasses the baseline/lowpass stage in update() above; everything from the
        resonator bank onward is identical, so estimate() behaves the same either way.
        """
        self.filtered = y
        self.energy += self.a_energy * (y * y - self.energy)
        for k in range(N_BINS):
            r = self.rot[k] * self.step[k]
            self.rot[k] = r
            self.res[k] += self.a_bank * (y * r - self.res[k])
        self.n += 1
        if self.n % RENORM_INTERVAL == 0:
            for k in range(N_BINS):
                m = abs(self.rot[k])
                if m > 0.0:
                    self.rot[k] /= m

    def estimate(self, dt_s):
        """Called at render rate, not per sample. Updates bpm/confidence/phase."""
        best = 0
        best_p = -1.0
        total_p = 0.0
        for k in range(N_BINS):
            p = self.res[k].real ** 2 + self.res[k].imag ** 2
            total_p += p
            if p > best_p:
                best_p = p
                best = k

        # Parabolic interpolation on the power peak for sub-bin resolution.
        raw_bpm = self.bpms[best]
        if 0 < best < N_BINS - 1:
            lo = self.res[best - 1].real ** 2 + self.res[best - 1].imag ** 2
            hi = self.res[best + 1].real ** 2 + self.res[best + 1].imag ** 2
            denom = lo - 2.0 * best_p + hi
            if abs(denom) > 1e-12:
                shift = 0.5 * (lo - hi) / denom
                shift = max(-1.0, min(1.0, shift))
                raw_bpm += shift * (self.bpms[1] - self.bpms[0])

        # Peak share of total bank power. Scale-free and bounded: flat noise across
        # the bank gives 1/N_BINS, a dominant rhythm gives much more.
        self.confidence = best_p / (total_p + 1e-12)
        # Instantaneous beat phase. arg(R) alone is only the *offset* of the beat
        # relative to the demodulation reference -- for a signal sitting on the bin
        # frequency R is a near-static phasor, so it does not advance once per beat.
        # Multiplying the rotator back in (R * conj(rot)) restores the running phase.
        beat = self.res[best] * self.rot[best].conjugate()
        self.phase = math.atan2(beat.imag, beat.real)

        if self.confidence >= CONF_GATE:
            weight = min(1.0, self.confidence / CONF_REF)
            limit = SLEW_BPM_PER_S * weight * dt_s
            delta = max(-limit, min(limit, raw_bpm - self.bpm))
            self.bpm += delta
        return self.bpm, self.confidence, self.phase


class GsrTracker:
    """Two-timescale skin-conductance decomposition with auto-ranging.

    Arousal is driven by the *rate of rise* of the phasic component, not by its
    offset from a baseline, so the slow tonic drift (1412 -> 1281 counts over
    bio2.log) cannot zero the output the way the previous fixed-scale delta did.
    """

    def __init__(self):
        self.a_tonic = ema_alpha(TONIC_TAU)
        self.a_phasic = ema_alpha(PHASIC_TAU)
        self.a_attack = ema_alpha(SCR_ATTACK_TAU)
        self.a_release = ema_alpha(SCR_RELEASE_TAU)
        self.a_range = ema_alpha(RANGE_TAU)
        self.a_out_attack = ema_alpha(OUT_ATTACK_TAU)
        self.a_out_release = ema_alpha(OUT_RELEASE_TAU)

        self.tonic = None
        self.smooth = 0.0
        self.prev = 0.0
        self.drive = 0.0
        self.range = RANGE_FLOOR
        self.arousal = 0.0

    def reset(self, x):
        """Long-press recalibration: re-anchor tonic and the auto-range envelope."""
        self.tonic = x
        self.smooth = x
        self.prev = x
        self.drive = 0.0
        self.range = RANGE_FLOOR
        self.arousal = 0.0

    def update(self, x):
        if self.tonic is None:
            self.reset(x)

        self.smooth += self.a_phasic * (x - self.smooth)
        self.tonic += self.a_tonic * (self.smooth - self.tonic)
        phasic = self.smooth - self.tonic

        # Grove GSR output *falls* as skin conductance rises, so a falling value is
        # the arousal direction. Half-wave rectify: only rises count. The slope is
        # taken on the phasic component, not on the smoothed signal -- the raw signal
        # drifts downward at ~0.9 counts/s over bio2.log, which would otherwise read
        # as permanent arousal.
        slope = (self.prev - phasic) * FS  # counts/s of rising conductance
        self.prev = phasic
        if slope < 0.0:
            slope = 0.0
        elif slope > SLOPE_CLAMP:
            slope = SLOPE_CLAMP

        a = self.a_attack if slope > self.drive else self.a_release
        self.drive += a * (slope - self.drive)

        # Auto-range against recent mean activity; replaces the hardcoded /120.0.
        self.range += self.a_range * (self.drive - self.range)
        if self.range < RANGE_FLOOR:
            self.range = RANGE_FLOOR

        target = min(1.0, self.drive / (RANGE_GAIN * self.range))
        a = self.a_out_attack if target > self.arousal else self.a_out_release
        self.arousal += a * (target - self.arousal)
        return self.arousal, self.tonic, phasic


# ---------------------------------------------------------------------------
# Legacy algorithm, transcribed from main_armband.ino for side-by-side comparison
# ---------------------------------------------------------------------------
def legacy_pulse(t_ms, pulse):
    hp = 1712.0
    pmax, pmin = 1750.0, 1670.0
    last = 0.0
    rising = False
    bpm = 78.0
    beats = []
    for t, raw in zip(t_ms, pulse):
        hp = 0.998 * hp + 0.002 * raw
        ac = 2000.0 + (raw - hp) * 10.0
        pmax = max(pmax, ac) - 0.1
        pmin = min(pmin, ac) + 0.1
        thr = pmin + (pmax - pmin) * 0.70
        if ac > thr and not rising and (t - last) > 450:
            rising = True
            ibi = t - last
            last = t
            if 450 <= ibi <= 1400:
                bpm = bpm * 0.8 + (60000.0 / ibi) * 0.2
                beats.append(t)
        elif ac < thr:
            rising = False
    return beats, bpm


def legacy_gsr(gsr):
    sm = 1350.0
    base = 1350.0
    ex = 0.0
    out = []
    for v in gsr:
        sm = 0.1 * v + 0.9 * sm
        if sm > base:
            base = base * 0.999 + sm * 0.001
        else:
            base = base * 0.9999 + sm * 0.0001
        delta = max(0.0, base - sm)
        e = min(1.0, delta / 120.0)
        ex = ex * 0.7 + e * 0.3
        out.append(ex)
    return out


# ---------------------------------------------------------------------------


def load(path):
    t, p, g = [], [], []
    with open(path) as fh:
        for line in fh:
            parts = line.strip().split(",")
            if len(parts) != 3:
                continue
            try:
                ts, rp, rg = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                continue  # header
            t.append(ts)
            p.append(rp)
            g.append(rg)
    return t, p, g


def run(path, report_hz=1.0, verbose=True):
    t, p, g = load(path)
    if not t:
        sys.exit(f"{path}: no usable samples")

    pulse = PulseTracker()
    gsr = GsrTracker()

    acc_p = acc_g = 0.0
    acc_n = 0
    last_report = t[0]
    last_est = t[0]
    rows = []

    for i in range(len(t)):
        acc_p += p[i]
        acc_g += g[i]
        acc_n += 1
        if acc_n < DECIM:
            continue
        xp = acc_p / DECIM
        xg = acc_g / DECIM
        acc_p = acc_g = 0.0
        acc_n = 0

        pulse.update(xp)
        arousal, tonic, phasic = gsr.update(xg)

        # Render-rate work: the firmware does this at 60 FPS; 25 Hz here is the
        # fastest the sim can, and the results are rate-insensitive.
        dt = (t[i] - last_est) / 1000.0
        last_est = t[i]
        bpm, conf, phase = pulse.estimate(dt)

        if t[i] - last_report >= 1000.0 / report_hz:
            last_report = t[i]
            rows.append((t[i] / 1000.0, bpm, conf, phase, arousal, tonic))

    if verbose:
        print(f"# {path}  ({(t[-1] - t[0]) / 1000.0:.1f} s, {len(t)} raw samples)")
        print("time_s,bpm,confidence,phase_rad,arousal,tonic")
        for r in rows:
            print(
                f"{r[0]:.1f},{r[1]:.1f},{r[2]:.2f},{r[3]:.2f},{r[4]:.3f},{r[5]:.1f}"
            )
    return t, p, g, rows


def summarise(path, rows, t, p, g, settle_s=20.0):
    import statistics

    t0 = t[0] / 1000.0
    warm = [r for r in rows if r[0] - t0 >= settle_s]
    bpms = [r[1] for r in warm]
    confs = [r[2] for r in warm]
    arous = [r[4] for r in warm]

    # Longest stretch with confidence below the gate.
    worst_gap = 0.0
    gap_start = None
    for r in warm:
        if r[2] < CONF_GATE:
            if gap_start is None:
                gap_start = r[0]
            worst_gap = max(worst_gap, r[0] - gap_start)
        else:
            gap_start = None

    lb, lbpm = legacy_pulse(t, p)
    dur = (t[-1] - t[0]) / 1000.0
    lgsr = legacy_gsr(g)

    print(f"\n=== {path} ===")
    print(f"duration                 {dur:.1f} s")
    print("--- pulse ---")
    print(f"legacy  avg BPM          {len(lb) / dur * 60:.1f}  ({len(lb)} beats)")
    if len(lb) > 2:
        ibi = [lb[i + 1] - lb[i] for i in range(len(lb) - 1)]
        print(f"legacy  IBI std          {statistics.pstdev(ibi):.1f} ms"
              f"  (max gap {max(ibi):.0f} ms)")
    print(f"dsp_v2  BPM median       {statistics.median(bpms):.1f}")
    print(f"dsp_v2  BPM std          {statistics.pstdev(bpms):.1f}")
    print(f"dsp_v2  BPM min/max      {min(bpms):.1f} / {max(bpms):.1f}")
    print(f"dsp_v2  confidence med   {statistics.median(confs):.2f}")
    print(f"dsp_v2  longest low-conf {worst_gap:.1f} s")
    print("--- gsr ---")
    print(f"legacy  arousal mean     {statistics.mean(lgsr):.3f}"
          f"  max {max(lgsr):.3f}")
    print(f"legacy  time at zero     {100.0 * sum(1 for v in lgsr if v < 0.02) / len(lgsr):.1f} %")
    print(f"dsp_v2  arousal mean     {statistics.mean(arous):.3f}"
          f"  max {max(arous):.3f}")
    print(f"dsp_v2  time at zero     {100.0 * sum(1 for v in arous if v < 0.02) / len(arous):.1f} %")
    print(f"dsp_v2  time saturated   {100.0 * sum(1 for v in arous if v > 0.95) / len(arous):.1f} %")


def run_ble(path, report_hz=1.0):
    """Re-analyse a `tools/blectl.py stream` capture.

    That capture has no raw 500 Hz signal -- the device only streams pulse_filtered,
    the value AFTER its own baseline/lowpass chain (tools/blectl.py stream --csv
    header: timestamp_ms,pulse_filtered,gsr_phasic,pulse_raw,gsr_raw). So this feeds
    pulse_filtered straight into the resonator bank via update_filtered(), skipping
    the front-end filtering stage that already happened on-device, rather than
    re-decimating and re-filtering raw ADC counts the way `run()` above does for a
    raw_streamer capture.

    This exists mainly to look at the same harmonic-capture behaviour documented in
    DESIGN.md 2.3 from a live BLE capture instead of an offline one -- the same
    question tools/blectl.py spectrum answers on-device in real time, from a
    different angle.
    """
    pulse = PulseTracker()
    rows = []
    last_report = None
    with open(path) as fh:
        header = fh.readline().strip().split(",")
        try:
            ts_i = header.index("timestamp_ms")
            pf_i = header.index("pulse_filtered")
        except ValueError:
            sys.exit(f"{path}: not a blectl stream CSV (header: {header})")
        for line in fh:
            parts = line.strip().split(",")
            if len(parts) <= max(ts_i, pf_i):
                continue
            ts_ms = float(parts[ts_i])
            y = float(parts[pf_i])
            pulse.update_filtered(y)
            bpm, conf, phase = pulse.estimate(1.0 / FS)
            if last_report is None or ts_ms - last_report >= 1000.0 / report_hz:
                last_report = ts_ms
                rows.append((ts_ms / 1000.0, bpm, conf, phase))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", nargs="+", help="raw_streamer CSV capture(s)")
    ap.add_argument("--compare", action="store_true",
                    help="print the summary table instead of the per-second trace")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="trace output rate in Hz (default 1)")
    ap.add_argument("--from-ble", action="store_true",
                    help="input is a `blectl.py stream --csv` capture, not a raw "
                         "500 Hz raw_streamer capture -- see run_ble()")
    args = ap.parse_args()

    if args.from_ble:
        for path in args.logs:
            rows = run_ble(path, report_hz=args.rate)
            print(f"# {path} (BLE stream re-analysis, {len(rows)} rows)")
            print("time_s,bpm,confidence,phase_rad")
            for t, bpm, conf, phase in rows:
                print(f"{t:.1f},{bpm:.1f},{conf:.2f},{phase:.2f}")
        return

    for path in args.logs:
        t, p, g, rows = run(path, report_hz=args.rate, verbose=not args.compare)
        if args.compare:
            summarise(path, rows, t, p, g)


if __name__ == "__main__":
    main()
