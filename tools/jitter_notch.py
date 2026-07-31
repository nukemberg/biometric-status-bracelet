#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=1.26"]
# ///
"""How much does sampling jitter degrade the boxcar's 50 Hz notch?

The 20-sample boxcar at 500 Hz nulls 50 Hz because 20 samples span exactly two
cycles of a 50 Hz wave, so they sum to zero. That argument assumes the samples are
uniformly spaced. NimBLE shares core 0 with the sampling loop and widens the measured
period spread from +/-190 us to +/-850 us, which raises the obvious question --
answered here by measurement rather than by argument.

Timing model: the loop uses vTaskDelayUntil, which holds an absolute deadline, so
scheduling error does not accumulate. Sample i lands at i*2ms + e_i with e_i bounded
and non-negative (a task can be late, never early).

Usage: tools/jitter_notch.py
"""

from __future__ import annotations

import numpy as np

FS = 500.0
DECIM = 20
MAINS = 50.0
TRIALS = 20000
RNG = np.random.default_rng(0)


def notch_rejection_db(max_late_us: float, freq: float = MAINS) -> float:
    """Rejection of `freq` after a 20-sample boxcar, given bounded lateness.

    Returns dB below the un-averaged amplitude. Higher is better; ideal timing gives
    complete cancellation.
    """
    ideal = np.arange(DECIM) / FS
    late = RNG.uniform(0.0, max_late_us * 1e-6, size=(TRIALS, DECIM))
    t = ideal[None, :] + late

    # Random phase per trial, since mains has no fixed relationship to our clock.
    phase = RNG.uniform(0, 2 * np.pi, size=(TRIALS, 1))
    residual = np.cos(2 * np.pi * freq * t + phase).mean(axis=1)

    rms = np.sqrt(np.mean(residual**2))
    unaveraged_rms = np.sqrt(0.5)          # RMS of a unit-amplitude cosine
    if rms <= 0:
        return float("inf")
    return 20.0 * np.log10(unaveraged_rms / rms)


def main() -> None:
    cases = [
        ("ideal timing", 0.0),
        ("baseline, no BLE (+/-190us)", 190.0),
        ("BLE advertising (+/-850us)", 850.0),
        ("hypothetical +/-1500us", 1500.0),
        ("hypothetical +/-2000us (a full period)", 2000.0),
    ]

    print(f"50 Hz rejection after a {DECIM}-sample boxcar at {FS:.0f} Hz\n")
    print(f"{'timing':<42} {'rejection':>10}   {'residual':>10}")
    print("-" * 66)
    for name, late_us in cases:
        db = notch_rejection_db(late_us)
        # Express as the fraction of the original 50 Hz amplitude that survives.
        frac = 10 ** (-db / 20) if np.isfinite(db) else 0.0
        db_s = "complete" if not np.isfinite(db) else f"{db:6.1f} dB"
        print(f"{name:<42} {db_s:>10}   {frac*100:9.2f}%")

    # Put the numbers in units that matter: the measured 50 Hz amplitude on the GSR
    # line is ~14 counts RMS (DESIGN.md section 1), against a phasic signal whose
    # useful features are ~10-40 counts.
    print("\nApplied to the measured GSR mains amplitude of ~14 counts RMS:")
    for name, late_us in cases:
        db = notch_rejection_db(late_us)
        frac = 10 ** (-db / 20) if np.isfinite(db) else 0.0
        print(f"  {name:<42} {14.0*frac:6.3f} counts survive")

    print("\nSCR features of interest are ~10-40 counts, and SLOPE_CLAMP is 60 counts/s.")


if __name__ == "__main__":
    main()
