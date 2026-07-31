# Biometric Status Bracelet — Design

A wearable festival/party biometrics panel worn on the forearm. It samples heart rate,
skin conductance and temperature, and renders them on a 21-LED WS2812B matrix.

Design priority, in order: **LED responsiveness > measurement accuracy > power**. This
is a party wearable, not a medical device, and several decisions below only make sense
in that light.

---

## 1. Hardware

### Controller

| | |
|---|---|
| **Target** | ESP32-S3 (DevKitC-1; Seeed XIAO ESP32-S3 for the final sleeve build) |
| **Development** | ESP32 WROOM-32 DevKit — used for all bench work and every capture in `samples/` |
| **Power** | 18650 pack, regulated 5 V |

Firmware selects pins with `#define MCU_ESP32_S3`. All analog sensors must sit on
**ADC1**: on both chips ADC2 is unusable while WiFi/BLE is active.

### Pin map

| Component | ESP32-S3 | WROOM-32 (dev) |
|---|---|---|
| Grove GSR v1.2 (SIG) | GPIO 1 (ADC1_CH0) | GPIO 34 |
| PPG pulse sensor (S) | GPIO 2 (ADC1_CH1) | GPIO 35 |
| BME280 SDA / SCL | GPIO 8 / 9 | GPIO 21 / 22 |
| Button (to GND, pull-up) | GPIO 5 | GPIO 18 |
| WS2812B data | GPIO 4 | GPIO 4 |

### Sensors

- **Grove GSR v1.2** — LM358-based skin conductance. Output *falls* as conductance
  rises. TP1 left disconnected. 3.3 V rail.
- **PPG pulse sensor** (XY1911-074 B506) — optical, green LED + photodiode. 3.3 V rail.
- **BME280** — I²C temperature/humidity. Address 0x76 or 0x77.
- **WS2812B ×21** — 3 segments of 7, data via 330 Ω series resistor. Most strips accept
  3.3 V logic; add a 74AHCT125 if flickering appears.

### Analog conditioning: 0.1 µF capacitors

A **0.1 µF ceramic capacitor is fitted from each analog signal pin to GND** (GPIO 34 and
GPIO 35 on the dev board). Measured effect on the GSR line, before vs after:

| | Before | After |
|---|---|---|
| GSR range | 3183–3588 | 1240–1415 |
| GSR std dev | ±180 | ±26.6 |

**Keep the capacitors — but the usual explanation for why they help is wrong.** Earlier
project notes credited them with eliminating 50 Hz mains hum. Spectral analysis of the
*post-capacitor* `samples/bio2.log` shows 50 Hz still present on the GSR line at roughly
**200× the surrounding noise floor (~14 counts RMS)**. The capacitor did not remove it.

Most of the improvement above is the DC operating point moving into the active
conductance band (3404 → 1310 mean), which is electrode contact quality, not filtering.
A 0.1 µF cap against the LM358's low output impedance has a corner far above 50 Hz, so
it cannot attenuate mains meaningfully. What actually nulls 50 Hz is the firmware's
20-sample boxcar (§3.1).

If more analog rejection is ever wanted, a series 10 kΩ + 1 µF RC (fc ≈ 16 Hz) on the
GSR pin would do real work; the bare shunt capacitor does not.

---

## 2. What we learned about the signals

This section exists because several plausible-sounding beliefs about this hardware turned
out to be false when measured. Numbers come from the captures in `samples/`.

### 2.1 The PPG signal is very weak, and contact dominates everything

| Capture | Cardiac AC (p2p) | HF noise (>10 Hz) | SNR | Perfusion index |
|---|---|---|---|---|
| `bio2.log` (poor contact) | 3–10 counts | 2.8 counts | **0.23–2.1, mostly < 1** | 0.40 % |
| `bio4.csv` (good contact) | 32 counts | 3.3 counts | **2.14** | 1.60 % |

At 12-bit / 11 dB attenuation one count ≈ 0.8 mV, so the cardiac signal is roughly
**5 mV** in the poor case. No algorithm recovers a signal buried below its noise floor —
**sensor placement is worth more than any amount of DSP.** Fingertip or inner wrist,
light but firm contact, ambient light blocked.

### 2.2 Threshold beat detection does not work here

Two successive threshold-and-refractory detectors were measured against `bio2.log`
(true rate ~88 BPM from Welch):

| Engine | Reported | IBI std | Worst gap |
|---|---|---|---|
| 450 ms fixed refractory | 113.8 BPM | 610 ms | 10.7 s |
| 285 ms dynamic refractory, 4-beat window | **187.4 BPM** (median 203, pegged at its 210 cap) | 379 ms | 8.5 s |

Shortening the refractory window made it *worse*: at SNR < 1 the extra triggers are
noise, not beats. Any scheme that decides "beat / no beat" from a single threshold
crossing inherits the full noise of one sample.

### 2.3 Spectral tracking works, with one residual failure mode

The resonator bank (§3.2) measured against `bio4.csv`, ground truth 64 BPM from a
simultaneous manual radial count and confirmed by the Welch peak at 64.1 BPM:

- median **62.2 BPM (−2.7 %)**, inside the 3.19 BPM bin spacing
- within ±8 BPM of truth **87 %** of the time
- confidence settles at 0.21–0.37 on good contact, ~0.12 on poor

The remaining 13 % is **harmonic capture**: when the fundamental momentarily dips the
bank latches onto 2× or 3× the true rate for a few seconds. On `bio4.csv` the 2nd
harmonic carries 124 power against the fundamental's 151 — close enough that a small dip
flips the argmax.

**Five fixes were tried and none worked:** denser bins (48→128, no change), spectral
whitening (per-frame std 9.4→40, much worse), power-domain smoothing before the pick
(no change), harmonic-sum scoring (promotes subharmonics too, wrecked bio2: 0.55→0.20),
and an explicit octave guard (no change to the slew-limited output). The defect is
documented rather than fixed; §3.4 mitigates its *visual* impact instead.

### 2.4 Confidence does not detect wear

An unworn board was observed reporting confidence 0.29 — as high as a good worn signal —
because the bank locks onto noise as happily as onto a pulse. Wear detection must come
from the sensors themselves (§3.5), never from tracker confidence.

### 2.5 GSR drifts, and absolute deltas are useless

Raw GSR drifts monotonically over minutes (1412 → 1281 across `bio2.log`, ~0.9 counts/s)
as normal tonic habituation. A fixed-scale delta against a slow baseline therefore reads
zero most of the time — the original engine was pinned at 0 for **33 %** of the run and
never exceeded 0.42 of full scale. Arousal must be driven by the *rate of rise* of the
phasic component, auto-ranged against recent activity.

---

## 3. Firmware architecture

Two FreeRTOS tasks, as before: sampling + DSP pinned to core 0, FastLED render on core 1
via `loop()`. Shared state in a single `BiometricData` struct behind a `portMUX` critical
section.

### 3.1 Front end — 500 Hz → 25 Hz boxcar

One `analogRead()` per channel per 2 ms tick, accumulated 20 deep and averaged to 25 Hz.

The 20-sample window is 40 ms, whose sinc nulls land **exactly on 25 / 50 / 75 Hz** — a
free, deep notch on the mains hum the capacitors do not remove. It also delivers the same
√20 noise reduction the previous 16× oversampling burst did, using **1/16 the ADC calls**.

Timing uses `vTaskDelayUntil()`. The previous `while (micros() - start < 2000) {}` spin
held core 0 at 100 % duty and prevented the idle task from ever saving power.

### 3.2 Pulse — sliding-DFT resonator bank

Per 25 Hz sample: EMA high-pass at 0.5 Hz, two cascaded EMA low-passes at 4 Hz (which
also removes a ~10.2 Hz tremor artifact with harmonics at 20.4 and 40.8 Hz that otherwise
dominates the spectrum), then 48 complex one-pole resonators spanning **40–190 BPM**,
τ = 10 s.

No gain stage. The old `*10.0` boost scaled signal and noise identically and bought
nothing.

Each resonator is a sliding DFT bin with exponential forgetting, advanced by recursive
complex rotation so the sample loop calls no trig. Renormalised every 256 samples to keep
the rotator on the unit circle. Cost ≈ 14 kflop/s — negligible.

At render rate the bank yields three things at once:
- **rate** — parabolic-interpolated argmax of |R|², slew-limited to 8 BPM/s weighted by confidence
- **confidence** — peak share of total bank power (flat noise gives 1/48)
- **beat phase** — `arg(R · conj(rot))`

That last term matters: `arg(R)` alone is only the phase *offset* relative to the
demodulation reference. For a signal sitting on the bin frequency R is a near-static
phasor and does not advance once per beat. Using it directly produced a 14.6 BPM
animation against an 80 BPM signal. Multiplying the rotator back in restores the running
phase.

### 3.3 GSR — two-timescale decomposition with auto-ranging

Tonic EMA (τ = 45 s), phasic EMA (τ = 0.7 s), phasic = smooth − tonic. Arousal is driven
by the half-wave-rectified **derivative of the phasic component** (not of the smoothed
signal — the raw drift would otherwise read as permanent arousal), clamped at 60 counts/s
to reject contact steps, with fast attack (0.15 s) / slow release (3 s).

Auto-range divides by a slow mean of recent drive (τ = 30 s, gain 2.5), replacing a
hardcoded `/120.0`. A peak-hold envelope was tried and rejected: a single contact artifact
pins it and crushes everything afterwards to zero.

**Tradeoff:** "resting" reads mid-scale rather than low, because the scale is relative to
recent activity. Correct for a lively LED bar, wrong for an absolute measure.

Measured on `bio2.log`: 0 % of samples pinned at zero (was 33 %), 13 % saturated, full
range used.

### 3.4 LED rendering

- **Phase-locked oscillator.** A free-running phase accumulator advances at the tracked
  BPM each frame and is gently pulled (12 % of error per frame) toward the resonator
  phase, free-running when confidence is below gate. The animation is therefore smooth at
  frame rate and **cannot stall**. This replaced a `beatFade` counter decayed per DSP
  sample, which went 255→0 in 42 ms — an invisible flash.
- **Continuous colour gradient.** A single descending hue ramp: 50 BPM → amber (hue 48),
  140 BPM → pure red (hue 0), 190 BPM → pink (hue 229, via uint8 wraparound). Damped with
  τ = 3 s for colour only; the beat animation still uses live BPM.

  Discrete colour bands were tried first and abandoned. A harmonic excursion from 62 to
  126 BPM crossed three band edges and strobed the panel cyan-green-yellow-red. Hysteresis
  (5 BPM margin, 4 s dwell) fixed the symptom but a gradient removes the edges entirely.
- **Confidence desaturation.** Low confidence washes the pulse segment toward white
  instead of freezing it — poor contact reads as "unsure", not as a dead panel.

### 3.5 Wear detection

Two independent gates, because each covers the other's blind spot:

1. **GSR in range.** Skin between the pads reads 1175–1509 counts across every capture;
   an open circuit rails to ~3900–4095. Window: 500–3000.
2. **PPG perfusion index** (cardiac AC / DC) above 0.15 %. Measured 1.60 % on good
   contact, 0.40 % on poor-but-worn.

Asymmetric dwell: 2 s to claim contact, 5 s to release. On release the resonator bank is
cleared so the next wearer never sees the previous one's rate.

When not worn the panel shows a slow indigo breath and no vitals — the device does not
invent a heart rate for an empty room.

**Calibration caveat:** the 0.15 % floor is derived from steady-state captures. In real
use the perfusion index is dominated by motion and reaches 5–31 %, so the floor is very
conservative. It cannot false-negative on a real wearer, but it is a weak discriminator in
the other direction. Only the GSR gate is calibrated against measured not-worn data.

---

## 4. Validation methodology

Signal-processing changes here are validated **offline before flashing**, because
debugging DSP through LED animations does not work.

| Tool | Role |
|---|---|
| `tools/dsp_v2_sim.py` | Python reference implementation of the whole pipeline, plus both legacy engines for side-by-side comparison. Run against any capture. |
| `tools/dsp_v2_parity.cpp` + `.sh` | Compiles the **actual firmware** on the host with Arduino stubbed, replays a capture through the real `PulseTracker`/`GsrTracker`, and diffs against the Python. BPM/confidence/phase agree to 0.0000. |
| `firmware/raw_streamer` | 500 Hz raw CSV capture for spectral ground truth. |
| `firmware/dsp_v2` | On-device bench sketch; streams computed values as CSV, no LEDs. |

Ground truth for heart rate comes from a **manual radial pulse count taken during the
capture**, cross-checked against a Welch spectrum. Per-window Welch alone is not a
reliable reference — on `bio2.log` it swings 58–131 BPM between windows.

---

## 5. Sample data

| File | What it is | Use |
|---|---|---|
| `samples/bio4.csv` | 226 s raw 500 Hz, good contact, **ground truth 64 BPM** (manual count + Welch 64.1) | Primary regression case |
| `samples/bio2.log` | 150 s raw 500 Hz, poor contact, SNR < 1, ~88 BPM | Worst-case / degradation test |
| `samples/bio3.csv` | 204 s of on-device `dsp_v2` output (BPM, confidence, phase, arousal, tonic) | On-device behaviour reference |

Format for the raw captures is `Timestamp_ms,RawPulse,RawGSR`.

---

## 6. Connectivity (design — not yet implemented)

BLE initialised at boot and advertising continuously. Power measurement puts the radio at
~5–15 mA against ~45 mA for the MCU and 50–150 mA for the LEDs, so gating it behind a
button gesture was considered and rejected as under 5 % of the budget — not worth the
state machine.

Stack is **NimBLE** (~250 KB flash vs ~700 KB for Bluedroid), keeping the default
partition table.

### GATT — one custom service

A custom service rather than the standard Heart Rate Service: 0x180D has no way to express
confidence, wear state, or "this reading is currently untrustworthy", and publishing a bare
BPM that generic fitness apps treat as authoritative would be misleading given §2.

All little-endian. ATT MTU negotiated to 247 so the spectrum fits one packet.

| Characteristic | Mode | Rate | Size | Contents |
|---|---|---|---|---|
| Vitals | notify | 4 Hz | 16 B | bpm×10, confidence, arousal, perfusion×100, gsr_raw, pulse_raw, temp×100, flags (worn/strobe/mode), brightness, gsr_tonic |
| Signals | notify | 5 Hz | 44 B | 5 batched 25 Hz samples: pulse_filtered, gsr_phasic, pulse_raw, gsr_raw + start timestamp |
| Spectrum | notify | 1 Hz | 54 B | 48 log-scaled resonator bin powers + peak bin + BPM range |
| Control | write | — | 2–3 B | set mode, set brightness, recalibrate GSR, set stream mask, reset bank |
| Config | read/write | — | var | `[paramId][float32]` writes; read returns all tunables packed |
| Info | read | — | var | firmware version, build date, bin count |

Signals and Spectrum are **opt-in** via the Control stream mask. A casual phone connection
costs ~64 B/s; full dev streaming ~280 B/s.

The Spectrum characteristic exists specifically to watch harmonic capture (§2.3) happen
live, which offline analysis could only infer.

**Config persists to NVS**, with an explicit reset-to-defaults command exposed in the web
app.

### Consumers

- **Web app** — single self-contained HTML file using Web Bluetooth, served over HTTPS via
  GitHub Pages. Requires Chrome on Android; iOS Safari does not implement Web Bluetooth.
- **CLI** (`tools/blectl.py`) — Python + `bleak`, for reading vitals and streams from the
  development machine without a USB tether.

### Known risk

NimBLE's controller task pins to **core 0**, shared with the 500 Hz DSP task. The boxcar's
50 Hz notch depends on an accurate 40 ms window, so BLE activity could degrade it. Rather
than restructure preemptively, the DSP loop will publish a scheduling-jitter metric
(min/max/mean actual period vs the 2 ms target). If measured jitter proves harmful, the
fix is hardware-timer sampling, which decouples the ADC from task scheduling entirely.
