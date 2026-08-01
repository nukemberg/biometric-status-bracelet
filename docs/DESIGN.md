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

Confidence is a power *share*, so its useful range is nothing like 0–1: flat noise sits at
1/48 = 0.02 and `CONF_REF = 0.18` is already "fully trusted". Everything that consumes it —
LED saturation, beat-phase pull, the web readout — uses `min(1, confidence/confRef)`, which
is the number worth showing a human. The raw share is kept on the wire and in logs; only
the UI converts.

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
- **confidence** — peak share of total bank power (flat noise gives 1/48 = 0.02)
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

**The floor matters more than it looks.** Slope is counts per *second* at `DSP_HZ = 25`, so
one ADC count of phasic movement between two consecutive samples is already 25 counts/s.
`RANGE_FLOOR` was 0.5, giving a floored denominator of `RANGE_GAIN × 0.5 = 1.25` — on-wrist,
raw GSR resting quietly inside a ~100-count band swung arousal across most of its range,
because 12-bit quantisation dither alone clears that denominator by an order of magnitude.
Now 3.0 (denominator 7.5). That is sized to sit above dither, **not measured** — see -9ny
and §4.1.

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

### 3.5 Wear detection and pulse trust

These are two separate questions, answered by two different signals, because the obvious
metric turns out to answer only one of them.

**Wear is decided by GSR alone.** Skin between the pads reads 1000–1509 counts across
every capture. Logged captures showed an open circuit at 3507–4095, but on-wrist testing
found the unworn bracelet resting near 2400 — inside the original 500–3000 window, which
therefore called an empty bracelet worn. The window is now 500–2100, which still clears
the worn population by a wide margin. Asymmetric dwell: 2 s to claim contact, 5 s to release. On
release the resonator bank is cleared so the next wearer never sees the previous one's
rate.

**Perfusion index cannot gate wear.** Measured with the runtime formula
(`2.83·√(EMA y²)/baseline`):

| | median | range |
|---|---|---|
| `bio4.csv` good contact, worn | 0.678 | min 0.482 |
| `bio2.log` poor contact, worn | 0.180 | p10 0.151, p90 0.306 |
| bench, nothing attached | ~0.19 | 0.17–0.20 |

Poor-but-worn and not-worn are indistinguishable. No threshold separates them: any value
that rejects an empty sensor also blanks the panel on a real wearer with mediocre contact.

An earlier `PI_WORN_MIN` of 0.15 % was set from a *different* calculation (scipy band-pass
p2p ÷ mean, giving 0.40 % / 1.60 %) and therefore sat **below** the unworn floor — the PPG
half of the gate never rejected anything, and only GSR was doing real work. The lesson is
that a threshold must be calibrated against the metric the firmware actually computes.

**Perfusion instead decides whether the rate is believable.** Above `PI_TRUST_MIN` (0.40 %,
between bio2's p90 and bio4's minimum) the pulse segment shows its colour gradient. Below
it the device still knows it is worn — GSR says so — but the pulse segment shows a
colourless sweep instead of a confident wrong number. At bio2 signal quality the tracker
was measured reporting 113 BPM against a true 88, so a "searching" state is the honest
display. GSR and temperature segments stay live throughout, and the auto-strobe will not
fire on an untrusted rate.

**Known blind spot:** perfusion measures cardiac-band amplitude, and cannot tell a strong
pulse from strong motion. A handled or shaken board reaches 1.7 % and reads as trusted
while the tracker walks steadily up through implausible rates. Motion rejection is not
solved.

### 3.6 Hardware roadmap

The current design is **v1: analog sensors through the ESP32 SAR ADC**, and everything
in §2 is calibrated against that path.

**v2 (planned, `bd -kp5`)** replaces the front end: MAX30102 for PPG, optionally ADS1115
for GSR. The motivation is §2.1 — SNR below 1 is the wall every DSP effort has hit, and
no algorithm recovers a signal beneath its noise floor. MAX30102 brings its own LED
driver, 18-bit ADC and ambient subtraction, plus a proximity mode that gives a second
independent wear signal. ADS1115's PGA at ±0.256 V gives ~7.8 µV/count against the
ESP32's ~800 µV/count, and its ΔΣ data rate doubles as its filter — at 8–16 SPS it
rejects mains better than the boxcar of §3.1, letting GSR leave the 500 Hz path entirely.

**The cost is that v2 invalidates the calibration in §2.** SNR, perfusion index,
`PI_TRUST_MIN`, `CONF_REF`/`CONF_GATE`, bank τ and the harmonic-capture measurements are
all properties of the analog path. `samples/bio2.log` and `bio4.csv` cease to be valid
regression cases and ground truth must be recaptured. It is a replatform, not a drop-in.

**An IMU (`bd -00y`) is independent of that** and can land first: it needs no front-end
change and invalidates nothing. Its role is not to measure arousal — motion-to-excitement
correlations are weak and highly person-specific. It is the *denominator* that stops the
other sensors lying: HR 110 from excitement and HR 110 from walking are indistinguishable
without it, and arm movement produces SCR-shaped GSR artifacts that look exactly like
real arousal events. It is also a principled replacement for the perfusion-based pulse
trust heuristic, which §3.5 shows failing under motion.

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

### 4.1 On-wrist GSR recalibration

GSR is the one part of the pipeline that cannot be settled offline: both thresholds that
matter — the wear window and the arousal auto-range floor — depend on the electrodes, the
skin, and how tightly the band is strapped. Two of the three parameters below are
runtime-tunable over BLE, so most of this needs no reflash.

**Step 1 — wear window (`gsrWornMin` / `gsrWornMax`).** Runtime-tunable.

```bash
tools/blectl.py monitor --seconds 30 --csv unworn.csv   # bracelet on the bench
tools/blectl.py monitor --seconds 30 --csv worn.csv     # bracelet strapped on
```

Read `gsr_raw` from each. Expect two well-separated clusters — worn is the *lower* one
(skin conductance pulls the Grove output down). Latest hardware: worn 1000–1500, unworn
~2400. Set the ceiling between them, nearer the unworn side:

```bash
tools/blectl.py config set gsr_worn_max 2100
```

The value persists in NVS. Confirm with `blectl config get`. Caveat from -6y4: an unworn
sensor is a floating input, so the unworn cluster moves between sessions and this
threshold is a re-measurement, not a fix. If the two clusters overlap, stop — that is the
IMU-gate problem (-00y), not a threshold problem.

**Step 2 — resting baseline.** Strapped on, sitting still, no talking, 3 minutes:

```bash
tools/blectl.py monitor --seconds 180 --csv rest.csv
```

`gsr_raw` should drift slowly downward (tonic habituation, ~0.9 counts/s) — that drift is
subtracted by design and must **not** move arousal. What to expect in `arousal`: with the
auto-range being relative, resting settles **mid-scale, around 0.3–0.6, and wanders**. It
does not sit at zero, and that is intentional (§3.3). What is *wrong* is arousal parked at
or near 1.0 while `gsr_raw` moves by only a few counts — that is the dither-saturation
failure the floor exists to prevent.

**Step 3 — provoke a real SCR.** The standard bench provocation is a sharp inhale. Sit
still for 30 s, then take one deep breath, then hold still for another 60 s. Repeat 3–4
times a minute apart. A startle (a clap out of your own line of sight) or mental
arithmetic under time pressure works too.

What a genuine skin-conductance response looks like:

| | |
|---|---|
| Latency | **1–3 s** after the stimulus — not instant. An immediate jump is motion or a contact artifact, not an SCR. |
| Rise | `gsr_raw` falls (conductance up) over **1–3 s**, typically tens of counts |
| Arousal | rises to near full scale within ~2–4 s (0.10 s output attack, then the drive envelope) |
| Recovery | decays back over **5–15 s** (3 s release on drive, 1.5 s on output, plus tonic re-settling) |
| Refractory | back-to-back breaths give a smaller second response — space them ~60 s apart |

Then move your arm deliberately without breathing hard. If that produces a bigger
excursion than the breath did, the response you are looking at is mechanical, not
electrodermal — check strap tension and electrode contact before tuning anything.

**Step 4 — pick the floor.** `RANGE_FLOOR` is **not** runtime-tunable today (-9ny covers
exposing it), so this step needs a reflash per value. Replay the captures through
`tools/dsp_v2_sim.py` instead, which is the whole point of the offline harness: sweep the
constant against `rest.csv` and the breath capture until resting sits low-to-mid and a
real SCR still reaches full scale. Then change it in **both** `dsp.h` and `dsp_v2_sim.py`
and re-run `tools/dsp_v2_parity.sh` — the constants are duplicated, and parity is the only
thing that catches them drifting apart.

---

## 5. Sample data

| File | What it is | Use |
|---|---|---|
| `samples/bio4.csv` | 226 s raw 500 Hz, good contact, **ground truth 64 BPM** (manual count + Welch 64.1) | Primary regression case |
| `samples/bio2.log` | 150 s raw 500 Hz, poor contact, SNR < 1, ~88 BPM | Worst-case / degradation test |
| `samples/bio3.csv` | 204 s of on-device `dsp_v2` output (BPM, confidence, phase, arousal, tonic) | On-device behaviour reference |

Format for the raw captures is `Timestamp_ms,RawPulse,RawGSR`.

---

## 6. Connectivity

BLE initialised at boot and advertising continuously. Gating it behind a button gesture was
considered and rejected: on datasheet estimates (not measured) the radio draws ~5–15 mA
against ~45 mA for the MCU and 50–150 mA for the LEDs, so under 5 % of the budget — not
worth a state machine and a third button gesture. Worth revisiting if actual current
measurement contradicts the estimate.

### Boot brownout — observed, not yet root-caused (-av5)

After flashing, the board brownout-reset 2–3 times in a loop (`E BOD: Brownout detector
was triggered`) immediately after the RMT/LED-channel init log line, before reaching
`BleService::begin()`, then recovered and completed boot. The symptom (a brownout at a
peripheral-enable that self-recovers after a few cycles) is characteristic of a marginal
USB supply, not sustained LED current — and indeed the firmware holds the LEDs off during
`setup()` and only lights them from `loop()`, so no LED current overlaps radio init. The
"full-brightness startup sweep" an earlier note speculated about is not in the code.

The boot sequence is staged defensively regardless: the RMT/LED driver and the BLE radio
— the two highest-inrush peripherals — are brought up separately with short settle delays
and log markers between them, so a marginal supply has a window to recover and a future
capture can time any recurrence to a specific peripheral. **This does not confirm root
cause.** That needs a real current measurement of the boot transient, on USB and again on
a 18650 pack, before battery operation is trusted. The power figures above remain
estimates.

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

### Firmware structure

BLE pushes `main_armband.ino` past 900 lines, so the sketch folder splits into units with
one responsibility each:

| File | Responsibility | Depends on |
|---|---|---|
| `main_armband.ino` | setup, FreeRTOS tasks, render loop | all |
| `libraries/BraceletDSP/src/dsp.h` | `PulseTracker`, `GsrTracker`, `WearDetect` | `<math.h>`, `<stdint.h>` |
| `ble_service.h` / `.cpp` | GATT setup, packet packing, command dispatch | `BiometricData` snapshot + control callback |
| `config.h` / `.cpp` | tunables struct, NVS load/save/reset | NVS |

The load-bearing constraint is that **`dsp.h` stays free of Arduino and BLE types**. That
is what allows `tools/dsp_v2_parity.sh` to keep compiling the real trackers on the host
and proving them equal to the Python reference. BLE reads a snapshot struct and never
reaches into tracker internals.

Shipped as an Arduino library (`--libraries ./libraries`) rather than a sketch-local
header, because `dsp_v2` and `main_armband` both need it and previously carried duplicate
copies that could drift apart. `WearDetect` reports a release via a `justReleased` flag
rather than clearing the resonator bank itself — that coupling would have dragged the
whole pulse engine into a class that answers one yes/no question.

### Consumers

- **Web app** — single self-contained `webapp/index.html` using Web Bluetooth: live vitals
  tiles, scrolling chart for the 25 Hz signals, bar chart for the 48-bin spectrum, and
  controls for mode / brightness / recalibrate plus config sliders with reset-to-defaults.
  No build step, no dependencies. Served over HTTPS via GitHub Pages from `/webapp`.
  Requires Chrome on Android; **iOS Safari does not implement Web Bluetooth**.
- **CLI** (`tools/blectl.py`) — Python + `bleak`. Subcommands mirror the GATT: `scan`,
  `monitor`, `stream`, `spectrum`, `config get/set`, `cmd`. Lets the development machine
  read the device untethered.

### Testing

The packet layout is the risk: a struct-packing mismatch between firmware, web app and CLI
produces silently wrong numbers rather than an error.

- `tools/ble_packet_test.cpp` compiles the **real packing header** on the host, round-trips
  known values and asserts the byte layout — the same pattern as the parity harness, which
  has already caught one real bug.
- `blectl.py --selftest` decodes a recorded packet fixture, so firmware and Python are
  checked against the same bytes.
- On device, `blectl monitor` runs alongside USB serial and the two are compared.
- The scheduling-jitter metric ships from the first BLE commit, so the core-0 risk below is
  measured rather than argued about.

### Core allocation and jitter — measured, not a risk

NimBLE's controller task pins to **core 0**, shared with the 500 Hz DSP task, which
raised the question of whether it disturbs the 40 ms boxcar window the 50 Hz notch
depends on. Measured rather than argued:

| timing | 50 Hz rejection | survives on GSR (14 counts RMS mains) |
|---|---|---|
| ideal | complete | 0.000 counts |
| baseline, no BLE (±190 µs) | 48 dB | 0.054 counts |
| BLE advertising (±850 µs) | 35 dB | 0.241 counts |
| BLE streaming signals + spectrum (±970 µs) | 34 dB | 0.275 counts |
| ±2000 µs, a full period | 28 dB | 0.562 counts |

SCR features of interest are 10–40 counts, so even full-period jitter leaves mains at
~1.4 % of the smallest thing we measure. **No core rebalancing is needed.** The notch
degrades gracefully because `vTaskDelayUntil` holds an absolute deadline, so timing
error does not accumulate and the window *length* stays exact — only its uniformity
suffers. Sample counts confirm nothing is dropped: 5003 per 10 s window with BLE running
is exactly 500 Hz. Reproduce with `tools/jitter_notch.py`.

Current split: **core 0** runs the DSP task plus the NimBLE controller and host; **core 1**
runs the Arduino loop (FastLED at 60 fps, the bank search, button, serial, BLE publish).

If load ever does need spreading, the levers are moving the NimBLE *host* task to core 1
via `CONFIG_BT_NIMBLE_PINNED_TO_CORE` — the *controller* is fixed to core 0 by the
precompiled IDF — or taking ADC sampling into a hardware-timer ISR. Neither is currently
justified.

### Security posture

BLE is unauthenticated by deliberate choice: vitals are readable and config writable by
any device in range, with no pairing. The tradeoff was raised and declined — usability
over security for this device. Reads are in any case only a sharper version of what the
21 LEDs already display to anyone nearby.
