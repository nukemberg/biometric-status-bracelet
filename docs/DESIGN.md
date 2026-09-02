# Biometric Status Bracelet — Design

A wearable festival/party biometrics panel worn on the forearm, mounted in a fabric
sleeve with a velcro closure. It samples heart rate, skin conductance and temperature,
and renders them on a 21-LED WS2812B matrix.

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

Firmware selects pins with `#define MCU_ESP32_S3`. GSR is now the only analog sensor
and must sit on **ADC1**: on both chips ADC2 is unusable while WiFi/BLE is active.

### Pin map

| Component | ESP32-S3 | WROOM-32 (dev) |
|---|---|---|
| Grove GSR v1.2 (SIG, via RC) | GPIO 1 (ADC1_CH0) | GPIO 34 |
| I²C SDA (MAX30102 + BME280) | GPIO 8 | GPIO 21 |
| I²C SCL (MAX30102 + BME280) | GPIO 9 | GPIO 22 |
| MAX30102 INT | GPIO 10 (wired, INT_ENABLE_1 PPG_RDY on; polled, not ISR-driven) | GPIO 19 |
| Button (to GND, pull-up) | GPIO 5 -- **not populated on this build**, see below | GPIO 18 |
| WS2812B data | GPIO 6 | GPIO 4 |

GSR reaches the pin through a series 10 kΩ / shunt 1 µF RC — see §Analog conditioning.

**WS2812B moved off GPIO4 on the S3 build**: that pin measured ~860 Ω to GND
unpowered during bring-up (damaged), so the strip data line is on GPIO6 instead.
`PIN_BUTTON` (GPIO5) is still wired in firmware (`processButton()`: short press
cycles display mode, long press ≥1 s recalibrates GSR) but no external button is
actually soldered on this build -- only the bare devkit's own BOOT/RESET are
physically present, and the firmware does not read those.

### Sensors

- **Grove GSR v1.2** — LM358-based skin conductance. Output *falls* as conductance
  rises. TP1 left disconnected. 3.3 V rail.
- **MAX30102** — I²C reflective PPG, address **0x57**. Red + IR LEDs, 18-bit ADC,
  on-chip ambient subtraction. Replaced the analog XY1911-074 B506; see §3.6.
  Board fitted is the **MH-ET LIVE** breakout: two 4-pin headers, `GND RD IRD INT` on
  one side and `VIN SDA SCL GND` on the other. `RD`/`IRD` are the RED and IR LED drive
  pins broken out for external LEDs — the onboard LEDs are fitted, so leave both
  floating. The die is on the face *opposite* the silkscreen; headers solder to the
  labelled side and the optical window faces skin.
- **BME280** — I²C temperature/humidity. Address 0x76 or 0x77.
- **WS2812B ×21** — 3 segments of 7, data via 330 Ω series resistor. Most strips accept
  3.3 V logic; add a 74AHCT125 if flickering appears. Glued to the case in a **spiral**,
  not a straight run: the data chain order is segment A → segment C (physically the
  middle turn) → segment B, and segment B's physical LED order runs opposite the data
  chain's. Firmware (`main_armband.ino`) captures this as `SEG_A`/`SEG_B`/`SEG_C`
  (base index + direction) and a `segLed()` helper rather than raw index math — any
  new segment-positional render must go through that, not a literal `7+i`.

### MAX30102 wiring notes

Shares SDA/SCL with the BME280 — 0x57 against 0x76/0x77, no address collision. Bus runs
at 400 kHz (`Wire.setClock`), which the 25 Hz FIFO drain wants: at 100 kHz a 6-byte
burst plus its pointer read is a meaningful slice of the 40 ms decimation budget on the
same task that has to hit a 2 ms tick.

Three things that bite:

1. **Pull-up rail on cheap breakouts.** Many purple GY-MAX30102 boards tie the SDA/SCL
   pull-ups to the module's internal **1.8 V** rail. The bus then idles at 1.8 V, below
   the ESP32's V_IH of 0.75 × 3.3 = 2.48 V, so *every* device on the bus including the
   BME280 stops responding. Cut those pull-ups and fit 4.7 kΩ to 3.3 V externally.
   SparkFun and Adafruit boards do not have this problem.

   The MH-ET LIVE board fitted here exposes this as a **solder jumper on the top edge,
   pads marked `1V8` and `3V3`** — it ships strapped either way, so verify rather than
   assume. Same rail feeds the INT pull-up, not just SDA/SCL. Check by measuring SDA to
   GND with the bus idle: want ~3.3 V, and ~1.8 V means move the jumper. An I²C scan
   returning 0x57 confirms it end-to-end.
2. **Decoupling.** The IR LED pulses at ~50 mA. 10 µF + 0.1 µF at the module, or the
   transient shows up on the GSR line — which is on the same 3.3 V rail and whose
   features of interest are 10–40 counts.
3. **LED drive current** (`MAX30102_LED_CURRENT`, default 0x32 ≈ 10 mA) is **not tuned
   on skin**. Too low and the DC sits in the noise; too high and the ADC clips the
   cardiac AC away at the top of its range. Both present downstream as "no pulse" and
   neither announces itself. See §4.2.

**INT is now wired and read, but only as a diagnostic.** `PIN_PPG_INT` (GPIO10) has a
`pinMode(INPUT)` + `attachInterrupt(FALLING)`; the ISR just counts PPG_RDY edges
(`ppgIntCount`). The DSP task still *drains the FIFO* by polling the pointers at its
25 Hz decimation boundary rather than on the interrupt — that read is I²C and cannot
happen inside an ISR — so the edge count exists purely as a second, independent check
on the MAX30102's oscillator: `[PPG] fifo` (from FIFO pointer math) and `[PPG] int`
(from counted edges) should track each other; a persistent mismatch would point at the
ISR/wiring rather than the oscillator. Confirmed on-device: both settle around 25 Hz.

Real bug found and fixed getting here: `Max30102::read()` read `FIFO_DATA` every
drain but never `INT_STATUS_1` (register 0x00) — and on this part, **PPG_RDY clears
only by reading INT_STATUS_1**, not by reading FIFO_DATA (that clears A_FULL). Without
it the INT pin latches low on the very first sample and never releases. Confirmed with
a raw GPIO10 polling test: 0 transitions over 10 s while the FIFO was draining fine.
Fixed in `libraries/BraceletMAX30102/src/max30102.h` — `read()` now reads
INT_STATUS_1 at the top of every call. The line is open-drain and needs a pull-up; the
MH-ET board provides one, on whichever rail its `1V8`/`3V3` jumper selects — and the
board's **second GND pin** (the `RD IRD INT` header, separate from the `VIN SDA SCL
GND` header) needs wiring too, not just the SDA/SCL-side GND, or INT reads flat instead
of toggling despite continuity checks passing.

### OVF_COUNTER is not a loss counter on this part

Measured during bring-up, and worth knowing before anyone "fixes" the driver back.

The datasheet presents `OVF_COUNTER` (0x05) as a latched count of samples lost to a full
FIFO. On the part fitted here (`PART_ID` 0x15, `REV_ID` 0x03) it is not. It reads **5
during completely healthy operation** — `WR` advancing 5, `RD` following 5, `avail`
steady at 5, IR data clean and continuous at 24.9 Hz — and it reads 5 again on the next
poll after being explicitly written to 0. It tracks samples *produced* since the last
clear, not samples dropped.

The first version of the driver treated any non-zero `OVF` as "continuity lost, resync"
and so **discarded every sample the sensor ever produced**, while reporting a rising
overflow count that made it look like a core-0 scheduling problem. The jitter monitor
said otherwise (mean 2000.0 µs, 0 overruns), which is what made the sensor path the
suspect. `firmware/max30102_probe` is the diagnostic that settled it — it reads the
config registers back and dumps the raw pointer bytes, and it also proved the burst read
of 0x04–0x06 with a repeated start agrees byte-for-byte with three individual reads, so
the driver's I²C path was never at fault.

Consequence: `avail = (WR - RD) & 0x1F` is the only trustworthy signal, and it carries
the classic wrap ambiguity — a completely full 32-deep FIFO is indistinguishable from an
empty one. Losing samples needs a 32/25 = **1.3 s** stall, and that shows up as a dip in
the measured FIFO rate. Coarser than a working overflow counter, and honest.

**Measured FIFO rate: 25.09 Hz** against the 25.00 the DSP assumes — a uniform 0.36 %
low bias on every reported BPM. Below the 3.19 BPM bin spacing at any plausible rate, so
no correction is applied; recorded here so it is a known quantity rather than a
discovery.

### Analog conditioning: shunt capacitor and the GSR RC

A **0.1 µF ceramic capacitor is fitted from the GSR signal pin to GND** (GPIO 34 on the
dev board, GPIO 1 on the S3). A second one sat on GPIO 35 when that pin carried the
analog PPG; the MAX30102 swap (§3.6) retired that channel, and GSR is now the only
analog input. Measured effect on the GSR line, before vs after:

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

**The RC that does do real work.** A series **10 kΩ** from the Grove SIG output into a
shunt **1 µF** at the GPIO gives fc ≈ 16 Hz — about −10 dB at 50 Hz, which is modest next
to what the boxcar already achieves. Its actual value is **anti-aliasing**: nothing else
band-limits the GSR line ahead of a 500 Hz sampler, so everything above 250 Hz currently
folds into the passband. Step response τ = 10 ms against sub-Hz phasic EDA content, so
there is no signal cost.

Three constraints, none optional:

- **Order is series R first, then C at the pin** — never the reverse. The ESP32 ADC
  sample-and-hold dumps charge onto a small internal cap at the sampling instant; a
  10 kΩ series resistor on its own causes a settling error. The 1 µF sitting directly at
  the GPIO is a large reservoir by comparison and supplies that charge. Keep it
  physically close to the pin.
- **Ceramic X7R or film, not electrolytic.** A few µA of electrolytic leakage across
  10 kΩ becomes tens of mV of drifting offset — precisely the drift the GSR path is
  already fighting (§2.5). Avoid Y5V.
- The existing 0.1 µF is on the same node and simply parallels in (1.1 µF, fc ≈ 14.5 Hz).
  Leave it or remove it; do not add a second.

> **Not yet built or measured.** The before/after table above is the 0.1 µF cap alone.
> Re-capture after fitting the RC and add a third column; if the std dev moves, the wear
> window and arousal auto-range calibration want revisiting.

**Why not a better ADC.** The obvious upgrades do not help this signal. GSR noise is
±26.6 counts against a 12-bit LSB of ~0.8 mV, so the converter is roughly 30× quieter
than the line it measures — an ADS1115 would resolve 0.06 mV and buy nothing but I²C
traffic on the bus and core that already carry the MAX30102 FIFO and the DSP tick. An
8-bit ADC0832 is strictly worse: VCC-referenced at 5 V gives a 19.5 mV LSB against a
~140 mV working swing, collapsing 175 counts of range to about 7. A genuinely
differential front end *would* reject body-coupled mains at the electrodes rather than
after the fact, but the Grove board collapses to single-ended at its LM358, so that means
replacing it — excitation divider, dual rail-to-rail buffer, sense resistor, the lot —
and recalibrating everything downstream. The limit here is electrode contact, not bits.

---

## 2. What we learned about the signals

This section exists because several plausible-sounding beliefs about this hardware turned
out to be false when measured. Numbers come from the captures in `samples/`.

> **Every PPG number below was measured on the v1 analog front end and is now history,
> not calibration.** The MAX30102 (§3.6) replaced that sensor, so the SNR figures, the
> perfusion-index table, `PI_TRUST_MIN` and the harmonic-capture measurements all
> describe hardware the bracelet no longer has. They are kept because they are the
> reasons the architecture is shaped the way it is — §2.2 is why there is a resonator
> bank at all, §2.4 is why confidence never gates wear — and those conclusions survive
> the sensor change even though the numbers do not. **Nothing here has been re-measured
> on the MAX30102 yet.** See §4.2 for what to capture.
>
> §2.5 (GSR) is unaffected: same sensor, same ADC, same rate.

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

### 3.1 Front end — two channels, two clocks

The two sensors no longer share a sample clock.

**GSR — 500 Hz → 25 Hz boxcar.** One `analogRead()` per 2 ms tick, accumulated 20 deep
and averaged to 25 Hz. The 20-sample window is 40 ms, whose sinc nulls land **exactly on
25 / 50 / 75 Hz** — a free, deep notch on the mains hum the capacitors do not remove. It
also delivers the same √20 noise reduction the previous 16× oversampling burst did, using
**1/16 the ADC calls**.

Timing uses `vTaskDelayUntil()`. The previous `while (micros() - start < 2000) {}` spin
held core 0 at 100 % duty and prevented the idle task from ever saving power.

**PPG — MAX30102 FIFO at 25 Hz.** 100 Hz internal sampling with 4× on-chip averaging
lands the FIFO at exactly `DSP_HZ`, so `PulseTracker` is fed at the rate every
coefficient in `dsp.h` already assumes and nothing downstream changed. 411 µs pulse width
(18-bit) with both LEDs active caps the internal rate at 100 Hz per the datasheet's
pulse-width table, which is why it is 100/4 rather than 400/16 — resolution is worth more
here than headroom, given that §2.1 says amplitude was the entire v1 problem.

The DSP task drains the FIFO at its own decimation boundary and calls
`PulseTracker.update()` once per sample retrieved — normally one, occasionally zero or
two as the two clocks drift past each other. All three are expected. The signals ring
buffer pushes one entry **per PPG sample** rather than per tick, so a zero-sample tick
does not put a hole in the streamed pulse trace.

> **The MAX30102 oscillator is the one unverified assumption in this chain.** Every
> resonator bin frequency is derived from `DSP_HZ = 25`, but the part clocks its FIFO off
> its own oscillator and the datasheet does not specify its tolerance. If it actually
> delivers 25.6 Hz, every reported BPM is 2.4 % low — uniformly, silently, with nothing
> else looking wrong. The firmware prints the measured rate in its `[PPG]` line every
> 10 s and `dsp_v2_sim.py --compare` prints it per capture, precisely so this is observed
> rather than assumed. See §4.2.

### 3.2 Pulse — sliding-DFT resonator bank

Per 25 Hz sample: EMA high-pass at 0.5 Hz, two cascaded EMA low-passes at 4 Hz (which
also removes a ~10.2 Hz tremor artifact with harmonics at 20.4 and 40.8 Hz that otherwise
dominates the spectrum), then 48 complex one-pole resonators spanning **40–190 BPM**,
τ = 10 s.

Unchanged by the MAX30102 swap — the input is still one float at 25 Hz. What changed is
its *scale*: 18-bit IR counts with a DC level in the tens of thousands, where the analog
path delivered a ~1800-count 12-bit reading. The chain is scale-free (the high-pass
removes DC, confidence is a power ratio), so no coefficient moved. The consequences are
at the edges: the perfusion index means something different (§3.5), and the BLE wire
format needed both pulse fields widened to 32 bits (§6).

The tremor low-pass is retained deliberately even though the artifact it was measured
against was a property of the analog sensor's mounting. 4 Hz is 240 BPM — above
`BPM_MAX` — so it costs nothing on cardiac signal either way, and removing it would be a
change made on the assumption that the new sensor has no HF artifact rather than on a
measurement showing it.

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

**Perfusion instead decides whether the rate is believable.** Above `PI_TRUST_MIN` the
pulse segment shows its colour gradient. Below it the device still knows it is worn — GSR
says so — but the pulse segment shows a colourless sweep instead of a confident wrong
number. At bio2 signal quality the tracker was measured reporting 113 BPM against a true
88, so a "searching" state is the honest display. GSR and temperature segments stay live
throughout, and the auto-strobe will not fire on an untrusted rate.

> **`PI_TRUST_MIN` is uncalibrated against the MAX30102.** It is still 0.40, the value
> derived between bio2's p90 and bio4's minimum on the *analog* sensor. It was carried
> forward rather than replaced with a guess at what the new part produces, because an
> invented number that looks plausible reads as calibrated and a known-stale one does
> not — the same lesson the old `PI_WORN_MIN` taught (a threshold from a different
> formula that sat below the unworn floor and therefore rejected nothing). It is
> runtime-tunable over BLE, so fixing it needs no reflash; §4.2 is the procedure.
>
> `CONFIG_SCHEMA_VERSION` was bumped to 2 for exactly this reason. The field set did not
> change, but a stored `piTrustMin` tuned on the old sensor is still perfectly in range
> for the new one and would load without complaint. The bump forces one fallback to
> compiled defaults so the threshold is re-derived rather than inherited.

**Known blind spot:** perfusion measures cardiac-band amplitude, and cannot tell a strong
pulse from strong motion. A handled or shaken board reaches 1.7 % and reads as trusted
while the tracker walks steadily up through implausible rates. Motion rejection is not
solved, and the MAX30102 does not solve it — a better PPG signal makes the motion
artifact cleaner too. That is still `-00y` (IMU).

**Unclaimed win: IR DC is a real wear signal.** §2.4 and the table above establish that
neither confidence nor the analog perfusion index could separate worn from unworn, which
is why GSR gates wear alone — and why `-6y4` (unworn sensors have no stable electrical
signature) is still open. The MAX30102 changes that premise: with its LED driven and its
ambient subtraction active, off-skin IR DC is a few thousand counts against tens of
thousands on skin, which is the clean separation the analog path never had. **This is not
implemented.** `WearDetect` still gates on GSR alone. Tracked as `-xe2`; it needs the
capture in §4.2 before a threshold can be picked honestly.

### 3.6 Hardware roadmap

**v2 PPG is done: the MAX30102 has replaced the analog pulse sensor** (`bd -kp5`, PPG
half). The motivation was §2.1 — SNR below 1 is the wall every DSP effort hit, and no
algorithm recovers a signal beneath its noise floor. The part brings its own LED driver,
18-bit ADC and ambient subtraction.

What the swap touched and what it did not:

| | |
|---|---|
| **Unchanged** | Every DSP coefficient. `PulseTracker` still takes one float at 25 Hz; the filters, the bank, the slew limiter and the phase oscillator are byte-identical. GSR is untouched end to end. |
| **Changed** | Sample source (I²C FIFO, not ADC), value scale (18-bit IR, not 12-bit), BLE wire format (v2 — both pulse fields widened to 32 bits), capture format (4-column with an `IrNew` flag), NVS config schema (v2). |
| **Invalidated** | §2.1–§2.4. SNR, perfusion index, `PI_TRUST_MIN`, and the harmonic-capture measurements were all properties of the analog path. `samples/bio2.log` and `bio4.csv` are no longer valid regression cases and no longer parse. |

**The calibration debt is real and is not paid.** `PI_TRUST_MIN` is a stale constant, the
LED drive current has never been set against skin, and the FIFO's true rate has not been
measured. §4.2 is the procedure; `samples/synthetic.csv` keeps the parity harness honest
in the meantime but says nothing about signal quality.

**Still planned, not done:**

- **ADS1115 for GSR** — the other half of `-kp5`. Its PGA at ±0.256 V gives ~7.8 µV/count
  against the ESP32's ~800 µV/count, and its ΔΣ data rate doubles as its filter: at
  8–16 SPS it rejects mains better than the boxcar of §3.1, letting GSR leave the 500 Hz
  path entirely. Deliberately deferred — GSR is not the channel that was failing, and
  doing both front ends at once would have meant no working reference for either.
- **IR-DC wear gating** (`-xe2`) — see §3.5.
- **IMU** (`-00y`) — independent of the front end and still the only real answer to
  motion. HR 110 from excitement and HR 110 from walking remain indistinguishable, and
  arm movement still produces SCR-shaped GSR artifacts that look exactly like arousal.
  A better PPG sensor does not help here; it makes the artifact cleaner too.

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
| `tools/dsp_v2_sim.py` | Python reference implementation of the whole pipeline, plus the legacy GSR engine for side-by-side comparison. Run against any capture. |
| `tools/dsp_v2_parity.cpp` + `.sh` | Compiles the **actual firmware** on the host with Arduino stubbed, replays a capture through the real `PulseTracker`/`GsrTracker`, and diffs against the Python. BPM/confidence/phase agree to 0.0000. |
| `tools/make_synthetic_capture.py` | Deterministic synthetic capture, so parity is checkable with no hardware and no recorded data. |
| `firmware/raw_streamer` | 4-column CSV capture: 500 Hz GSR plus flagged MAX30102 FIFO samples. |
| `firmware/dsp_v2` | On-device bench sketch; streams computed values as CSV, no LEDs. |
| `firmware/max30102_probe` | I²C bring-up diagnostic: bus scan, config register read-back, raw FIFO pointer dump. Not part of the product. |

The legacy *pulse* engine was removed from `dsp_v2_sim.py` along with the sensor it was
tuned for — its hardcoded operating point (baseline 1712, window 1670–1750, ×10 gain) is
a 12-bit reading of the XY1911-074, and running it against IR counts would emit beat
times that mean nothing. The comparison it existed to make is settled in §2.2 and does
not need re-answering on different hardware. The legacy GSR engine stays: same sensor,
same ADC, still a valid before/after.

**`samples/synthetic.csv` is a parity fixture, not a signal-quality reference.** Those
are two different jobs. *Do the two implementations compute the same numbers from the
same input* needs only an input that exercises every branch, and synthetic data does that
completely, without hardware — it currently holds C++ and Python to 0.0000 on
BPM/confidence/phase and 0.001 on arousal, tighter than any recorded capture ever did.
*Does the tracker read the right rate off a real wrist* needs real data with ground
truth, and nothing synthetic substitutes for it.

Ground truth for heart rate comes from a **manual radial pulse count taken during the
capture**, cross-checked against a Welch spectrum. Per-window Welch alone is not a
reliable reference — on `bio2.log` it swings 58–131 BPM between windows.

The synthetic fixture immediately earned its place: it exposed a latent bug in the Python
reference's `GsrTracker.reset()`, which seeded `prev` with the raw ADC value where `prev`
holds the *phasic* component (zero by construction after a reset). That injected a
`x × FS` slope on the next sample — 35 000 counts/s on a 1400-count signal — clamping to
`SLOPE_CLAMP` and saturating arousal for several seconds after every recalibration. The
firmware always had it right. It survived because every recorded capture starts with a
real contact transient that saturates arousal on *both* sides, so the two agreed at the
1.0 ceiling; only a capture with a quiet start separates them (0.107 against 1.000).

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

### 4.2 MAX30102 bring-up and calibration

Nothing in this subsection has been done yet. It is the debt §3.6 records, in the order
the steps depend on each other — each one is meaningless until the one above it passes.

**Step 0 — the bus survives.** ✅ *Done 2026-08-17.* Bus scan finds 0x57 and 0x76; both
devices answer. If a GY-MAX30102 board with 1.8 V pull-ups (§1) is fitted, *both* go
silent and the symptom presents as "the temperature sensor broke", not as a PPG problem.
`firmware/max30102_probe` scans the bus and reads every config register back.

**Step 1 — the FIFO rate.** ✅ *Done 2026-08-17: 25.09 Hz.* Everything downstream scales
by this.

```bash
just monitor --seconds 60 | grep '\[PPG\]'
```

Expect ~25 Hz. A steady offset scales every reported BPM by exactly its ratio — 25.6 Hz
would mean every rate reads 2.4 % low, silently. The measured 25.09 Hz is a 0.36 % bias,
far inside the 3.19 BPM bin spacing, so no correction is applied.

A rate *dip* means samples were lost to a stall. Do not look for an overflow count: this
part's `OVF_COUNTER` is not a loss counter (§1), and rate is the detector.

**Step 2 — LED current and DC operating point.** Strapped on, sitting still:

```bash
just monitor --seconds 30 | grep 'IR:'
```

| IR DC | Meaning | Action |
|---|---|---|
| < ~5 000 | Off skin, or LED current starved | Tighten strap; raise `MAX30102_LED_CURRENT` |
| ~20 000 – 150 000 | Healthy operating range | Proceed |
| > ~240 000 | Approaching the 262 143 full scale; cardiac AC is being clipped at the top | **Lower** `MAX30102_LED_CURRENT` |

The failure at both ends looks identical downstream — flat signal, no rate — so read the
DC number rather than inferring it from the LEDs. `MAX30102_LED_CURRENT` is a compile-time
constant in `max30102.h` and needs a reflash per value; it is deliberately not a BLE
tunable, because a value that saturates the ADC produces a capture that cannot be
salvaged offline and should not be one slider-drag away.

**Step 3 — capture and set `PI_TRUST_MIN`.** This replaces the stale 0.40 (§3.5). Three
captures, same procedure as the analog path used:

```bash
just monitor-save 60 unworn.log     # on the bench, nothing attached
just monitor-save 60 loose.log      # worn, deliberately slack strap
just monitor-save 180 good.log      # worn, firm contact, sitting still
```

Read `PerfIdx` from each. The threshold goes **between the loose-but-worn p90 and the
good-contact minimum** — the same construction as before, and for the same reason: any
value that rejects an empty sensor must not blank the panel on a real wearer with
mediocre contact. Then, without reflashing:

```bash
just config-set pi_trust_min <value>
```

Once it has held up across a few sessions, move the default in `dsp.h` so a config reset
lands somewhere sensible.

**Step 4 — a real regression capture.** Only after steps 1–3 pass:

```bash
just monitor-save 240 /dev/null &   # keep the port open
# flash firmware/raw_streamer, capture with a manual radial count running
```

Save it to `samples/` with the ground-truth rate in its name and record it in §5. That
capture is what finally lets §2.1–§2.3 be re-measured rather than carried as history, and
what lets `just test-parity` run against something real instead of the fixture.

**Step 5 — IR-DC wear gating** (`-xe2`). `unworn.log` and `good.log` from step 3 already
contain the data; if the two IR-DC populations separate cleanly, §3.5's premise changes
and `-6y4` becomes tractable.

---

## 5. Sample data

| File | What it is | Use |
|---|---|---|
| `samples/synthetic.csv` | 60 s generated, 64 BPM injected, deterministic | **Current parity fixture.** No signal-quality meaning |
| `samples/bio4.csv` | 226 s raw 500 Hz **analog PPG**, good contact, ground truth 64 BPM | Retired — GSR column still valid |
| `samples/bio2.log` | 150 s raw 500 Hz **analog PPG**, poor contact, SNR < 1, ~88 BPM | Retired — GSR column still valid |
| `samples/bio3.csv` | 204 s of on-device `dsp_v2` output (BPM, confidence, phase, arousal, tonic) | Retired for pulse; arousal/tonic still valid |

**There is currently no MAX30102 wrist capture.** The three `bio*` files are analog
front-end recordings; their pulse columns are readings of a sensor the bracelet no longer
has, and their three-column format no longer parses — `dsp_v2_sim.py` rejects them with
an explicit message rather than reinterpreting `RawPulse` as an IR channel, which would
have produced a full set of plausible numbers from the wrong hardware. They are kept
because their **GSR** columns are still a valid regression case for a channel nothing
changed about, and because §2 cites them.

Current raw capture format is `Timestamp_ms,RawIr,RawGSR,IrNew`. `IrNew=1` marks rows
carrying a genuinely new FIFO sample; on `IrNew=0` rows `RawIr` is `0` and must be
ignored — **not** held over, **not** interpolated. A zero-order hold was considered and
rejected: the 20-sample GSR boxcar would then average across IR sample boundaries
whenever the two clocks drifted out of alignment, which is an extra low-pass present in
the capture and not on the device, and it would make the C++/Python parity check inexact
in a way that reads as a rounding difference rather than a structural one.

Filling this gap is step 4 of §4.2.

---

## 6. Connectivity

BLE initialised at boot and advertising continuously. Gating it behind a button gesture was
considered and rejected: on datasheet estimates (not measured) the radio draws ~5–15 mA
against ~45 mA for the MCU and 50–150 mA for the LEDs, so under 5 % of the budget — not
worth a state machine and a third button gesture. Worth revisiting if actual current
measurement contradicts the estimate.

### Boot self-test

After the LED driver comes up, `bootSelfTest()` flashes the whole strip blue for
0.5 s (strip/wiring sanity), then holds 2 s of per-segment red/green: `SEG_A` = GSR
(pass = raw ADC reading off both rails, GSR has no self-ID like an I²C device),
`SEG_B` = MAX30102 found, `SEG_C` = BME280 found. The result also goes through
`BleService::log()`, so it's in the Log ring buffer for a client that connects after
boot, not just on the serial console at the moment it happened.

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

### Future idea: multi-device sync

Not designed or scoped, just noted: a broadcast mode so nearby armbands could sync
palettes/effects for a group of wearers at the same event. Would need its own GATT
service or advertising-payload scheme; no work has started on this.

### GATT — one custom service

A custom service rather than the standard Heart Rate Service: 0x180D has no way to express
confidence, wear state, or "this reading is currently untrustworthy", and publishing a bare
BPM that generic fitness apps treat as authoritative would be misleading given §2.

All little-endian. ATT MTU negotiated to 247 so the spectrum fits one packet.

Protocol is at **v2**. The MAX30102 forced it: `pulse_raw` is an 18-bit count and
`pulse_filtered` runs to thousands, against v1 fields sized `u16` and `i16×10` for a
12-bit ADC reading ~1800. Left alone, the raw value would have wrapped and the filtered
value saturated at 3276.7 — both of which decode to a plausible number rather than an
error, which is the exact failure this library exists to prevent. Both widened to 32 bits;
`BLE_VITALS_LEN` 18 → 20, `BLE_SIGNALS_LEN` 46 → 66.

| Characteristic | Mode | Rate | Size | Contents |
|---|---|---|---|---|
| Vitals | notify | 4 Hz | 20 B | bpm×10, confidence, arousal, perfusion×100, gsr_raw, **pulse_raw (u32)**, temp×100, flags (worn/strobe/mode), brightness, gsr_tonic |
| Signals | notify | 5 Hz | 66 B | 5 batched 25 Hz samples: **pulse_filtered (i32×10)**, **pulse_raw (u32)**, gsr_phasic, gsr_raw + start timestamp |
| Spectrum | notify | 1 Hz | 56 B | 48 log-scaled resonator bin powers + peak bin + BPM range |
| Control | write | — | 2–3 B | set mode, set brightness, recalibrate GSR, set stream mask, reset bank, reset config, dump log, enter bootloader |
| Config | read/write | — | var | `[paramId][float32]` writes; read returns all tunables packed |
| Info | read | — | var | firmware version, build date, bin count |
| Log | notify | on error / on demand | var | plain-ASCII `"<seconds>s <message>"`, no binary layout -- a debug aid, not the scientific data path |

Signals and Spectrum are **opt-in** via the Control stream mask. A casual phone connection
costs ~64 B/s; full dev streaming ~280 B/s.

**Log** is a 20-entry ring buffer (`BLE_LOG_DEPTH`) of warnings/errors
(`BleService::log()`, printf-style, always mirrored to Serial regardless of BLE
state). Writing `CMD_DUMP_LOG` to Control replays the buffered history oldest-first
over Log notifications; new entries then notify live. The web app subscribes and
requests the replay right after connecting, showing a collapsible "Device log" panel.
Kept intentionally outside the versioned binary protocol -- it doesn't need
`BLE_PROTOCOL_VERSION` discipline since it's for a human, not a decoder.

The Spectrum characteristic exists specifically to watch harmonic capture (§2.3) happen
live, which offline analysis could only infer.

**`CMD_ENTER_BOOTLOADER` (0x08)** reboots the board into the ESP32-S3 ROM USB download
mode, so a bracelet already installed in the sleeve rig can be re-flashed without
physical access to the BOOT and RESET buttons. The firmware sets
`RTC_CNTL_FORCE_DOWNLOAD_BOOT` and calls `esp_restart()`; that flag survives exactly the
one reset it triggers, so the device enumerates as a bootloader on the native USB port
and the *next* ordinary power cycle boots the firmware again. One-shot on purpose --
losing button access must not also mean a board can get stranded in download mode.

It is the only Control command that ends the session rather than changing a display, so
it is the only one with a mandatory magic argument (`CMD_BOOTLOADER_MAGIC`, 0xB0): a
truncated or corrupted write cannot land on it, and it is unreachable from a client that
only speaks the one-byte command shape. The web app arms it with a second confirming tap
before sending; `tools/blectl.py bootloader` prompts (or takes `--yes`). Both then tell
you to flash, since the device is unreachable over BLE until you do. In practice
`just flash` auto-resets over CDC without any button press, so this command is the
backup path rather than the primary one.

The command is dispatched on the NimBLE task but acted on in `loop()`, so it depends on
the render loop actually running. That is guaranteed only because serial writes are
non-blocking (`Serial.setTxTimeoutMs(0)`, -2po): with the default 250 ms timeout, a
plugged-in cable whose port nobody was draining stalled `loop()` and swallowed this
command silently, while NimBLE went on advertising and answering reads from its own
task -- the device looked alive and simply ignored the command. Before restarting, the firmware persists any pending config write (the
debounced NVS save is up to two seconds away and there is no next frame) and blanks the
strip, so "in download mode" looks different from a hung render loop.

**Config persists to NVS**, with an explicit reset-to-defaults command exposed in the web
app.

### Firmware structure

BLE pushes `main_armband.ino` past 900 lines, so the sketch folder splits into units with
one responsibility each:

| File | Responsibility | Depends on |
|---|---|---|
| `main_armband.ino` | setup, FreeRTOS tasks, render loop | all |
| `libraries/BraceletDSP/src/dsp.h` | `PulseTracker`, `GsrTracker`, `WearDetect` | `<math.h>`, `<stdint.h>` |
| `libraries/BraceletMAX30102/src/max30102.h` | MAX30102 config + FIFO drain | `<Wire.h>` |
| `ble_service.h` / `.cpp` | GATT setup, packet packing, command dispatch | `BiometricData` snapshot + control callback |
| `config.h` / `.cpp` | tunables struct, NVS load/save/reset | NVS |

The load-bearing constraint is that **`dsp.h` stays free of Arduino and BLE types**. That
is what allows `tools/dsp_v2_parity.sh` to keep compiling the real trackers on the host
and proving them equal to the Python reference. BLE reads a snapshot struct and never
reaches into tracker internals.

The MAX30102 driver is a **separate library for that reason**, not for tidiness: it needs
`<Wire.h>` and so could never live in `BraceletDSP` without costing the validation path.
Its scope is deliberately narrow — configure the part, drain the FIFO, hand out IR counts.
No beat detection, no SpO2, no filtering. SparkFun's MAX3010x library was not used
because it ships its own heart-rate and SpO2 estimators, which would be a second,
unvalidated source of numbers competing with the resonator bank that §4 exists to check.

Shipped as an Arduino library (`--libraries ./libraries`) rather than a sketch-local
header, because `dsp_v2` and `main_armband` both need it and previously carried duplicate
copies that could drift apart. `WearDetect` reports a release via a `justReleased` flag
rather than clearing the resonator bank itself — that coupling would have dragged the
whole pulse engine into a class that answers one yes/no question.

### Consumers

- **Web app** — single self-contained `webapp/index.html` using Web Bluetooth: live vitals
  tiles, scrolling chart for the 25 Hz signals, bar chart for the 48-bin spectrum, and
  controls for mode / brightness / recalibrate plus config sliders with reset-to-defaults.
  The **IR raw** row names its state (`off skin` / `weak` / `good` / `CLIPPING`) against
  the §4.2 bands rather than showing a bare count, because the two failure modes are
  indistinguishable downstream and this is the readout you watch while adjusting strap
  tension. Its bar is piecewise-scaled so each band gets a fixed share of the track —
  linear against the 18-bit full scale crushed the whole off-skin→good transition into
  the bottom third, which is precisely the range being steered. The perfusion bar is
  scaled so the device's *live* `pi_trust_min` sits at half full, so it stays meaningful
  without a per-sensor constant and self-corrects when the threshold is retuned.
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
