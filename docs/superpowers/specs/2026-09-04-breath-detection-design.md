# Breath detection: measured respiration as a first-class signal

Status: design approved 2026-09-04. Implements the remaining scope of bead
`-lc9`. Consumers are split into `-g9w`; the GSR finding below is `-0b6`.

## 1. What the measurements settled

Three captures were recorded with `tools/paced_capture.py`, which paces the
wearer and writes its cue timings into the same CSV as `# MARK` rows, so the
analysis never aligns two clocks:

| capture | protocol |
|---|---|
| `samples/breath_paced_01.csv` | 6 and 12 br/min, 60 s each, plus hold |
| `samples/breath_paced_02.csv` | same rates, 180 s each — the confirmation run |
| `samples/breath_sigh_01.csv` | 7 cued deep breaths, 60 s apart |

`tools/breath_analysis.py` scores five candidate signals against those marks.
The decisive statistic is a cue-locked cycle fold — drift and one-off
artifacts are uncorrelated with breath phase, so averaging N cycles suppresses
them by sqrt(N) while anything genuinely breath-locked survives. Every fold is
paired with a null control folded at 1.37x the true period.

From `breath_paced_02.csv` (z, peak-to-peak of the folded cycle over its own
standard error; `null` is the control):

| stage | n | arousal | gsr_phasic | gsr_bp | ppg_riiv | ppg_rsa |
|---|---|---|---|---|---|---|
| paced6 | 17 | 3.5 / 4.4 | 20.6 / 21.2 | 3.7 / 4.7 | **7.9 / 4.1** | 5.5 / 3.5 |
| paced12 | 35 | 2.3 / 2.5 | 2.3 / 2.7 | 2.9 / 3.3 | **14.8 / 3.1** | 8.9 / 3.2 |

Only `ppg_riiv` beats its null at both rates. Its peak-picker recovers 6.0 and
12.0 br/min exactly, at SNR 65.6 and 105.9, without being told what to look
for. `ppg_rsa` agrees in phase and is real at 12/min (z=8.9) but roughly ten
times noisier at 6/min, consistent with RSA amplitude falling as breathing
slows.

**Decision: RIIV is canonical.** It also needs no beat detection, which
matters because `PI_TRUST_MIN` is met only part of the time on this hardware.

### 1.1 Two negative results that change the ticket

**Steady breathing does not modulate arousal.** Folded arousal amplitude is
+/-0.002 over 35 cycles, sitting at its null. This is not a sensitivity
limit of the capture: the GSR channel spans 890 counts across 387 distinct
levels with the breath band 75x above its own noise floor, and `gsr_bp`
band-passes the raw signal without touching `PHASIC_TAU`/`TONIC_TAU` at all —
same null. There is no periodic respiratory component in the phasic path, so
the adaptive notch proposed in `-lc9` has nothing to remove.

**Arousal is far too insensitive, not too sensitive.** `-lc9` reports the bar
saturating once per breath. Over a 7-minute session containing seven
deliberate SCR provocations, arousal had median 0.000, p90 0.002, and a
maximum of 0.652 — it never saturated. Tracked to `FLOOR_MIN`; filed as
`-0b6` and out of scope here.

What GSR *does* respond to is isolated sighs — 7/7 cued deep breaths, median
per-epoch z=6.9, against the `ppg_riiv` positive control at 8.4. That is the
sigh-to-SCR mechanism already described in `DESIGN.md:614`, and it is a
different phenomenon from tidal breathing.

## 2. `BreathTracker`

New struct in `libraries/BraceletDSP/src/dsp.h`, fed the same 25 Hz IR stream
`PulseTracker` already receives. It reuses that struct's proven shape — a
resonator bank yielding rate, phase and confidence from one structure —
retargeted an octave and a half down.

### 2.1 Band extraction

RIIV lives at 0.05-0.6 Hz on top of an IR baseline that is orders of
magnitude larger. Extract it as the difference of two one-pole EMAs:

```
slow += aSlow * (x - slow);     // BREATH_SLOW_TAU  8.0 s
fast += aFast * (x - fast);     // BREATH_FAST_TAU  0.3 s
float y = fast - slow;
```

Two EMAs rather than a biquad: it matches the existing `onepoleAlpha` idiom in
this file, carries no filter-state initialisation problem on a fresh wearer,
and cannot ring on the contact steps this hardware produces regularly.

### 2.2 Bank

`N_BREATH_BINS` = 24 spanning `BREATH_MIN` 4.0 to `BREATH_MAX` 30.0 br/min,
identical in structure to `PulseTracker`'s: per-bin rotator, complex one-pole
resonator, periodic renormalisation, parabolic interpolation of the winning
bin, and `arg(R * conj(rot))` for running phase. `BREATH_BANK_TAU` = 40 s,
scaled to breath timescales the way `BANK_TAU` = 10 s is scaled to cardiac
ones.

Cost is ~24 bins at 25 Hz, about 7 kflop/s — half what the pulse bank already
spends, and negligible on an ESP32. 24 rather than 16 so that bin spacing is
1.13 br/min: the +/-1 br/min assertion in section 4 then holds on bin spacing
alone, instead of resting entirely on the parabolic interpolation.

Edge bins are excluded from the peak search, as in `PulseTracker::estimate`:
they have no neighbour on one side, which makes the parabolic interpolation
unsafe there.

### 2.3 Confidence gate

`hold` in both paced captures makes an ungated peak-picker report ~4 br/min,
read off slow hemodynamic drift while the wearer is not breathing at all.
Below `BREATH_CONF_GATE` the tracker must freeze its rate and report
untrusted, exactly as `CONF_GATE` parks BPM on its last good value.

`breathTrusted` is published as a flag; a client that ignores it can render a
plausible rate during apnea, which is the failure this gate exists to prevent.

## 3. Protocol v2 -> v3

`BleVitals` grows from 20 to 22 bytes. MTU is 247 (`ble_service.cpp:222`), so
there is no fragmentation risk.

| byte | field | note |
|---|---|---|
| 19 | `breathPhase` | u8, 0..255 -> 0..2*PI. **Meaning changes**: measured, not paced |
| 20 | `breathRpm` | u8, x4 -> 0..63 br/min at 0.25 resolution |
| 21 | `breathConf` | u8, 0..255 |

`breathTrusted` takes flags bit `0x20` (`VF_BREATH_TRUSTED`), the next free bit
above `VF_MODE_MASK`.

Byte 19 is the subtle change: it currently carries the open-loop pacer phase
from `advanceBreathPhase()` and is documented as "mode 3 only". After this it
carries measured phase in every mode. Bumping `BLE_PROTOCOL_VERSION` to 3
forces `tools/bracelet_protocol.py`, the webapp decoder and
`tools/ble_packet_test.cpp` to move together — which is what the version byte
is for.

## 4. Validation

1. **Parity.** `tools/dsp_v2_sim.py` gets a `BreathTracker` mirror; `just
   test-parity` covers it C++/Python bit-identically, as it already does for
   the other two trackers.
2. **Wire format.** `tools/ble_packet_test.cpp` gains v3 golden fixtures;
   `just test-proto` decodes them from Python.
3. **Regression fixtures.** All three captures are committed. Assertions:
   - `breath_paced_02.csv`: recovered rate within +/-1 br/min on both paced
     stages
   - `breath_paced_01.csv`: same, at the shorter stage lengths
   - both: `breathTrusted` false for the majority of `hold`
4. **Falsifier.** The `hold` assertion is the one that must not be relaxed. A
   tracker that reports a confident rate during apnea has failed regardless of
   how well it scores on the paced stages.

## 5. Out of scope

- `-0b6`: the `FLOOR_MIN` dead zone. Independent of breath.
- `-g9w`: webapp trace, LED phase-lock from measured phase, honest rendering
  below the confidence gate.
- Fusing RIIV with RSA. RSA is real at 12/min and corroborates the mechanism,
  but combining two estimators before either is deployed adds a failure mode
  with no measured benefit. Revisit if RIIV proves fragile in use.
