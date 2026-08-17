# Project Overview: Biometric Status Bracelet / Armband

A wearable festival/party biometrics panel worn on the forearm via a fabric sleeve with velcro closure. It processes real-time biometric signals and displays interactive lighting effects on an LED matrix/bar display.

## Hardware Specification

### Controller & Power
- **MCU:** ESP32-S3 DevKit (Standard DevKitC-1 or similar dual USB-C/micro-USB).
  - *Note on Form Factor:* For the final wearable enclosure on the forearm, a smaller **Seeed Studio XIAO ESP32-S3** or **Adafruit QT Py ESP32-S3** can be swapped in if space on the sleeve is tight, but DevKit works perfectly for prototyping and wearability testing.
- **Power:** 18650 rechargeable battery pack supplying regulated 5V output.
- **Wearable Mounting:** Fabric forearm sleeve base with velcro strap.

### Sensors & Inputs
1. **GSR (Galvanic Skin Response):** Seeed Studio Grove - GSR Sensor v1.2
   - Powered from 3.3V rail. Signal (SIG) to GPIO 1 (ADC1_CH0). TP1 left disconnected.
   - Measures skin resistance / stress / excitement.
2. **Pulse Sensor:** **MAX30102** digital reflective photoplethysmogram (PPG), $I^2C$ address **0x57**
   - Powered from 3.3V rail. Shares SDA (GPIO 8) / SCL (GPIO 9) with the temperature sensor — no address collision. INT to GPIO 10 (wired, not currently read).
   - Red + IR LEDs, 18-bit ADC, on-chip ambient-light subtraction. IR channel is the one used.
   - Configured for 100 Hz internal sampling with 4x on-chip averaging, so its FIFO delivers exactly 25 Hz — the rate the DSP already runs at. Does **not** use the ESP32 ADC.
   - Replaced the analog XY1911-074 B506, whose SNR was below 1 on this hardware. See DESIGN.md §3.6.
   - **Caveat:** many cheap purple GY-MAX30102 breakouts tie the SDA/SCL pull-ups to an internal 1.8V rail, which drops the whole bus below the ESP32's input-high threshold and silently kills the BME280 too. Cut them, fit 4.7kΩ to 3.3V.
3. **Temperature Sensor:** Digital $I^2C$ Sensor (**TMP102**, **HDC1080**, or **BME280**)
   - Powered from 3.3V rail. Connected via SDA (GPIO 8) and SCL (GPIO 9).
   - Micro-amp ($\mu\text{A}$) ultra-low power consumption for maximum 18650 battery life. High precision skin/ambient temperature. Zero analog noise!
4. **Physical Tactile Button:** Connected between GPIO 5 and GND (using internal pull-up resistor).
   - Functions: Short press = cycle visual modes / toggle brightness; Long press (2s) = recalibrate GSR baseline.

### Output / Visual Display
- **Addressable LED Strip:** WS2812B (60 LEDs/m density).
- **Layout:** 21 LEDs (3x7 matrix) or 18 LEDs (3x6 matrix) arranged in 3 parallel serpentine or parallel segments.
  - Data pin: Connected to GPIO 4 via 330Ω resistor. Power: 5V rail from 18650 pack.
- **Visual Segments:**
  1. **Bar 1 - Heart Pulse Segment (7 LEDs):** Thumping / glowing cardiac wave animation, color-coded by BPM range (e.g. relaxed blue/green -> energetic red/magenta).
  2. **Bar 2 - Excitement / Stress Bar (7 LEDs):** Dynamic multi-color VU-meter driven by GSR (skin conductance delta).
  3. **Bar 3 - Temperature Meter (7 LEDs):** Thermal gradient gauge (Cool Blue -> Yellow -> Warm Red/Pink).

### Connectivity & Control
- **BLE (Bluetooth Low Energy):**
  - Serves live biometric values (BPM, GSR raw & dynamic arousal level, Temperature °C).
  - Allows smartphone app / web-bluetooth page to tweak LED brightness, change palettes, or trigger manual party/hype modes.
  - Potential multi-device broadcast mode for festival sync with other armbands nearby.

---

## Technical Considerations & Signal Processing

### Voltage & ADC Safety
- All sensors operate off the ESP32-S3 **3.3V rail**.
- On ESP32-S3, Wi-Fi can interfere with ADC2 pins. Therefore, **all analog sensors MUST be assigned to ADC1 pins** (GPIO 1 through 10). GSR is now the only analog sensor.
- The MAX30102's IR LED pulses at ~50 mA. Decouple it at the module (10 µF + 0.1 µF) or the transient rides the shared 3.3V rail onto the GSR line, whose features of interest are only 10–40 counts.
- WS2812B data pin (GPIO 4) includes a 330Ω inline resistor. Most 18–21 LED WS2812B strips accept 3.3V logic directly; if flickering occurs, use the diode drop trick or 74AHCT125 level shifter.

### Pin Allocation Map (Default)
| Component | Function | Suggested Pin |
|---|---|---|
| WS2812B Data | LED Strip Data Signal | GPIO 4 |
| Grove GSR v1.2 | Analog Skin Conductance | GPIO 1 (ADC1_CH0) |
| MAX30102 + Temp Sensor | Shared $I^2C$ Data (SDA) | GPIO 8 |
| MAX30102 + Temp Sensor | Shared $I^2C$ Clock (SCL) | GPIO 9 |
| MAX30102 INT | PPG_RDY interrupt (wired, unused) | GPIO 10 |
| Button | Mode / Recalibrate Switch | GPIO 5 (Input Pullup) |

$I^2C$ bus runs at 400 kHz. Addresses: MAX30102 `0x57`, BME280 `0x76`/`0x77`.

GPIO 2 (formerly the analog pulse sensor) is now free.

### Visual & Interactive Modes

1. **Standard Biometric Panel (Default):**
   - Bar 1: Pulse Heartbeat Beat & Wave
   - Bar 2: Excitement Meter (GSR)
   - Bar 3: Body Temperature Bar
2. **Overdrive / Hype Party Mode (Auto-Triggered or Button):**
   - Activated when High BPM + High GSR Excitement occur simultaneously.
   - Matrix-wide strobing, rainbow chases, or fire effect across all 3 segments.
3. **BLE Control / Stealth Mode:**
   - Low-brightness night mode or custom user-selected palette via BLE commands.

