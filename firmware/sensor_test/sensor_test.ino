/*
 * Biometric Status Bracelet - Hardware & Sensor Diagnostic Tool
 * Target MCU: ESP32-S3 DevKit
 * 
 * Hardware Connections:
 * - Grove GSR v1.2      -> GPIO 1 (ADC1_CH0) [Power from 3.3V]
 * - XY1911-074 Pulse    -> GPIO 2 (ADC1_CH1) [Power from 3.3V]
 * - BME280 (I2C)        -> SDA: GPIO 8, SCL: GPIO 9 [Power from 3.3V]
 * - User Button         -> GPIO 5 (Input Pullup, short to GND when pressed)
 * - WS2812B LED Strip   -> GPIO 4 (Data) [Power from 5V]
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>

// Pin Definitions
#define PIN_GSR         1
#define PIN_PULSE       2
#define PIN_SDA         8
#define PIN_SCL         9
#define PIN_BUTTON      5
#define PIN_LED         4

#define NUM_LEDS        21  // 3 segments x 7 LEDs

// Objects
Adafruit_BME280 bme;
Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// Status flags
bool bmeFound = false;

// Dynamic GSR Zeroing / Baseline
uint16_t gsrBaseline = 2000;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for serial console to open

  Serial.println(F("\n=============================================="));
  Serial.println(F(" BIOMETRIC BRACELET - SENSOR DIAGNOSTIC TEST "));
  Serial.println(F("=============================================="));

  // Initialize GPIOs
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  analogReadResolution(12); // 0 - 4095

  // Initialize LEDs
  strip.begin();
  strip.setBrightness(40); // 0-255 (low brightness for testing)
  strip.show();

  // Test LED color sequence (Red, Green, Blue) to verify pixel ordering
  Serial.print(F("[LED Strip Test] Lighting up 3x7 matrix... "));
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < 7) strip.setPixelColor(i, strip.Color(255, 0, 0));       // Segment 1 Red
    else if (i < 14) strip.setPixelColor(i, strip.Color(0, 255, 0));  // Segment 2 Green
    else strip.setPixelColor(i, strip.Color(0, 0, 255));              // Segment 3 Blue
  }
  strip.show();
  delay(1000);
  strip.clear();
  strip.show();
  Serial.println(F("OK!"));

  // Initialize I2C and BME280
  Serial.print(F("[BME280 Test] Initializing I2C (SDA=8, SCL=9)... "));
  Wire.begin(PIN_SDA, PIN_SCL);

  // Try standard I2C addresses 0x76 and 0x77
  if (bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire)) {
    bmeFound = true;
    Serial.println(F("SUCCESS! (BME280 detected)"));
  } else {
    Serial.println(F("FAILED! (Check BME280 wiring or I2C address)"));
  }

  Serial.println(F("\n--- DIAGNOSTIC STREAMING STARTED ---"));
  Serial.println(F("Format: Pulse_RAW | GSR_RAW | GSR_Delta | Temp_C | Humidity_% | Button"));
  Serial.println(F("Press Button on GPIO 5 to calibrate/zero GSR baseline!\n"));
  delay(1000);
}

unsigned long lastBmeReadTime = 0;
float currentTemp = 0.0;
float currentHumidity = 0.0;

void loop() {
  // 1. Read Analog PPG Pulse Sensor
  uint16_t pulseRaw = analogRead(PIN_PULSE);

  // 2. Read Analog GSR Sensor
  uint16_t gsrRaw = analogRead(PIN_GSR);

  // 3. Button Handler (Short press re-calibrates GSR baseline)
  bool buttonPressed = (digitalRead(PIN_BUTTON) == LOW);
  if (buttonPressed) {
    gsrBaseline = gsrRaw;
    // Flash LEDs white to signal calibration
    for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(100, 100, 100));
    strip.show();
  } else {
    strip.clear();
  }

  // Calculate GSR conductance shift / delta from baseline
  int16_t gsrDelta = (int16_t)gsrRaw - (int16_t)gsrBaseline;

  // 4. Read BME280 Temperature & Humidity every 500ms
  if (bmeFound && (millis() - lastBmeReadTime > 500)) {
    currentTemp = bme.readTemperature();
    currentHumidity = bme.readHumidity();
    lastBmeReadTime = millis();
  }

  // 5. Visual LED feedback on segment 1 (Pulse trace light)
  uint8_t pulseBrightness = map(constrain(pulseRaw, 1000, 3500), 1000, 3500, 0, 255);
  strip.setPixelColor(0, strip.Color(pulseBrightness, 0, pulseBrightness / 2));
  strip.show();

  // 6. Print Serial Plotter friendly output
  // You can open "Serial Plotter" in Arduino IDE to graph these values live!
  Serial.print("Pulse_RAW:");
  Serial.print(pulseRaw);
  Serial.print(" GSR_RAW:");
  Serial.print(gsrRaw);
  Serial.print(" GSR_Baseline:");
  Serial.print(gsrBaseline);
  Serial.print(" Temp_C:");
  Serial.print(currentTemp);
  Serial.print(" Humidity_%:");
  Serial.print(currentHumidity);
  Serial.print(" Button:");
  Serial.println(buttonPressed ? 4000 : 0);

  // 20ms delay (~50Hz sampling rate for high resolution pulse wave)
  delay(20);
}
