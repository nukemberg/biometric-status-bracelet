#include "ble_service.h"

#include <stdarg.h>

#if ENABLE_BLE

#include <Arduino.h>
#include <NimBLEDevice.h>

#include <dsp.h>   // N_BINS and BPM range, reported in the Info characteristic

namespace {

NimBLEServer *server = nullptr;
NimBLECharacteristic *vitalsChr = nullptr;
NimBLECharacteristic *infoChr = nullptr;
NimBLECharacteristic *controlChr = nullptr;
NimBLECharacteristic *signalsChr = nullptr;
NimBLECharacteristic *spectrumChr = nullptr;
NimBLECharacteristic *configChr = nullptr;
NimBLECharacteristic *logChr = nullptr;
bool connected = false;
uint8_t streams = 0;
BleService::Handlers handlers;

// Ring buffer for BleService::log(). Overwrites oldest on wrap, same policy as the
// signals ring buffer in main_armband.ino: a debug aid that falls behind must show
// the most recent history, not the oldest.
struct LogEntry {
  uint32_t ms = 0;
  char text[64] = {0};
};
LogEntry logRing[BLE_LOG_DEPTH];
uint8_t logHead = 0;    // next slot to write
uint8_t logCount = 0;   // valid entries, saturates at BLE_LOG_DEPTH

// Single-entry notify payload: plain ASCII "<seconds>s <message>", no header. Debug
// aid, not the scientific data path -- see the comment on BLE_LOG_DEPTH.
void notifyLogEntry(const LogEntry &e) {
  if (!logChr || !connected) return;
  char buf[80];
  int n = snprintf(buf, sizeof buf, "%lus %s", e.ms / 1000UL, e.text);
  if (n < 0) return;
  if ((size_t)n >= sizeof buf) n = sizeof buf - 1;
  logChr->setValue((uint8_t *)buf, (size_t)n);
  logChr->notify();
}

// Human-readable on purpose. This is the first thing anyone reads when debugging with
// nRF Connect, and a packed binary blob would need a decoder to answer "what is this
// and does my client match it". Cheap to parse either way.
String buildInfoString() {
  String s;
  s += "fw=2.0.0";
  s += ";proto=" + String(BLE_PROTOCOL_VERSION);
  s += ";built=" __DATE__ " " __TIME__;
  s += ";bins=" + String(N_BINS);
  s += ";bpm=" + String((int)BPM_MIN) + "-" + String((int)BPM_MAX);
  return s;
}

// Every command is validated before dispatch. A malformed write from any client in
// range must not be able to put the device into a state it cannot render -- an
// out-of-range display mode would index past the end of the mode switch.
class ControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &) override {
    const std::string &v = chr->getValue();
    if (v.empty()) {
      Serial.println(F("[BLE] empty control write ignored"));
      return;
    }
    const uint8_t cmd = (uint8_t)v[0];
    const bool hasArg = v.size() >= 2;
    const uint8_t arg = hasArg ? (uint8_t)v[1] : 0;

    switch (cmd) {
      case CMD_SET_MODE:
        if (!hasArg || arg > 2) {
          Serial.println(F("[BLE] set-mode: bad argument"));
          return;
        }
        if (handlers.setMode) handlers.setMode(arg);
        break;

      case CMD_SET_BRIGHTNESS:
        if (!hasArg) {
          Serial.println(F("[BLE] set-brightness: missing argument"));
          return;
        }
        if (handlers.setBrightness) handlers.setBrightness(arg);
        break;

      case CMD_RECALIBRATE_GSR:
        if (handlers.recalibrateGsr) handlers.recalibrateGsr();
        break;

      case CMD_SET_STREAMS:
        if (!hasArg) {
          Serial.println(F("[BLE] set-streams: missing argument"));
          return;
        }
        streams = arg & (STREAM_SIGNALS | STREAM_SPECTRUM);
        if (handlers.setStreams) handlers.setStreams(streams);
        break;

      case CMD_RESET_BANK:
        if (handlers.resetBank) handlers.resetBank();
        break;

      case CMD_RESET_CONFIG:
        if (handlers.resetConfig) handlers.resetConfig();
        break;

      case CMD_DUMP_LOG: {
        // Oldest first, so the client's log view fills in chronological order
        // rather than needing to re-sort a batch of out-of-order notifications.
        uint8_t start = (logCount < BLE_LOG_DEPTH) ? 0
                        : logHead;  // buffer has wrapped; logHead is the oldest slot
        for (uint8_t i = 0; i < logCount; i++) {
          notifyLogEntry(logRing[(start + i) % BLE_LOG_DEPTH]);
        }
        break;
      }

      default:
        Serial.print(F("[BLE] unknown command 0x"));
        Serial.println(cmd, HEX);
        return;
    }

    Serial.print(F("[BLE] cmd 0x"));
    Serial.print(cmd, HEX);
    if (hasArg) {
      Serial.print(F(" arg "));
      Serial.print(arg);
    }
    Serial.println();
  }
};

ControlCallbacks controlCallbacks;

// The config characteristic carries the same [paramId][f32] record in both
// directions: a write is one record applying a single tunable, a read is
// CFG_PARAM_COUNT records returning all of them. The read is populated on demand
// so it always reflects live state rather than a snapshot taken at some earlier
// point -- a hue anchor changed moments ago must read back as changed.
class ConfigCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic *chr, NimBLEConnInfo &) override {
    if (!handlers.getConfig) return;
    float values[CFG_PARAM_COUNT];
    handlers.getConfig(values);
    uint8_t buf[BLE_CONFIG_READ_LEN];
    blePackConfigRead(buf, values);
    chr->setValue(buf, sizeof buf);
  }

  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &) override {
    const std::string &v = chr->getValue();
    uint8_t paramId = 0;
    float value = 0.0f;
    if (!bleUnpackConfigWrite(reinterpret_cast<const uint8_t *>(v.data()),
                              v.size(), paramId, value)) {
      Serial.println(F("[BLE] config write: malformed or unknown paramId"));
      return;
    }
    if (handlers.setConfigParam) handlers.setConfigParam(paramId, value);
    Serial.print(F("[BLE] config set param 0x"));
    Serial.print(paramId, HEX);
    Serial.print(F(" = "));
    Serial.println(value, 4);
  }
};

ConfigCallbacks configCallbacks;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &info) override {
    connected = true;
    Serial.print(F("[BLE] connected, peer "));
    Serial.println(info.getAddress().toString().c_str());
  }

  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &info, int reason) override {
    connected = false;
    // Streams are per-session. Leaving them on would keep the DSP task packing
    // notifications nobody receives, and would surprise the next client.
    streams = 0;
    Serial.print(F("[BLE] disconnected, reason "));
    Serial.println(reason);
    // Without this the bracelet becomes invisible after the first client leaves,
    // which looks exactly like a crash from the phone's side.
    NimBLEDevice::startAdvertising();
  }
};

ServerCallbacks serverCallbacks;

}  // namespace

namespace BleService {

void begin(const char *deviceName, const Handlers &h) {
  handlers = h;
  NimBLEDevice::init(deviceName);

  // The spectrum packet is 56 bytes and the signals packet 46, both beyond the 23-byte
  // default ATT MTU. Ask for enough that each fits a single notification -- fragmented
  // notifications would arrive as partial packets the decoders would reject.
  NimBLEDevice::setMTU(247);

  server = NimBLEDevice::createServer();
  server->setCallbacks(&serverCallbacks);

  NimBLEService *svc = server->createService(BLE_SVC_BRACELET);

  vitalsChr = svc->createCharacteristic(
      BLE_CHR_VITALS, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  signalsChr = svc->createCharacteristic(BLE_CHR_SIGNALS, NIMBLE_PROPERTY::NOTIFY);
  spectrumChr = svc->createCharacteristic(BLE_CHR_SPECTRUM, NIMBLE_PROPERTY::NOTIFY);

  controlChr = svc->createCharacteristic(
      BLE_CHR_CONTROL, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  controlChr->setCallbacks(&controlCallbacks);

  configChr = svc->createCharacteristic(
      BLE_CHR_CONFIG, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  configChr->setCallbacks(&configCallbacks);

  infoChr = svc->createCharacteristic(BLE_CHR_INFO, NIMBLE_PROPERTY::READ);
  infoChr->setValue(buildInfoString().c_str());

  logChr = svc->createCharacteristic(BLE_CHR_LOG, NIMBLE_PROPERTY::NOTIFY);

  svc->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_BRACELET);
  adv->setName(deviceName);
  adv->enableScanResponse(true);
  adv->start();

  Serial.print(F("[BLE] advertising as "));
  Serial.print(deviceName);
  Serial.print(F(" | "));
  Serial.println(buildInfoString());
}

void publishVitals(const BleVitals &v) {
  if (!vitalsChr) return;

  uint8_t buf[BLE_VITALS_LEN];
  size_t n = blePackVitals(buf, v);

  // Keep the value current even while disconnected so a fresh client's first READ
  // returns real data rather than zeros, but only notify when someone is listening.
  vitalsChr->setValue(buf, n);
  if (connected) vitalsChr->notify();
}

bool publishSignals(uint32_t firstTimestampMs, const BleSignalSample *samples,
                    uint8_t count) {
  if (!signalsChr || !connected || !(streams & STREAM_SIGNALS) || count == 0) {
    return false;
  }
  uint8_t buf[BLE_SIGNALS_LEN];
  size_t n = blePackSignals(buf, firstTimestampMs, samples, count);
  signalsChr->setValue(buf, n);
  return signalsChr->notify();
}

void publishSpectrum(const float *binPower, uint8_t bins, uint8_t peakBin,
                     float bpmLo, float bpmHi) {
  if (!spectrumChr || !connected || !(streams & STREAM_SPECTRUM)) return;
  uint8_t buf[BLE_SPECTRUM_LEN];
  size_t n = blePackSpectrum(buf, binPower, bins, peakBin, bpmLo, bpmHi);
  spectrumChr->setValue(buf, n);
  spectrumChr->notify();
}

uint8_t streamMask() { return streams; }

bool isConnected() { return connected; }

void log(const char *fmt, ...) {
  char text[sizeof(LogEntry::text)];
  va_list args;
  va_start(args, fmt);
  vsnprintf(text, sizeof text, fmt, args);
  va_end(args);

  Serial.println(text);

  LogEntry &e = logRing[logHead];
  e.ms = millis();
  memcpy(e.text, text, sizeof text);
  logHead = (uint8_t)((logHead + 1) % BLE_LOG_DEPTH);
  if (logCount < BLE_LOG_DEPTH) logCount++;

  notifyLogEntry(e);
}

}  // namespace BleService

#else  // ENABLE_BLE

namespace BleService {
void begin(const char *, const Handlers &) {}
void publishVitals(const BleVitals &) {}
bool publishSignals(uint32_t, const BleSignalSample *, uint8_t) { return false; }
void publishSpectrum(const float *, uint8_t, uint8_t, float, float) {}
uint8_t streamMask() { return 0; }
bool isConnected() { return false; }

void log(const char *fmt, ...) {
  char text[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(text, sizeof text, fmt, args);
  va_end(args);
  Serial.println(text);
}
}  // namespace BleService

#endif
