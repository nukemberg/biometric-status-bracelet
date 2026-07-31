#include "ble_service.h"

#if ENABLE_BLE

#include <Arduino.h>
#include <NimBLEDevice.h>

#include <dsp.h>   // N_BINS and BPM range, reported in the Info characteristic

namespace {

NimBLEServer *server = nullptr;
NimBLECharacteristic *vitalsChr = nullptr;
NimBLECharacteristic *infoChr = nullptr;
bool connected = false;

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

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &info) override {
    connected = true;
    Serial.print(F("[BLE] connected, peer "));
    Serial.println(info.getAddress().toString().c_str());
  }

  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &info, int reason) override {
    connected = false;
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

void begin(const char *deviceName) {
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

  infoChr = svc->createCharacteristic(BLE_CHR_INFO, NIMBLE_PROPERTY::READ);
  infoChr->setValue(buildInfoString().c_str());

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

bool isConnected() { return connected; }

}  // namespace BleService

#else  // ENABLE_BLE

namespace BleService {
void begin(const char *) {}
void publishVitals(const BleVitals &) {}
bool isConnected() { return false; }
}  // namespace BleService

#endif
