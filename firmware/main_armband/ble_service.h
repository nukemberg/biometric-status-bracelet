/*
 * BLE GATT service for the bracelet.
 *
 * Deliberately narrow: this unit knows how to advertise, how to pack a snapshot into
 * a notification, and nothing else. It never reaches into PulseTracker or GsrTracker.
 * The caller hands it a finished BleVitals and it transports that. Keeping the arrow
 * pointing this way is what lets libraries/BraceletDSP stay host-testable.
 *
 * The wire format lives in libraries/BraceletProtocol, shared with the CLI and web app
 * and asserted byte-for-byte by tools/ble_packet_test.sh.
 */

#pragma once

#include <stdint.h>

#include <ble_protocol.h>

// Compile-time escape hatch. NimBLE's controller task shares core 0 with the 500 Hz
// sampling loop, and the 20-sample boxcar nulls 50 Hz only while its window really is
// 40 ms. Being able to build the identical firmware with and without the radio is what
// makes that comparison meaningful rather than anecdotal.
#define ENABLE_BLE 1

namespace BleService {

// What a connected client is allowed to ask for. Passed in by the caller so this unit
// stays ignorant of display modes, trackers and LED internals -- it validates and
// dispatches, nothing more. Any handler may be null; the command is then ignored.
struct Handlers {
  void (*setMode)(uint8_t mode) = nullptr;
  void (*setBrightness)(uint8_t value) = nullptr;
  void (*recalibrateGsr)() = nullptr;
  void (*setStreams)(uint8_t mask) = nullptr;
  void (*resetBank)() = nullptr;
  void (*resetConfig)() = nullptr;
  // Config characteristic (-pmw). getConfig fills values[CFG_PARAM_COUNT] in
  // paramId order for a read; setConfigParam applies one live tunable and is
  // expected to persist it. Either may be null, in which case the corresponding
  // operation is a no-op (and a read returns the last packed value).
  void (*getConfig)(float *values) = nullptr;
  void (*setConfigParam)(uint8_t paramId, float value) = nullptr;
};

// Brings up the stack and starts advertising. Safe to call when ENABLE_BLE is 0, in
// which case every function here is a no-op and NimBLE is not linked in at all.
void begin(const char *deviceName, const Handlers &handlers);

// Optional streams. Both are no-ops unless the client has enabled them, so the
// caller can call them unconditionally, though checking streamMask() first avoids
// the cost of gathering the data at all.
// Returns false when the stack could not queue the notification (its TX buffers are
// full). The caller MUST NOT discard the samples in that case -- dropping them here
// would silently punch holes in a stream whose only purpose is offline analysis.
bool publishSignals(uint32_t firstTimestampMs, const BleSignalSample *samples,
                    uint8_t count);
void publishSpectrum(const float *binPower, uint8_t bins, uint8_t peakBin,
                     float bpmLo, float bpmHi);

// Which optional streams a client has switched on. The caller checks this before
// doing the work of collecting stream data, so an unsubscribed connection costs
// nothing beyond vitals.
uint8_t streamMask();

// Publishes a vitals snapshot. Cheap and safe to call when nothing is connected --
// it returns immediately rather than packing a notification nobody will receive.
void publishVitals(const BleVitals &v);

bool isConnected();

// Error/warning log, ring-buffered (BLE_LOG_DEPTH entries) and mirrored to Serial.
// Callers pass printf-style format + args, same as Serial.printf. Always safe to call,
// BLE enabled or not, connected or not -- Serial output happens either way, and the
// ring buffer plus notify only engage when the radio is actually up. A connected
// client replays the buffered history by writing CMD_DUMP_LOG to the Control
// characteristic; new entries notify live from then on.
void log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

}  // namespace BleService
