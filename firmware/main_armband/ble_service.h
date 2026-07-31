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

// Brings up the stack and starts advertising. Safe to call when ENABLE_BLE is 0, in
// which case every function here is a no-op and NimBLE is not linked in at all.
void begin(const char *deviceName);

// Publishes a vitals snapshot. Cheap and safe to call when nothing is connected --
// it returns immediately rather than packing a notification nobody will receive.
void publishVitals(const BleVitals &v);

bool isConnected();

}  // namespace BleService
