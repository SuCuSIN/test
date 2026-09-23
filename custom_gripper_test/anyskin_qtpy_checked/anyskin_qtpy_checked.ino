// Checked AnySkin QT Py M0 reader. No EEPROM writes or motor control.
// Preserves five (temperature, X, Y, Z) float32 groups + CR LF.
#include <Wire.h>
#include <math.h>
#include "MLX90393.h"

#define Serial SERIAL_PORT_USBVIRTUAL

const uint8_t kCount = 5;
// Confirmed by this board's I2C scan. Slot order is address order, not yet
// verified physical magnetometer order for the pretrained slip model.
const uint8_t kAddresses[kCount] = {0x0C, 0x10, 0x11, 0x12, 0x13};
MLX90393 chips[kCount];
MLX90393::txyz readings[kCount];
bool seen[128];
bool ready[kCount];
uint8_t initStatus[kCount], startStatus[kCount], readStatus[kCount];
uint32_t failures[kCount], successes[kCount];
bool diagnostic = false;
uint32_t lastFrame = 0, lastDiagnostic = 0;
static_assert(sizeof(float) == 4, "Protocol needs float32");
static_assert(sizeof(MLX90393::txyz) == 16, "Unexpected packet padding");

void invalidate(uint8_t i) {
  readings[i] = MLX90393::txyz{NAN, NAN, NAN, NAN};
}

void initializeSensors() {
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    seen[address] = Wire.endTransmission() == 0;
  }
  for (uint8_t i = 0; i < kCount; ++i) {
    ready[i] = false;
    failures[i] = successes[i] = 0;
    initStatus[i] = startStatus[i] = readStatus[i] = MLX90393::STATUS_ERROR;
    invalidate(i);
    if (!seen[kAddresses[i]]) { continue; }
    initStatus[i] = chips[i].begin(kAddresses[i], -1, Wire);
    if (chips[i].hasError(initStatus[i])) { continue; }
    ready[i] = chips[i].isOK(initStatus[i]);
  }
  delay(10);
}

void printDiagnostics() {
  Serial.println("ANYSKIN_CHECKED_V3 single-shot 50Hz T,X,Y,Z; D=diagnostics S=stream R=reinitialize");
  Serial.print("I2C_ACK:");
  for (uint8_t a = 1; a < 127; ++a) {
    if (seen[a]) { Serial.print(" 0x"); Serial.print(a, HEX); }
  }
  Serial.println();
  for (uint8_t i = 0; i < kCount; ++i) {
    Serial.print("M"); Serial.print(i + 1);
    Serial.print(" address=0x"); Serial.print(kAddresses[i], HEX);
    Serial.print(" ack="); Serial.print(seen[kAddresses[i]]);
    Serial.print(" init=0x"); Serial.print(initStatus[i], HEX);
    Serial.print(" start=0x"); Serial.print(startStatus[i], HEX);
    Serial.print(" read=0x"); Serial.print(readStatus[i], HEX);
    Serial.print(" cause="); Serial.print(chips[i].lastReadError);
    Serial.print(" ready="); Serial.print(ready[i]);
    Serial.print(" good="); Serial.print(successes[i]);
    Serial.print(" errors="); Serial.print(failures[i]);
    Serial.print(" txyz="); Serial.print(readings[i].t, 3);
    Serial.print(','); Serial.print(readings[i].x, 3);
    Serial.print(','); Serial.print(readings[i].y, 3);
    Serial.print(','); Serial.println(readings[i].z, 3);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(10);
  initializeSensors();
}

void loop() {
  while (Serial.available()) {
    const char command = Serial.read();
    if (command == 'D') { diagnostic = true; lastDiagnostic = millis() - 1000; }
    if (command == 'S') { diagnostic = false; }
    // Faults stay latched until an explicit retry/reset; no silent recovery.
    if (command == 'R' && diagnostic) { initializeSensors(); }
  }
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastFrame) < 20) { return; }
  lastFrame = now;
  bool allGood = true;
  // Start all chips before waiting: no register reads/writes in burst mode,
  // and no dependence on previously programmed burst period or external trigger.
  for (uint8_t i = 0; i < kCount; ++i) {
    invalidate(i);
    if (!ready[i]) { allGood = false; continue; }
    startStatus[i] = chips[i].startMeasurement(0xF);
    if (chips[i].hasError(startStatus[i])) {
      ++failures[i];
      ready[i] = false;
      allGood = false;
    }
  }
  // Gain 7, filter 2, OSR/OSR2 0 are explicitly configured in begin().
  delay(10);
  for (uint8_t i = 0; i < kCount; ++i) {
    invalidate(i);
    if (!ready[i]) { allGood = false; continue; }
    // This API issues RM only; it does not start a burst.
    readStatus[i] = chips[i].readBurstData(readings[i]);
    if (chips[i].hasError(readStatus[i]) || !isfinite(readings[i].t) ||
        !isfinite(readings[i].x) || !isfinite(readings[i].y) || !isfinite(readings[i].z)) {
      ++failures[i];
      ready[i] = false;
      invalidate(i);
      allGood = false;
    } else {
      ++successes[i];
    }
  }
  if (!Serial) { return; }
  if (diagnostic) {
    if (static_cast<uint32_t>(now - lastDiagnostic) >= 1000) {
      lastDiagnostic = now;
      printDiagnostics();
    }
    return;
  }
  // Reject the WHOLE frame if any magnetometer is unavailable. This prevents
  // downstream grouping from accidentally hiding a missing magnetometer.
  const MLX90393::txyz invalid = {NAN, NAN, NAN, NAN};
  for (uint8_t i = 0; i < kCount; ++i) {
    const MLX90393::txyz &value = allGood ? readings[i] : invalid;
    Serial.write(reinterpret_cast<const uint8_t *>(&value), sizeof(value));
  }
  Serial.write('\r');
  Serial.write('\n');
}
