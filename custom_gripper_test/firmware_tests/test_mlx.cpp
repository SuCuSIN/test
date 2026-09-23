#include <cassert>
#include <cmath>
#include "MLX90393.h"

TwoWire Wire;

void invalid(const MLX90393::txyz &v) {
  assert(std::isnan(v.t) && std::isnan(v.x) && std::isnan(v.y) && std::isnan(v.z));
}

int main() {
  MLX90393 chip;
  // Set the bus pointer without pretending initialization succeeded.
  Wire.nack = 2;
  assert(chip.hasError(chip.begin(0x0D, -1, Wire)));
  MLX90393::txyz value{1, 2, 3, 4};
  assert(chip.hasError(chip.readBurstData(value)));
  invalid(value);
  Wire.nack = 0;
  Wire.response = {0};
  // Fill conversion cache with acknowledged writes.
  assert(chip.isOK(chip.writeRegister(0, 0x7C)));
  assert(chip.isOK(chip.writeRegister(1, 0)));
  assert(chip.isOK(chip.writeRegister(2, 0)));
  Wire.response = {0x80, 0xB4, 0xA4, 0, 10, 0, 20, 0, 30};
  assert(chip.isOK(chip.readBurstData(value)));
  assert(std::fabs(value.x - 1.5f) < 0.001f);
  assert(std::fabs(value.y - 3.0f) < 0.001f);
  assert(std::fabs(value.z - 7.26f) < 0.001f);
  Wire.nack = 2;
  assert(chip.hasError(chip.readBurstData(value)));
  invalid(value);
  Wire.nack = 0;
  Wire.shortRead = true;
  assert(chip.hasError(chip.readBurstData(value)));
  invalid(value);
  Wire.shortRead = false;
  Wire.response[0] = 0x90;
  assert(chip.hasError(chip.readBurstData(value)));
  invalid(value);
  Wire.response = {0x80, 0}; // Claimed count, truncated actual response.
  assert(chip.hasError(chip.readBurstData(value)));
  invalid(value);
  MLX90393::txyzRaw raw{1, 2, 3, 4};
  assert(chip.hasError(chip.readRawBurstData(raw)));
  assert(raw.t == 0 && raw.x == 0 && raw.y == 0 && raw.z == 0);
  // A register error must not contaminate the conversion cache.
  Wire.response = {0x10, 0, 0};
  uint16_t reg = 123;
  assert(chip.hasError(chip.readRegister(0, reg)));
  assert(reg == 123);
}
