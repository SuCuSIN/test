#pragma once
#include "Arduino.h"
#include <vector>
class TwoWire {
public:
  uint8_t nack = 0;
  bool shortRead = false;
  std::vector<uint8_t> response{0};
  size_t cursor = 0;
  void beginTransmission(uint8_t) {}
  size_t write(uint8_t) { return 1; }
  uint8_t endTransmission() { return nack; }
  uint8_t requestFrom(uint8_t, uint8_t count) { cursor = 0; return shortRead ? 0 : count; }
  int available() { return cursor < response.size(); }
  int read() { return available() ? response[cursor++] : -1; }
};
extern TwoWire Wire;
