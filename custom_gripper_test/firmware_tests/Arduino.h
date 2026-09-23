#pragma once
#include <stdint.h>
#include <stddef.h>
#define INPUT 0
inline void pinMode(int, int) {}
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned long) {}
inline int digitalRead(int) { return 1; }
inline uint32_t millis() { static uint32_t value = 0; return ++value; }
