#pragma once
// Host-only test doubles. Never included by an Arduino firmware build.
#include <stdint.h>
#include <deque>
#include <sstream>
#include <string>

struct __FlashStringHelper;
#define F(text) reinterpret_cast<const __FlashStringHelper *>(text)
enum { LOW = 0, OUTPUT = 1, HEX = 16 };
extern uint64_t fakeUs;
extern int fakePwm[80];
inline uint32_t millis() { return uint32_t(fakeUs / 1000); }
inline uint32_t micros() { return uint32_t(fakeUs); }
inline void delay(uint32_t ms) { fakeUs += uint64_t(ms) * 1000; }
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t pin, int value) { fakePwm[pin] = value; }
inline void analogWrite(uint8_t pin, int value) { fakePwm[pin] = value; }

struct FakeSerial {
  std::string output;
  std::deque<char> input;
  void begin(int) {}
  int available() { return int(input.size()); }
  int read() { char c = input.front(); input.pop_front(); return c; }
  void print(const __FlashStringHelper *text) { output += reinterpret_cast<const char *>(text); }
  void print(char value) { output += value; }
  template<class T> void print(T value) { std::ostringstream s; s << +value; output += s.str(); }
  template<class T> void print(T value, int) { print(value); }
  template<class T> void println(T value) { print(value); output += '\n'; }
  template<class T> void println(T value, int format) { print(value, format); output += '\n'; }
};
extern FakeSerial Serial;
