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
extern bool fakeAttached[80];
extern int fakeAttachFailurePin, fakeWriteFailurePin;
inline bool ledcAttachChannel(uint8_t pin,uint32_t freq,uint8_t bits,uint8_t channel) {
  if (pin==fakeAttachFailurePin || freq!=16000 || bits!=8 || channel>=8) return false;
  fakeAttached[pin]=true; fakePwm[pin]=0; return true;
}
inline bool ledcWrite(uint8_t pin,uint32_t duty) {
  if (!fakeAttached[pin] || pin==fakeWriteFailurePin || duty>255) return false;
  fakePwm[pin]=int(duty); return true;
}
inline bool ledcDetach(uint8_t pin) { fakeAttached[pin]=false; fakePwm[pin]=0; return true; }
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
  void println() { output+='\n'; }
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
