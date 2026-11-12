// Mega 2560 motor test. Serial Monitor: 115200 baud, New Line.
// Keep motor power off during upload or reset. The bootloader can blink D13.
#include <EEPROM.h>

#if !defined(__AVR_ATmega2560__)
#error Select Arduino Mega or Mega 2560 with the ATmega2560 processor.
#endif

// FL, FR, BL, BR. Each row is input A, input B.
constexpr uint8_t motorPins[][2] = {{9, 7}, {5, 6}, {11, 13}, {10, 12}};
const char *const motorNames[] = {"FL", "FR", "BL", "BR"};
constexpr uint8_t testPwm = 100;
constexpr unsigned long pulseMs = 500;

uint8_t reverseMask = 0x04;  // Same default as the Mega soccer sketch.
bool pulseActive = false;
unsigned long pulseStartedMs = 0;
char pendingCommand = 0;
bool discardLine = false;

void stopMotors() {
  for (const auto &pins : motorPins) {
    analogWrite(pins[0], 0);
    analogWrite(pins[1], 0);
  }
  pulseActive = false;
}

void printDirections() {
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(motorNames[i]);
    Serial.println(reverseMask & (1U << i) ? F(" reversed") : F(" normal"));
  }
}

void executeCommand(char command) {
  if (pulseActive) return;
  if (command == 'p') {
    printDirections();
    return;
  }
  int wheel[4] = {};
  if (command >= '1' && command <= '8') {
    const uint8_t motor = (command - '1') % 4;
    wheel[motor] = command <= '4' ? int(testPwm) : -int(testPwm);
  } else {
    int forward = 0;
    int strafe = 0;
    int turn = 0;
    switch (command) {
      case 'w': forward = testPwm; break;
      case 'b': forward = -int(testPwm); break;
      case 'a': strafe = -int(testPwm); break;
      case 'd': strafe = testPwm; break;
      case 'q': turn = -int(testPwm); break;
      case 'e': turn = testPwm; break;
      default:
        Serial.println(F("Use 1-8, w/b/a/d/q/e, s, or p."));
        return;
    }
    wheel[0] = forward + strafe + turn;
    wheel[1] = forward - strafe - turn;
    wheel[2] = forward - strafe + turn;
    wheel[3] = forward + strafe - turn;
  }

  stopMotors();
  pulseStartedMs = millis();
  for (uint8_t i = 0; i < 4; ++i) {
    const int pwm = (reverseMask & (1U << i)) ? -wheel[i] : wheel[i];
    if (pwm > 0) analogWrite(motorPins[i][0], pwm);
    if (pwm < 0) analogWrite(motorPins[i][1], -pwm);
  }
  pulseActive = true;
  Serial.print(F("Testing command "));
  Serial.println(command);
}

void readCommands() {
  for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
    char input = char(Serial.read());
    if (input == '\r' || input == '\n') {
      if (!discardLine && pendingCommand) executeCommand(pendingCommand);
      pendingCommand = 0;
      discardLine = false;
    } else if (input == 's' || input == 'S') {
      stopMotors();
      if (!discardLine) Serial.println(F("Stopped."));
      discardLine = true;
    } else if (!discardLine) {
      if (pendingCommand) {
        stopMotors();
        discardLine = true;
        Serial.println(F("Send one command per line."));
      } else {
        pendingCommand = input;
      }
    }
  }
}

void setup() {
  for (const auto &pins : motorPins) {
    for (uint8_t pin : pins) {
      digitalWrite(pin, LOW);
      pinMode(pin, OUTPUT);
    }
  }
  stopMotors();
  if (EEPROM.read(0) == 0xA5) {
    const uint8_t saved = EEPROM.read(1);
    if (saved <= 0x0F) reverseMask = saved;
  }
  Serial.begin(115200);
  Serial.println(F("Mega motor test. Start with wheels lifted."));
  Serial.println(F("1/2/3/4 = FL/FR/BL/BR forward. 5/6/7/8 = reverse."));
  Serial.println(F("w/b = forward/back. a/d = strafe left/right. q/e = turn left/right."));
  Serial.println(F("500 ms at PWM 100. s stops, p prints motor directions."));
}

void loop() {
  if (pulseActive && millis() - pulseStartedMs >= pulseMs) {
    stopMotors();
    Serial.println(F("Pulse complete."));
  }
  readCommands();
}
