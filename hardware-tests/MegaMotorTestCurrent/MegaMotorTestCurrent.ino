// MOTOR-ONLY TEST: Mega 2560 + two MDD3A drivers + four motors.
// Current pin map: FL 6/7, FR 4/5, BL 11/10, BR 2/3.
// Serial Monitor: 115200 baud, Newline. Start with wheels lifted.
// No IR or MPU code is used. Motors are stopped after every 800 ms pulse.
#if !defined(__AVR_ATmega2560__)
#error Select Arduino Mega or Mega 2560 in Tools > Board.
#endif

// FL = front left, FR = front right, BL = back left, BR = back right.
// Each MDD3A motor channel has two control inputs, named 1 and 2 here.
const uint8_t FL1 = 6;
const uint8_t FL2 = 7;
const uint8_t FR1 = 4;
const uint8_t FR2 = 5;
const uint8_t BL1 = 11;
const uint8_t BL2 = 10;
const uint8_t BR1 = 2;
const uint8_t BR2 = 3;

// Change only a reversed motor's sign to -1.
const int8_t FL_SIGN = 1;
const int8_t FR_SIGN = 1;
const int8_t BL_SIGN = -1;
const int8_t BR_SIGN = 1;
const uint8_t MOTOR_PINS[8] = {FL1, FL2, FR1, FR2, BL1, BL2, BR1, BR2};
const int TEST_PWM = 80;
const unsigned long PULSE_MS = 800;

bool running = false;
unsigned long pulseStarted = 0;
char commandLine[8] = {};
uint8_t commandLength = 0;
bool discardLine = false;

void motor(uint8_t input1, uint8_t input2, int requested) {
  int pwm = constrain(requested, -TEST_PWM, TEST_PWM);
  analogWrite(input1, 0);
  analogWrite(input2, 0);
  if (pwm > 0) analogWrite(input1, pwm);
  else if (pwm < 0) analogWrite(input2, -pwm);
}

void setMotors(int fl, int fr, int bl, int br) {
  motor(FL1, FL2, fl * FL_SIGN);
  motor(FR1, FR2, fr * FR_SIGN);
  motor(BL1, BL2, bl * BL_SIGN);
  motor(BR1, BR2, br * BR_SIGN);
}

void stopAll() {
  setMotors(0, 0, 0, 0);
  running = false;
}

void printHelp() {
  Serial.println(F("1=FL+  2=FR+  3=BL+  4=BR+"));
  Serial.println(F("5=FL-  6=FR-  7=BL-  8=BR-"));
  Serial.println(F("w=forward  b=backward  a=left  d=right"));
  Serial.println(F("q=turn left  e=turn right"));
  Serial.println(F("s/x/0/space=stop  ?=help"));
  Serial.println(F("One command per line. Each movement lasts 800ms at PWM 80."));
}

void executeCommand(char command) {
  stopAll();
  if (command == '?') { printHelp(); return; }

  int fl = 0, fr = 0, bl = 0, br = 0;
  if (command >= '1' && command <= '8') {
    switch (command) {
      case '1': fl = TEST_PWM; break;
      case '2': fr = TEST_PWM; break;
      case '3': bl = TEST_PWM; break;
      case '4': br = TEST_PWM; break;
      case '5': fl = -TEST_PWM; break;
      case '6': fr = -TEST_PWM; break;
      case '7': bl = -TEST_PWM; break;
      case '8': br = -TEST_PWM; break;
    }
  } else {
    int forward = 0, strafe = 0, rotation = 0;
    switch (command) {
      case 'w': forward = TEST_PWM; break;
      case 'b': forward = -TEST_PWM; break;
      case 'a': strafe = -TEST_PWM; break;
      case 'd': strafe = TEST_PWM; break;
      case 'q': rotation = -TEST_PWM; break;
      case 'e': rotation = TEST_PWM; break;
      default: Serial.println(F("Unknown command; motors remain stopped.")); return;
    }
    // Same four-wheel mixer as your original FastRoboCup program.
    fl = forward + strafe + rotation;
    fr = forward - strafe - rotation;
    bl = forward - strafe + rotation;
    br = forward + strafe - rotation;
  }

  pulseStarted = millis();
  setMotors(fl, fr, bl, br);
  running = true;
  Serial.print(F("FL/FR/BL/BR: "));
  Serial.print(fl); Serial.print('/');
  Serial.print(fr); Serial.print('/');
  Serial.print(bl); Serial.print('/');
  Serial.println(br);
}

void handleSerial() {
  // Parse a whole line so a word cannot accidentally execute several tests.
  // Stop characters take effect without waiting for the newline.
  for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
    char c = Serial.read();
    if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    if (c == '\r' || c == '\n') {
      if (!discardLine) {
        if (commandLength == 1) executeCommand(commandLine[0]);
        else if (commandLength > 1) {
          stopAll(); Serial.println(F("Send one command per line."));
        }
      }
      commandLength = 0;
      discardLine = false;
      continue;
    }
    if (c == 's' || c == 'x' || c == '0' || c == ' ') {
      stopAll();
      if (!discardLine) Serial.println(F("STOPPED"));
      commandLength = 0;
      discardLine = true;
      continue;
    }
    if (discardLine) continue;
    if (commandLength < sizeof(commandLine) - 1) commandLine[commandLength++] = c;
    else {
      stopAll(); commandLength = 0; discardLine = true;
      Serial.println(F("Command too long; motors stopped."));
    }
  }
}

void stopIfExpired() {
  if (running && millis() - pulseStarted >= PULSE_MS) {
    stopAll();
    Serial.println(F("Pulse complete; STOPPED."));
  }
}

void setup() {
  for (uint8_t i = 0; i < 8; ++i) {
    digitalWrite(MOTOR_PINS[i], LOW);
    pinMode(MOTOR_PINS[i], OUTPUT);
  }
  stopAll();
  Serial.begin(115200);
  Serial.println(F("MEGA MOTOR-ONLY TEST. Motors start STOPPED."));
  printHelp();
}

void loop() {
  stopIfExpired();
  handleSerial();
  stopIfExpired();
}
