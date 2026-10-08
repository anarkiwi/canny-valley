// Rail sled controller: Arduino Leonardo, A4988 driver, home switch on pin 2
// (docs/sled.md).
#include <AccelStepper.h>

#include "sled_core.h"

namespace {

constexpr uint8_t kStepPin = 10, kDirPin = 11, kSleepPin = 9, kResetPin = 8;
constexpr uint8_t kEnPin = 4, kHomePin = 2, kEndPin = 3;
constexpr uint8_t kMsPins[] = {5, 6, 7};
constexpr uint8_t kOutPins[] = {kStepPin, kDirPin,    kSleepPin,  kResetPin,
                                kEnPin,   kMsPins[0], kMsPins[1], kMsPins[2]};

struct Io {
  bool homeSwitch() { return digitalRead(kHomePin) == LOW; }
  bool endSwitch() { return digitalRead(kEndPin) == LOW; }
  void setMicrostep(uint8_t bits) {
    for (uint8_t i = 0; i < 3; ++i)
      digitalWrite(kMsPins[i], (bits >> i) & 1 ? HIGH : LOW);
  }
  void print(const char *s) { Serial.print(s); }
  void print(long v) { Serial.print(v); }
  void println() { Serial.println(); }
};

AccelStepper motor(AccelStepper::DRIVER, kStepPin, kDirPin);
Io io;
sled::Controller<AccelStepper, Io> controller(motor, io);
bool connected = false;

} // namespace

void setup() {
  for (uint8_t pin : kOutPins)
    pinMode(pin, OUTPUT);
  digitalWrite(kEnPin, LOW);
  digitalWrite(kSleepPin, HIGH);
  digitalWrite(kResetPin, HIGH);
  pinMode(kHomePin, INPUT_PULLUP);
  pinMode(kEndPin, INPUT_PULLUP);
  Serial.begin(9600);
  controller.begin();
}

void loop() {
  controller.poll();
  while (Serial.available() > 0)
    controller.feed(static_cast<char>(Serial.read()));
  bool dtr = Serial.dtr();
  if (dtr && !connected)
    controller.banner();
  connected = dtr;
}
