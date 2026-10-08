// Rail sled controller logic, independent of the Arduino runtime
// (docs/sled.md). Motor is AccelStepper or any class with its interface; Io
// reads the switches, drives the microstep pins and prints replies.
#pragma once

#include <stdint.h>
#include <string.h>

namespace sled {

constexpr const char *kVersion = "1";
constexpr long kStepNm = 777544;
constexpr long kTravelFull = 950000000L / kStepNm;
constexpr long kHomeOffsetFull = 10;
constexpr long kMarginFull = 100;
constexpr long kDefaultSpeed = 500;
constexpr long kDefaultAccel = 1000;
constexpr long kMaxSpeed = 4000;
constexpr long kMaxAccel = 100000;
constexpr uint8_t kLineMax = 24;

inline bool parseLong(const char *s, long &v) {
  bool neg = *s == '-';
  if (neg || *s == '+')
    ++s;
  uint8_t n = 0;
  v = 0;
  for (; *s >= '0' && *s <= '9'; ++s, ++n)
    v = v * 10 + (*s - '0');
  if (neg)
    v = -v;
  return n > 0 && n < 10 && *s == '\0';
}

inline long scaleRound(long v, long num, long den) {
  long n = v * num;
  return n >= 0 ? (n + den / 2) / den : -((-n + den / 2) / den);
}

inline uint8_t microstepBits(long ms) {
  switch (ms) {
  case 1:
    return 0;
  case 2:
    return 1;
  case 4:
    return 2;
  case 8:
    return 3;
  case 16:
    return 7;
  default:
    return 0xff;
  }
}

template <class Motor, class Io> class Controller {
public:
  enum Phase : uint8_t {
    kIdle,
    kMove,
    kBackoff,
    kClear,
    kApproach,
    kCreep,
    kPark
  };

  Controller(Motor &motor, Io &io) : motor_(motor), io_(io) {}

  void begin() {
    io_.setMicrostep(microstepBits(ms_));
    motor_.setMaxSpeed(speed_);
    motor_.setAcceleration(accel_);
  }

  void banner() {
    io_.print("id qmrdk-sled ");
    io_.print(kVersion);
    io_.print(" ");
    io_.print(kStepNm);
    io_.print(" ");
    io_.print(ms_);
    io_.println();
  }

  void feed(char c) {
    if (c == '\r')
      return;
    if (c != '\n') {
      if (len_ < kLineMax)
        line_[len_] = c;
      len_ += len_ <= kLineMax;
      return;
    }
    bool fits = len_ <= kLineMax;
    line_[fits ? len_ : 0] = '\0';
    uint8_t n = len_;
    len_ = 0;
    if (n)
      fits ? command(line_) : reply("err unknown");
  }

  void poll() {
    motor_.run();
    bool home = io_.homeSwitch(), end = io_.endSwitch();
    if ((end || (home && !homing())) && (homed_ || phase_ != kIdle)) {
      halt();
      homed_ = false;
      finish("err limit", false);
      return;
    }
    bool done = !motor_.isRunning();
    switch (phase_) {
    case kIdle:
      break;
    case kMove:
      if (done)
        finish("ok", true);
      break;
    case kBackoff:
      if (!home) {
        motor_.moveTo(motor_.currentPosition() + kHomeOffsetFull * ms_);
        phase_ = kClear;
      } else if (done) {
        finish("err home timeout", false);
      }
      break;
    case kClear:
      if (done)
        next_ == kApproach ? approach()
                           : seek(kCreep, speed_ / 16, -1, 2 * kMarginFull);
      break;
    case kApproach:
    case kCreep:
      if (home) {
        halt();
        if (phase_ == kApproach) {
          next_ = kCreep;
          seek(kBackoff, speed_ / 2, 1, kMarginFull);
        } else {
          motor_.setCurrentPosition(-kHomeOffsetFull * ms_);
          motor_.setMaxSpeed(speed_);
          motor_.moveTo(0);
          phase_ = kPark;
        }
      } else if (done) {
        finish("err home timeout", false);
      }
      break;
    case kPark:
      if (done) {
        homed_ = true;
        finish("ok", true);
      }
      break;
    }
  }

  bool homed() const { return homed_; }
  Phase phase() const { return phase_; }
  long maxSteps() const { return kTravelFull * ms_; }

private:
  bool homing() const { return phase_ >= kBackoff; }

  void reply(const char *s) {
    io_.print(s);
    io_.println();
  }

  void reply(const char *key, long v) {
    io_.print(key);
    io_.print(" ");
    io_.print(v);
    io_.println();
  }

  void field(const char *key, long v) {
    io_.print(" ");
    io_.print(key);
    io_.print("=");
    io_.print(v);
  }

  void halt() { motor_.setCurrentPosition(motor_.currentPosition()); }

  void finish(const char *msg, bool withPos) {
    phase_ = kIdle;
    motor_.setMaxSpeed(speed_);
    withPos ? reply(msg, motor_.currentPosition()) : reply(msg);
  }

  void seek(Phase phase, long speed, int dir, long limitFull) {
    motor_.setMaxSpeed(speed > 0 ? speed : 1);
    motor_.moveTo(motor_.currentPosition() + dir * limitFull * ms_);
    phase_ = phase;
  }

  void approach() {
    next_ = kApproach;
    seek(kApproach, speed_, -1, kTravelFull + kMarginFull);
  }

  void status() {
    io_.print("status");
    field("homed", homed_);
    field("moving", phase_ != kIdle);
    field("homing", homing());
    field("pos", motor_.currentPosition());
    field("target", motor_.targetPosition());
    field("home", io_.homeSwitch());
    field("end", io_.endSwitch());
    field("microstep", ms_);
    field("speed", speed_);
    field("accel", accel_);
    field("max", maxSteps());
    io_.println();
  }

  void command(char *line) {
    char *arg = strchr(line, ' ');
    if (arg)
      *arg++ = '\0';
    long v = 0;
    if (arg && !parseLong(arg, v))
      return reply("err unknown");
    if (!arg) {
      if (!strcmp(line, "id?"))
        return banner();
      if (!strcmp(line, "pos?"))
        return reply("pos", motor_.currentPosition());
      if (!strcmp(line, "status?"))
        return status();
      if (!strcmp(line, "stop")) {
        if (phase_ == kIdle)
          return reply("ok", motor_.currentPosition());
        motor_.setMaxSpeed(speed_);
        motor_.stop();
        if (motor_.speed() == 0)
          halt();
        phase_ = kMove;
        return;
      }
      if (!strcmp(line, "home")) {
        if (phase_ != kIdle)
          return reply("err busy");
        homed_ = false;
        if (!io_.homeSwitch())
          return approach();
        next_ = kApproach;
        return seek(kBackoff, speed_, 1, kMarginFull);
      }
      return reply("err unknown");
    }
    if (!strcmp(line, "speed") || !strcmp(line, "accel")) {
      bool isSpeed = line[0] == 's';
      if (v < 1 || v > (isSpeed ? kMaxSpeed : kMaxAccel))
        return reply("err range");
      if (isSpeed) {
        speed_ = v;
        if (!homing())
          motor_.setMaxSpeed(v);
      } else {
        accel_ = v;
        motor_.setAcceleration(v);
      }
      return reply(line, v);
    }
    if (!strcmp(line, "move")) {
      if (phase_ != kIdle)
        return reply("err busy");
      if (!homed_)
        return reply("err not homed");
      if (v < 0 || v > maxSteps())
        return reply("err range");
      motor_.moveTo(v);
      phase_ = kMove;
      return;
    }
    if (!strcmp(line, "microstep")) {
      uint8_t bits = microstepBits(v);
      if (bits == 0xff)
        return reply("err range");
      if (phase_ != kIdle)
        return reply("err busy");
      motor_.setCurrentPosition(scaleRound(motor_.currentPosition(), v, ms_));
      ms_ = v;
      io_.setMicrostep(bits);
      return reply(line, v);
    }
    reply("err unknown");
  }

  Motor &motor_;
  Io &io_;
  Phase phase_ = kIdle, next_ = kApproach;
  bool homed_ = false;
  long ms_ = 1, speed_ = kDefaultSpeed, accel_ = kDefaultAccel;
  char line_[kLineMax + 1];
  uint8_t len_ = 0;
};

} // namespace sled
