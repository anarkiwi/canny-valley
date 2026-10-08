#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include "../sled_core.h"

namespace {

struct Motor {
  long pos = 0, target = 0, phys = 0, unit = 16, decel = 3;
  float maxSpeed = 0, accel = 0;
  bool stepped = false;
  long currentPosition() const { return pos; }
  long targetPosition() const { return target; }
  long distanceToGo() const { return target - pos; }
  bool isRunning() const { return distanceToGo() != 0; }
  float speed() const { return stepped && isRunning() ? 1.0f : 0.0f; }
  void moveTo(long t) { target = t; }
  void setCurrentPosition(long p) {
    pos = target = p;
    stepped = false;
  }
  void stop() {
    if (speed() != 0)
      target = pos + std::clamp(distanceToGo(), -decel, decel);
  }
  void setMaxSpeed(float s) { maxSpeed = s; }
  void setAcceleration(float a) { accel = a; }
  void run() {
    long dir = (distanceToGo() > 0) - (distanceToGo() < 0);
    stepped = dir != 0;
    pos += dir;
    phys += dir * unit;
  }
};

constexpr long kFull = 16;

struct Rig {
  Motor motor;
  long trip = -300 * kFull, hysteresis = 2 * kFull, trips = 1 << 20;
  bool pressed = false, end = false;
  uint8_t bits = 0xff;
  std::string out;
  sled::Controller<Motor, Rig> ctl{motor, *this};

  Rig() { ctl.begin(); }
  bool homeSwitch() {
    if (!pressed && motor.phys <= trip && trips > 0) {
      pressed = true;
      --trips;
    } else if (pressed && motor.phys > trip + hysteresis) {
      pressed = false;
    }
    return pressed;
  }
  bool endSwitch() { return end; }
  void setMicrostep(uint8_t b) {
    bits = b;
    motor.unit = b == 7 ? 1 : kFull >> b;
  }
  void print(const char *s) { out += s; }
  void print(long v) { out += std::to_string(v); }
  void println() { out += "\n"; }

  void feed(const std::string &line) {
    for (char c : line)
      ctl.feed(c);
  }
  void settle(long polls = 100000) {
    for (; polls > 0 && ctl.phase() != ctl.kIdle; --polls)
      ctl.poll();
    ctl.poll();
  }
  std::string send(const std::string &line) {
    out.clear();
    feed(line + "\r\n");
    settle();
    return out;
  }
  void homed() {
    REQUIRE(send("home") == "ok 0\n");
    REQUIRE(ctl.homed());
  }
};

} // namespace

TEST_CASE("parseLong") {
  long v = 0;
  CHECK((sled::parseLong("123", v) && v == 123));
  CHECK((sled::parseLong("-7", v) && v == -7));
  CHECK((sled::parseLong("+7", v) && v == 7));
  CHECK((sled::parseLong("999999999", v) && v == 999999999));
  for (const char *bad : {"", "-", "1x", "x1", " 1", "1 ", "1234567890"})
    CHECK_FALSE(sled::parseLong(bad, v));
}

TEST_CASE("scaleRound and microstepBits") {
  CHECK(sled::scaleRound(3, 1, 8) == 0);
  CHECK(sled::scaleRound(4, 1, 8) == 1);
  CHECK(sled::scaleRound(-4, 1, 8) == -1);
  CHECK(sled::scaleRound(-3, 1, 8) == 0);
  CHECK(sled::scaleRound(-5, 16, 1) == -80);
  const long ms[] = {1, 2, 4, 8, 16};
  const uint8_t bits[] = {0, 1, 2, 3, 7};
  for (int i = 0; i < 5; ++i)
    CHECK(sled::microstepBits(ms[i]) == bits[i]);
  CHECK(sled::microstepBits(32) == 0xff);
  CHECK(sled::kTravelFull == 1221);
}

TEST_CASE("protocol transcript") {
  Rig rig;
  std::ifstream file(SLED_TRANSCRIPT);
  REQUIRE(file);
  std::string line, sent, expect;
  int checked = 0;
  auto flush = [&] {
    if (!sent.empty()) {
      INFO("sent: " << sent);
      CHECK(rig.send(sent) == expect);
      ++checked;
    }
  };
  while (std::getline(file, line)) {
    if (line.rfind("> ", 0) == 0) {
      flush();
      sent = line.substr(2);
      expect.clear();
    } else if (line.rfind("< ", 0) == 0) {
      expect += line.substr(2) + "\n";
    }
  }
  flush();
  CHECK(checked > 20);
  CHECK(rig.bits == 0);
}

TEST_CASE("homing sequence parks 10 full steps out from the trip") {
  for (long ms : {1L, 4L, 16L}) {
    Rig rig;
    REQUIRE(rig.send("microstep " + std::to_string(ms)) ==
            "microstep " + std::to_string(ms) + "\n");
    rig.homed();
    CHECK(rig.motor.phys == rig.trip + sled::kHomeOffsetFull * kFull);
    CHECK(rig.motor.maxSpeed == sled::kDefaultSpeed);
    CHECK(rig.send("status?").find("homed=1 moving=0 homing=0 pos=0") !=
          std::string::npos);
  }
}

TEST_CASE("homing from on the switch backs off first") {
  Rig rig;
  rig.trip = 0;
  rig.homed();
  CHECK(rig.motor.phys == 10 * kFull);
  CHECK(rig.trips == (1 << 20) - 3);
}

TEST_CASE("homing timeouts") {
  SUBCASE("no switch") {
    Rig rig;
    rig.trips = 0;
    CHECK(rig.send("home") == "err home timeout\n");
    CHECK(rig.motor.phys == -(sled::kTravelFull + sled::kMarginFull) * kFull);
  }
  SUBCASE("switch stuck") {
    Rig rig;
    rig.trip = 1 << 20;
    CHECK(rig.send("home") == "err home timeout\n");
    CHECK(rig.motor.phys == sled::kMarginFull * kFull);
  }
  SUBCASE("no second trip") {
    Rig rig;
    rig.trips = 1;
    CHECK(rig.send("home") == "err home timeout\n");
  }
  SUBCASE("end switch aborts") {
    Rig rig;
    rig.end = true;
    CHECK(rig.send("home") == "err limit\n");
  }
}

TEST_CASE("limit switches stop a move and clear homed") {
  for (int which = 0; which < 2; ++which) {
    Rig rig;
    rig.homed();
    rig.feed("move 600\n");
    for (int i = 0; i < 50; ++i)
      rig.ctl.poll();
    which ? (void)(rig.end = true)
          : (void)(rig.trip = rig.motor.phys + 2 * kFull);
    rig.out.clear();
    rig.settle();
    CHECK(rig.out == "err limit\n");
    CHECK_FALSE(rig.ctl.homed());
    CHECK(rig.motor.distanceToGo() == 0);
    CHECK(rig.send("move 10") == "err not homed\n");
  }
}

TEST_CASE("switch pressed while homed and idle") {
  Rig rig;
  rig.homed();
  rig.trip = rig.motor.phys;
  rig.out.clear();
  rig.settle();
  CHECK(rig.out == "err limit\n");
  rig.out.clear();
  rig.settle();
  CHECK(rig.out.empty());
}

TEST_CASE("commands during motion") {
  Rig rig;
  rig.homed();
  rig.out.clear();
  rig.feed("move 600\n");
  for (int i = 0; i < 100; ++i)
    rig.ctl.poll();
  rig.feed("move 10\nhome\nmicrostep 2\npos?\nspeed 300\n");
  CHECK(rig.out == "err busy\nerr busy\nerr busy\npos 100\nspeed 300\n");
  CHECK(rig.motor.maxSpeed == 300);
  rig.out.clear();
  rig.feed("stop\n");
  rig.settle();
  CHECK(rig.out == "ok 103\n");
  CHECK(rig.ctl.homed());
}

TEST_CASE("stop before the first step and during homing") {
  Rig rig;
  rig.homed();
  CHECK(rig.send("move 600\nstop") == "ok 0\n");
  rig.out.clear();
  rig.feed("home\n");
  for (int i = 0; i < 5; ++i)
    rig.ctl.poll();
  rig.feed("speed 800\nstatus?\n");
  CHECK(rig.motor.maxSpeed == sled::kDefaultSpeed);
  rig.out.clear();
  rig.feed("stop\n");
  rig.settle();
  CHECK(rig.out == "ok -8\n");
  CHECK_FALSE(rig.ctl.homed());
  CHECK(rig.motor.maxSpeed == 800);
}

TEST_CASE("line framing") {
  Rig rig;
  CHECK(rig.send("").empty());
  CHECK(rig.send(std::string(sled::kLineMax, 'x')) == "err unknown\n");
  CHECK(rig.send(std::string(sled::kLineMax + 5, 'x')) == "err unknown\n");
  CHECK(rig.send("pos?") == "pos 0\n");
  rig.out.clear();
  rig.feed("p\ro\rs?\n");
  CHECK(rig.out == "pos 0\n");
}
