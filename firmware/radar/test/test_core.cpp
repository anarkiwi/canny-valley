#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cstring>
#include <map>
#include <string>
#include <vector>

extern "C" {
#include "core.h"
#include "fmt.h"
#include "hal.h"
#include "synth.h"
}

namespace {

struct Frame {
  uint8_t reg;
  uint32_t data;
};

struct Board {
  std::vector<Frame> writes;
  std::map<uint8_t, uint32_t> regs;
  std::vector<uint32_t> triggers;
  std::vector<uint8_t> eeprom = std::vector<uint8_t>(0x200, 0xFF);
  uint32_t lock = 2, ms = 0, sample = 0, trig_reg = 0, resets = 0;
  uint16_t temp = 800;
  uint8_t good_gpo = 9, spi2 = 0;
  Frame pending{};
  bool rf = true, clock = false, eeprom_ok = true, polled = false;

  static uint16_t code(uint32_t i) { return (uint16_t)(i * 7919u + 13u); }

  void land(Frame f, uint32_t at) {
    regs[f.reg] = f.data;
    if (f.reg == 0x0E) {
      if (f.data == 1 && trig_reg == 0)
        triggers.push_back(at);
      trig_reg = f.data;
    }
  }

  void isr() {
    uint16_t w = sample ? code(sample - 1) : 0xDEAD;
    if (spi2 == 1) {
      land(pending, sample);
      spi2 = 2;
    } else if (spi2 == 2) {
      spi2 = 0;
    }
    uint8_t a = radar_isr_step(w);
    if (a != SCHED_NONE) {
      REQUIRE(spi2 == 0);
      CHECK(SCHED_B1 == 0x1C);
      pending = {0x0E, SCHED_B4(a) == 0x02 ? 1u : 0u};
      spi2 = 1;
    }
    sample++;
  }

  uint32_t per_ms() const { return 20000u / regs.at(0x02); }

  void tick() {
    if (clock)
      for (uint32_t i = 0; i < per_ms(); i++)
        isr();
    ms++;
    polled = false;
  }
};

Board *B;

std::string text(const uint8_t *p, size_t n) {
  return std::string((const char *)p, n);
}

void run(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    B->tick();
    radar_poll();
  }
}

void send(const std::string &msg) {
  radar_rx((const uint8_t *)msg.data(), msg.size(), true);
}

std::string read(uint32_t max = 64, uint32_t wait_ms = 2000) {
  std::string out;
  for (uint32_t t = 0; t < wait_ms;) {
    const uint8_t *p;
    bool eom = false;
    size_t n = radar_tx_peek(&p, max, &eom);
    if (!n) {
      run(1);
      t++;
      continue;
    }
    out += text(p, n);
    radar_tx_done(n);
    if (eom)
      return out;
  }
  return out + "<timeout>";
}

std::string q(const std::string &cmd) {
  send(cmd + "\n");
  std::string r = read();
  return r.size() && r.back() == '\n' ? r.substr(0, r.size() - 1) : r;
}

std::string err() { return q("SYST:ERR?"); }

std::vector<uint16_t> hex_frame() {
  std::vector<uint16_t> out;
  for (int i = 0; i < 1000; i++) {
    std::string r = q("CAPT:FRAM?");
    REQUIRE(r.size() > 0);
    if (r == "Not Ready")
      return out;
    REQUIRE(r.size() % 4 == 0);
    REQUIRE(r.size() <= 124);
    for (size_t i = 0; i < r.size(); i += 4)
      out.push_back((uint16_t)std::stoul(r.substr(i, 4), nullptr, 16));
  }
  FAIL("frame does not end");
  return out;
}

struct Block {
  uint32_t seq, first;
  uint16_t count, flags;
  std::vector<uint16_t> s;
  bool empty;
};

uint32_t le32(const std::string &b, size_t o) {
  return (uint8_t)b[o] | (uint8_t)b[o + 1] << 8 | (uint8_t)b[o + 2] << 16 |
         (uint32_t)(uint8_t)b[o + 3] << 24;
}

uint16_t le16(const std::string &b, size_t o) {
  return (uint16_t)((uint8_t)b[o] | (uint8_t)b[o + 1] << 8);
}

Block block(uint32_t max = 64) {
  send("CAPT:BLOC?\n");
  std::string b = read(max);
  REQUIRE(b.size() >= 9);
  REQUIRE(b.substr(0, 2) == "#6");
  size_t len = std::stoul(b.substr(2, 6));
  REQUIRE(b.size() == 8 + len + 1);
  REQUIRE(b.back() == '\n');
  Block r{};
  r.empty = len == 0;
  if (r.empty)
    return r;
  r.seq = le32(b, 8);
  r.first = le32(b, 12);
  r.count = le16(b, 16);
  r.flags = le16(b, 18);
  REQUIRE(len == 12u + 2u * r.count);
  for (size_t i = 0; i < r.count; i++)
    r.s.push_back(le16(b, 20 + 2 * i));
  return r;
}

uint32_t frame_start(const std::vector<uint16_t> &s) {
  for (uint32_t i = 0; i < 1u << 20; i++)
    if (Board::code(i) == s[0] && Board::code(i + 1) == s[1])
      return i;
  FAIL("frame start not found");
  return 0;
}

void check_samples(const std::vector<uint16_t> &s, uint32_t a0) {
  for (size_t k = 0; k < s.size(); k++)
    REQUIRE(s[k] == Board::code(a0 + (uint32_t)k));
}

struct Fixture {
  Board board;
  Fixture() {
    B = &board;
    radar_init();
  }
};

} // namespace

extern "C" {
void hal_rf(bool on) { B->rf = on; }
void hal_synth_write(uint8_t reg, uint32_t data) {
  REQUIRE(!B->clock);
  B->writes.push_back({reg, data});
  B->land({reg, data}, B->sample);
}
uint32_t hal_synth_read(uint8_t reg) {
  REQUIRE(!B->clock);
  return reg == 0x12 ? B->lock : B->regs[reg];
}
void hal_synth_load(uint8_t reg, uint32_t data) {
  REQUIRE(!B->clock);
  REQUIRE(B->spi2 == 0);
  B->pending = {reg, data};
  B->spi2 = 1;
}
void hal_clock_start(void) {
  REQUIRE(!B->clock);
  REQUIRE((B->regs[0x0F] & 0xC0) == 0xC0);
  B->clock = true;
  B->sample = 0;
  B->triggers.clear();
}
void hal_clock_stop(void) {
  if (B->spi2 == 1)
    B->land(B->pending, B->sample);
  B->spi2 = 0;
  B->clock = false;
}
uint32_t hal_edges(uint16_t ms) {
  REQUIRE(!B->clock);
  uint32_t sel = B->regs[0x0F] & 0x1F;
  return sel == B->good_gpo ? ms * B->per_ms() : ms * 20000u;
}
uint16_t hal_temp_raw(void) {
  REQUIRE(!B->clock);
  return B->temp;
}
bool hal_eeprom_read(uint32_t addr, uint8_t *buf, size_t n) {
  REQUIRE(!B->clock);
  std::memcpy(buf, &B->eeprom[addr], n);
  return B->eeprom_ok;
}
void hal_delay_us(uint16_t us) { (void)us; }
uint32_t hal_ms(void) {
  if (B->polled)
    B->tick();
  B->polled = true;
  return B->ms;
}
void hal_reset(void) { B->resets++; }
}

TEST_CASE("fixed-point and hex formatting") {
  char s[32];
  CHECK(fmt_fixed(s, 2400000000LL, 9) == 11);
  CHECK(std::string(s) == "2.400000000");
  fmt_fixed(s, 5, 2);
  CHECK(std::string(s) == "0.05");
  fmt_fixed(s, -1234, 2);
  CHECK(std::string(s) == "-12.34");
  fmt_fixed(s, 42, 0);
  CHECK(std::string(s) == "42");
  fmt_fixed(s, 0, 0);
  CHECK(std::string(s) == "0");
  fmt_hex16(s, 0xBEEF);
  CHECK(std::string(s, 4) == "BEEF");
}

TEST_CASE("frequency words, sweep plan and register values") {
  synth_plan_t p;
  CHECK(synth_nq(2.4) == 120 * 16777216L);
  CHECK(synth_hz(synth_nq(2.4)) == 2400000000u);
  CHECK(synth_hz(SYNTH_NQ_MIN) == 2250000000u);
  CHECK(synth_hz(SYNTH_NQ_MAX) == 2500000000u);
  CHECK(synth_div_round(7, 2) == 4);
  CHECK(synth_div_round(-7, 2) == -4);
  CHECK(synth_div_round(5, 3) == 2);

  REQUIRE(synth_plan(&p, 2.4, 2.5, 16.0, 1.0, 800));
  CHECK(p.nr == 400);
  CHECK(p.nd == 25);
  CHECK(p.k == 320000u);
  CHECK(p.step == 262);
  CHECK(p.n1 == p.n0 + 262 * 320000);
  CHECK(p.ns == p.n0 + 262 * (int32_t)(SYNTH_MARGIN / 2));
  CHECK(p.ne - p.ns == p.step * (int32_t)(p.k - SYNTH_MARGIN));
  CHECK(synth_hz(p.n1) == 2499945068u);

  REQUIRE(synth_plan(&p, 2.5, 2.4, 8.0, 1.0, 800));
  CHECK(p.step == -524);
  CHECK(p.n1 < p.n0);

  CHECK_FALSE(synth_plan(&p, 2.4, 2.4, 8.0, 1.0, 800));
  CHECK_FALSE(synth_plan(&p, 2.4, 2.5, 0.1, 1.0, 800));
  CHECK_FALSE(synth_plan(&p, 2.4, 2.5, 0.0, 1.0, 800));
  CHECK(synth_plan(&p, 2.4, 2.5, 0.16, 1.0, 800));
  CHECK_FALSE(synth_plan(&p, 2.25, 2.5, 0.16, 1.0, 16383));
  CHECK(synth_samples(-1.0, 800) == 0);

  CHECK(synth_cp_offset(synth_nq(2.375)) == 33);
  CHECK(synth_cp_offset(synth_nq(2.25)) == 34);
  CHECK(synth_cp_offset(synth_nq(2.5)) == 33);
  CHECK(synth_reg09(synth_nq(2.45)) == (0x803264u | 33u << 14 | 1u << 21));
  CHECK(synth_reg06(SD_TWOWAY) == (0x201D1Fu | 6u << 5));
  CHECK((REG06_BASE >> 21 & 1u) == 1u);
  CHECK((REG07 >> 21 & 1u) == 1u);
  CHECK(synth_mode(TYPE_CW) == SD_FRAC);
  CHECK(synth_mode(TYPE_RAMP) == SD_ONEWAY);
  CHECK(synth_mode(TYPE_SAW) == SD_ONEWAY);
  CHECK(synth_mode(TYPE_TRI) == SD_TWOWAY);
  CHECK(synth_mode(TYPE_AUTO) == SD_TWOWAY);
}

TEST_CASE_FIXTURE(Fixture, "power-up programming and identity") {
  CHECK_FALSE(board.rf);
  std::vector<uint8_t> order;
  for (auto &w : board.writes)
    order.push_back(w.reg);
  REQUIRE(order.size() > 12);
  CHECK(order[0] == 0x01);
  CHECK(order[1] == 0x02);
  CHECK(board.writes[1].data == 800);
  CHECK(order[2] == 0x06);
  CHECK(board.writes[2].data == synth_reg06(SD_FRAC));
  CHECK(order[3] == 0x07);
  CHECK(board.writes[3].data == REG07);
  CHECK(board.writes[4].data == (REG07 | REG07_TRAIN));
  CHECK(board.regs[0x0F] == REG0F_IDLE);
  CHECK(q("*IDN?") == "Quonset Microwave,QM4004,0021,CV1.0.0-0123456789ab");
  CHECK(err() == "0,\"No error\"");
  CHECK(q("SYST:STAT?") == "0,\"Operational\"");
  CHECK(q("SYST:TEMP?") == "25.00");
  CHECK(q("SYST:IDEN?") == "QM4004");
  CHECK(q("SYST:MODNUM?") == "QM4004");
  CHECK(q("SYST:SERNUM?") == "0021");
  CHECK(q("SYST:FIRM?") == "CV1.0.0-0123456789ab");
  CHECK(q("SYST:VERS?") == "1999.0");
  CHECK(q("*TST?") == "0");
  CHECK(std::string(radar_serial()) == "0021");
  board.temp = 0x4000 - 64;
  CHECK(q("SYST:TEMP?") == "-2.00");
  send("*RST\n");
  CHECK(board.resets == 1);
}

TEST_CASE("serial number from the EEPROM") {
  Board board;
  B = &board;
  board.eeprom[0xFE] = 0x6F;
  board.eeprom[0x100] = 42;
  board.eeprom[0x101] = 0;
  radar_init();
  CHECK(q("*IDN?") == "Quonset Microwave,QM4004,0042,CV1.0.0-0123456789ab");
  board.eeprom[0x100] = 0x10;
  board.eeprom[0x101] = 0x27;
  radar_init();
  CHECK(std::string(radar_serial()) == "0021");
  board.eeprom[0x100] = 0x0F;
  board.eeprom_ok = false;
  radar_init();
  CHECK(std::string(radar_serial()) == "0021");
}

TEST_CASE_FIXTURE(Fixture, "power-up settings and limits") {
  CHECK(q("SWEEP:FREQSTAR?") == "2.400000000");
  CHECK(q("SWEEP:FREQSTOP?") == "2.499945068");
  CHECK(q("SWEEP:RAMPTIME?") == "16.000000");
  CHECK(q("SWEEP:DWEL?") == "1.000000");
  CHECK(q("SWEEP:TYPE?") == "2");
  CHECK(q("POWE:RF?") == "0");
  CHECK(q("POWE:RF:TIME?") == "30");
  CHECK(q("FREQ:LOCK?") == "1");
  CHECK(q("FREQ:REF:DIV?") == "1");
  CHECK(q("CAPT:RATE?") == "25000.000000");
  CHECK(q("SWEEP:ORIG?") == "0.018125,1");
  CHECK(q("SWEEP:LIM?") ==
        "2.250000000,2.500000000,23841857.910156,0.040000,0.160000");
  CHECK(err() == "0,\"No error\"");
}

TEST_CASE_FIXTURE(Fixture, "out-of-range parameters return 201") {
  for (const char *cmd :
       {"SWEEP:FREQSTAR 2.6", "SWEEP:FREQSTOP 2.2", "SWEEP:RAMPTIME 0",
        "SWEEP:RAMPTIME 0.1", "SWEEP:RAMPTIME 65537", "SWEEP:DWEL 0.1",
        "SWEEP:TYPE 5", "FREQ:REF:DIV 257", "FREQ:REF:DIV 0", "CAPT:FRAM 4097",
        "CAPT:FRAM 0", "CAPT:RATE 45000", "CAPT:RATE 1000",
        "POWE:RF:TIME 4000"}) {
    send(std::string(cmd) + "\n");
    CHECK_MESSAGE(err() == "201,\"Out of range\"", cmd);
    CHECK(err() == "0,\"No error\"");
  }
  CHECK(q("SWEEP:FREQSTAR?") == "2.400000000");
  CHECK(q("SWEEP:RAMPTIME?") == "16.000000");
  send("SWEEP:FREQSTAR\n");
  CHECK(err() == "-109,\"Missing parameter\"");
  send("SWEEP:BOGUS 1\n");
  CHECK(err() == "-113,\"Undefined header\"");
}

TEST_CASE_FIXTURE(Fixture, "decimal ramp times are realised in samples") {
  send("SWEEP:RAMPTIME 8.5\n");
  CHECK(q("SWEEP:RAMPTIME?") == "8.520000");
  send("SWEEP:RAMPTIME 65536\n");
  CHECK(q("SWEEP:RAMPTIME?") == "65536.000000");
  send("SWEEP:FREQSTAR 2.5;:SWEEP:FREQSTOP 2.4;:SWEEP:RAMPTIME 8\n");
  CHECK(q("SWEEP:ORIG?") == "0.018125,0");
  send("SWEEP:START\n");
  CHECK(board.regs[0x0A] == ((uint32_t)-524 & 0xFFFFFFu));
  send("FREQ:REF:DIV 4\n");
  CHECK(q("FREQ:REF:DIV?") == "4");
  CHECK(board.regs[0x02] == 800);
  CHECK(err() == "0,\"No error\"");
}

TEST_CASE_FIXTURE(Fixture, "triangle sweep runs from the sample clock") {
  send("SWEEP:RAMPTIME 8\n");
  send("SWEEP:START\n");
  CHECK(err() == "0,\"No error\"");
  CHECK(board.rf);
  CHECK(board.clock);
  CHECK(board.regs[0x06] == synth_reg06(SD_TWOWAY));
  CHECK((board.regs[0x0F] & 0x1F) == 9);
  CHECK(board.regs[0x09] == synth_reg09(synth_nq(2.45)));
  synth_plan_t p;
  REQUIRE(core_plan(&p));
  CHECK(board.regs[0x03] == (uint32_t)p.ns >> 24);
  CHECK(board.regs[0x04] == ((uint32_t)p.ns & 0xFFFFFFu));
  CHECK(board.regs[0x0C] == (uint32_t)p.ne >> 24);
  CHECK(board.regs[0x0D] == ((uint32_t)p.ne & 0xFFFFFFu));
  CHECK(board.regs[0x0A] == (uint32_t)p.step);
  run(100);
  REQUIRE(board.triggers.size() >= 10);
  for (size_t i = 0; i < board.triggers.size(); i++)
    CHECK(board.triggers[i] == 200u * i);
  CHECK(q("FREQ:LOCK?") == "1");
  board.lock = 0;
  CHECK(q("FREQ:LOCK?") == "1");
  CHECK(q("SYST:TEMP?") == "25.00");
  send("SWEEP:STOP\n");
  CHECK_FALSE(board.clock);
  CHECK_FALSE(board.rf);
  CHECK(board.regs[0x0E] == 0);
  CHECK(board.regs[0x0F] == REG0F_IDLE);
  CHECK(q("FREQ:LOCK?") == "0");
}

TEST_CASE_FIXTURE(Fixture, "frames are aligned to the triangle period") {
  send("SWEEP:RAMPTIME 8\n");
  send("SWEEP:START\n");
  run(7);
  send("CAPT:FRAM 1000\n");
  std::vector<uint16_t> s = hex_frame();
  REQUIRE(s.size() == 1000);
  uint32_t a0 = frame_start(s);
  CHECK(a0 % 400u == 0);
  CHECK(a0 > 0);
  check_samples(s, a0);
  CHECK(q("CAPT:FRAM?") == "Not Ready");
  CHECK(board.clock);
  send("CAPT:FRAM 4096\n");
  s = hex_frame();
  REQUIRE(s.size() == 4096);
  a0 = frame_start(s);
  CHECK(a0 % 400u == 0);
  check_samples(s, a0);
}

TEST_CASE_FIXTURE(Fixture, "frames as binary blocks with partial transfers") {
  send("SWEEP:START\n");
  send("CAPT:FRAM 4096\n");
  std::vector<uint16_t> all;
  for (uint32_t i = 0; i < 4; i++) {
    Block b = block(i % 2 ? 100 : 4096);
    CHECK(b.seq == i);
    CHECK(b.first == 1024u * i);
    CHECK(b.count == 1024);
    CHECK(b.flags == (i == 3 ? BLOCK_END : 0));
    all.insert(all.end(), b.s.begin(), b.s.end());
  }
  CHECK(block().empty);
  uint32_t a0 = frame_start(all);
  CHECK(a0 % 800u == 0);
  check_samples(all, a0);
}

TEST_CASE_FIXTURE(Fixture, "continuous acquisition with a slow reader") {
  send("SWEEP:START\n");
  send("CAPT:CONT 0\n");
  run(400);
  Block b = block();
  CHECK(b.seq == 0);
  CHECK(b.flags == 0);
  uint32_t a0 = frame_start(b.s);
  check_samples(b.s, a0);
  for (uint32_t i = 1; i < 5; i++)
    CHECK(block().seq == i);
  b = block();
  CHECK(b.seq == 5);
  CHECK(b.flags == BLOCK_OVERFLOW);
  CHECK(b.first > 5u * 1024u);
  check_samples(b.s, a0 + b.first);
  Block c = block();
  CHECK(c.first == b.first + 1024u);
  CHECK(c.flags == 0);
  send("CAPT:ABOR\n");
  CHECK(block().empty);
  CHECK(board.clock);
  send("CAPT:CONT 3000\n");
  b = block();
  a0 = frame_start(b.s);
  CHECK(block().count == 1024);
  c = block();
  CHECK(c.count == 952);
  CHECK(c.flags == BLOCK_END);
  check_samples(c.s, a0 + 2048);
  CHECK(block().empty);
}

TEST_CASE_FIXTURE(Fixture, "overflow at the end of an acquisition") {
  send("SWEEP:START\n");
  send("CAPT:CONT 6000\n");
  run(400);
  for (uint32_t i = 0; i < 5; i++)
    CHECK(block().count == 1024);
  Block b = block();
  CHECK(b.count == 0);
  CHECK(b.flags == (BLOCK_END | BLOCK_OVERFLOW));
  CHECK(b.first == 6000);
  CHECK(block().empty);
}

TEST_CASE_FIXTURE(Fixture, "single ramp and single triangle per frame") {
  send("SWEEP:TYPE 0;:SWEEP:RAMPTIME 4\n");
  send("SWEEP:START\n");
  CHECK_FALSE(board.clock);
  CHECK(board.regs[0x06] == synth_reg06(SD_ONEWAY));
  send("CAPT:FRAM 500\n");
  CHECK(board.clock);
  std::vector<uint16_t> s = hex_frame();
  REQUIRE(s.size() == 500);
  check_samples(s, 0);
  CHECK(board.triggers == std::vector<uint32_t>{0});
  run(2);
  CHECK_FALSE(board.clock);
  send("SWEEP:TYPE 1\n");
  CHECK(board.rf);
  send("SWEEP:START\n");
  send("CAPT:FRAM 1000\n");
  run(50);
  CHECK(board.triggers == (std::vector<uint32_t>{0, 100}));
  CHECK_FALSE(board.clock);
  CHECK(hex_frame().size() == 1000);
}

TEST_CASE_FIXTURE(Fixture, "sawtooth triggers and alignment") {
  send("SWEEP:TYPE 4;:SWEEP:RAMPTIME 4;:SWEEP:DWEL 0.5\n");
  CHECK(q("SWEEP:DWEL?") == "0.520000");
  send("SWEEP:START\n");
  CHECK(board.regs[0x06] == synth_reg06(SD_ONEWAY));
  run(60);
  REQUIRE(board.triggers.size() > 6);
  for (size_t i = 0; i < board.triggers.size(); i++)
    CHECK(board.triggers[i] == (i / 2) * 113u + (i % 2) * 100u);
  send("CAPT:FRAM 300\n");
  std::vector<uint16_t> s = hex_frame();
  CHECK(frame_start(s) % 113u == 0);
  send("SWEEP:DWEL 0.1\n");
  CHECK(err() == "201,\"Out of range\"");
}

TEST_CASE_FIXTURE(Fixture, "CW and stopped captures run the clock only for "
                           "the acquisition") {
  send("SWEEP:TYPE 3\n");
  send("SWEEP:START\n");
  CHECK(board.rf);
  CHECK_FALSE(board.clock);
  CHECK(board.regs[0x06] == synth_reg06(SD_FRAC));
  CHECK(q("SWEEP:ORIG?") == "0.000000,1");
  send("CAPT:FRAM 64\n");
  std::vector<uint16_t> s = hex_frame();
  check_samples(s, 0);
  CHECK(board.triggers.empty());
  run(2);
  CHECK_FALSE(board.clock);
  CHECK(board.regs[0x0F] == REG0F_IDLE);
  send("SWEEP:STOP\n");
  send("CAPT:FRAM 10\n");
  CHECK(hex_frame().size() == 10);
}

TEST_CASE_FIXTURE(Fixture, "setters re-apply a running sweep") {
  send("SWEEP:START\n");
  send("SWEEP:RAMPTIME 4\n");
  CHECK(board.clock);
  run(10);
  CHECK(board.triggers[1] == 100);
  send("SWEEP:FREQSTOP 2.4\n");
  CHECK(err() == "201,\"Out of range\"");
  CHECK_FALSE(board.clock);
  CHECK(board.rf);
  send("SWEEP:START\n");
  CHECK(err() == "201,\"Out of range\"");
  send("SWEEP:FREQSTOP 2.5;:SWEEP:START\n");
  CHECK(board.clock);
  send("SWEEP:TYPE 2\n");
  CHECK_FALSE(board.clock);
  CHECK(board.rf);
  CHECK(q("POWE:RF?") == "1");
}

TEST_CASE_FIXTURE(Fixture, "lock is latched before the sample clock starts") {
  board.lock = 0;
  send("SWEEP:START\n");
  CHECK(board.clock);
  board.lock = 2;
  CHECK(q("FREQ:LOCK?") == "0");
  send("SWEEP:TYPE 3;:SWEEP:START\n");
  CHECK(q("FREQ:LOCK?") == "1");
}

TEST_CASE_FIXTURE(Fixture, "sample rate selection") {
  send("SWEEP:START\n");
  send("CAPT:RATE 40000\n");
  CHECK(board.regs[0x02] == 500);
  CHECK(board.clock);
  CHECK(q("CAPT:RATE?") == "40000.000000");
  CHECK(q("SWEEP:ORIG?") == "0.029000,1");
  CHECK(q("SWEEP:LIM?") ==
        "2.250000000,2.500000000,23841857.910156,0.025000,0.100000");
  CHECK(q("SWEEP:RAMPTIME?") == "16.000000");
  send("CAPT:RATE 1221\n");
  CHECK(q("CAPT:RATE?") == "1221.001221");
  CHECK(core_set_rate(SYNTH_R_MAX + 1) == false);
  send("SWEEP:STOP;:SWEEP:RAMPTIME 0.2;:CAPT:RATE 1221\n");
  CHECK(err() == "201,\"Out of range\"");
}

TEST_CASE_FIXTURE(Fixture, "RF auto-mute after host silence") {
  send("POWE:RF 1\n");
  CHECK(board.rf);
  run(29000);
  CHECK(board.rf);
  run(1001);
  CHECK_FALSE(board.rf);
  send("POWE:RF:TIME 0;:POWE:RF ON\n");
  run(40000);
  CHECK(board.rf);
  send("POWE:RF:TIME 1;:SWEEP:START;:CAPT:CONT 0\n");
  run(3000);
  CHECK(board.rf);
}

TEST_CASE("sample-clock self-test") {
  Board board;
  B = &board;
  board.good_gpo = 7;
  radar_init();
  CHECK(err() == "0,\"No error\"");
  send("SWEEP:START\n");
  CHECK((board.regs[0x0F] & 0x1F) == 7);
  Board fail;
  B = &fail;
  fail.good_gpo = 0;
  radar_init();
  CHECK(err() == "-330,\"Self-test failed\"");
  send("CAPT:FRAM 10\n");
  CHECK(err() == "-330,\"Self-test failed\"");
  send("SWEEP:START\n");
  CHECK(err() == "-330,\"Self-test failed\"");
  CHECK_FALSE(fail.clock);
  send("SWEEP:TYPE 3;:SWEEP:START\n");
  CHECK(err() == "0,\"No error\"");
}

TEST_CASE_FIXTURE(Fixture, "message framing and output handling") {
  radar_rx((const uint8_t *)"*ID", 3, false);
  radar_rx((const uint8_t *)"N?", 2, true);
  CHECK(read() == "Quonset Microwave,QM4004,0021,CV1.0.0-0123456789ab\n");
  send("SWEEP:TYPE?");
  CHECK(read() == "2\n");
  send("*IDN?\n");
  send("SWEEP:TYPE?\n");
  CHECK(read() == "2\n");
  const uint8_t *p;
  bool eom;
  CHECK(radar_tx_peek(&p, 64, &eom) == 0);
  send("*IDN?\n");
  CHECK((radar_stb() & STB_MAV) != 0);
  radar_clear();
  CHECK(radar_tx_peek(&p, 64, &eom) == 0);
  CHECK((radar_stb() & STB_MAV) == 0);
  radar_tx_done(5);
  send("SWEEP:START;:CAPT:FRAM 100\n");
  send("CAPT:FRAM?\n");
  CHECK(radar_tx_peek(&p, 64, &eom) == 0);
  CHECK(read().size() == 4 * 31 + 1);
}
