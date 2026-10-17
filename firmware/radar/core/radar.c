#include <string.h>

#include "core.h"
#include "fmt.h"
#include "hal.h"

core_t core;
sched_t radar_sched;
sched_slot_t radar_slots[SCHED_NSLOT];

static sched_t *const S = &radar_sched;
static const uint8_t gpo_select[] = {9, 7};

static void synth_word(uint8_t reg_int, int32_t nq) {
  hal_synth_write(reg_int, (uint32_t)nq >> 24);
  hal_synth_write((uint8_t)(reg_int + 1u), (uint32_t)nq & 0xFFFFFFu);
}

static bool wait_lock(void) {
  uint32_t t0 = hal_ms();
  do {
    if (hal_synth_read(0x12) & 2u)
      return true;
  } while (hal_ms() - t0 < CORE_LOCK_MS);
  return false;
}

static bool acquiring(void) {
  return core.clock && (S->acq == ACQ_WAIT || S->acq == ACQ_RUN);
}

static void drop_output(void) {
  core.out = OUT_NONE;
  core.text_len = core.sent = 0;
}

static void discard(void) {
  S->rd_count = S->wr_count;
  core.rd_slot = S->wr_slot;
  core.hex_off = 0;
  S->end_pending = 0;
  if (core.out == OUT_BLOCK || core.out == OUT_HEX)
    drop_output();
}

static void clock_stop(void) {
  if (!core.clock)
    return;
  hal_clock_stop();
  core.clock = false;
  hal_synth_write(0x0E, 0);
  hal_synth_write(0x0F, REG0F_IDLE);
}

static void clock_start(uint16_t triggers, uint32_t gap0, uint32_t gap1,
                        uint32_t period, bool arm, uint32_t n) {
  core.temp = hal_temp_raw();
  memset(S, 0, sizeof *S);
  S->untrig_at = 0xFFFFFFFFu;
  S->gap[0] = gap0;
  S->gap[1] = gap1;
  S->period = period;
  S->trig_left = triggers;
  S->arm_n = n;
  S->req = arm ? REQ_ARM : REQ_NONE;
  core.rd_slot = 0;
  core.hex_off = 0;
  hal_synth_write(0x0F, core.gpo | REG0F_EXPORT);
  if (triggers)
    hal_synth_load(0x0E, 1);
  hal_clock_start();
  core.clock = true;
}

bool core_plan(synth_plan_t *p) {
  return synth_plan(p, core.f0, core.f1, core.ramp_ms, core.dwell_ms,
                    core.rdiv) &&
         (core.type != TYPE_SAW || p->nd >= SYNTH_GAP_MIN);
}

void core_abort(void) {
  uint32_t t0;
  if (core.clock && (S->acq != ACQ_IDLE || S->req)) {
    S->req = REQ_ABORT;
    t0 = hal_ms();
    while (S->req && hal_ms() - t0 < CORE_ACK_MS)
      ;
  }
  S->req = REQ_NONE;
  S->acq = ACQ_IDLE;
  if (!(core.running && core.continuous))
    clock_stop();
  discard();
}

static void halt(void) {
  core_abort();
  clock_stop();
  if (core.running)
    hal_synth_write(0x06, synth_reg06(SD_FRAC));
  core.running = core.continuous = false;
}

void core_stop(bool rf_off) {
  halt();
  if (rf_off) {
    hal_rf(false);
    core.rf = false;
  }
}

bool core_start(void) {
  synth_plan_t p;
  bool sweep = core.type != TYPE_CW;
  bool ok = core_plan(&p);
  halt();
  if (sweep && !ok) {
    SCPI_ErrorPush(&core.scpi, RADAR_ERR_RANGE);
    return false;
  }
  if (sweep && !core.clock_ok) {
    SCPI_ErrorPush(&core.scpi, RADAR_ERR_CLOCK);
    return false;
  }
  hal_synth_write(0x0E, 0);
  hal_synth_write(0x06, synth_reg06(SD_FRAC));
  hal_synth_write(
      0x09, synth_reg09(sweep ? (int32_t)(((int64_t)p.n0 + p.n1) / 2) : p.n0));
  if (sweep) {
    hal_synth_write(0x0A, (uint32_t)p.step & 0xFFFFFFu);
    synth_word(0x0C, p.ne);
  }
  synth_word(0x03, sweep ? p.ns : p.n0);
  core.locked = wait_lock();
  if (sweep)
    hal_synth_write(0x06, synth_reg06(synth_mode(core.type)));
  core.running = true;
  core.continuous = core.type == TYPE_AUTO || core.type == TYPE_SAW;
  hal_rf(true);
  core.rf = true;
  if (core.continuous)
    clock_start(SCHED_FOREVER, p.nr, core.type == TYPE_SAW ? p.nd : p.nr,
                core.type == TYPE_SAW ? p.nr + p.nd : 2u * p.nr, false, 0);
  return true;
}

void core_reapply(void) {
  if (core.running && !core_start())
    core_stop(false);
}

bool core_set_rate(uint16_t r) {
  bool was = core.running;
  if (r < SYNTH_R_MIN || r > SYNTH_R_MAX)
    return false;
  halt();
  core.rdiv = r;
  hal_synth_write(0x02, r);
  if (was && !core_start())
    core_stop(false);
  return true;
}

bool core_acquire(uint32_t n) {
  synth_plan_t p;
  if (!core.clock_ok) {
    SCPI_ErrorPush(&core.scpi, RADAR_ERR_CLOCK);
    return false;
  }
  core_abort();
  if (core.running && core.continuous) {
    S->arm_n = n;
    S->req = REQ_ARM;
  } else if (core.running && core.type != TYPE_CW) {
    core_plan(&p);
    hal_synth_write(0x06, synth_reg06(SD_FRAC));
    core.locked = wait_lock();
    hal_synth_write(0x06, synth_reg06(synth_mode(core.type)));
    clock_start(core.type == TYPE_RAMP ? 1 : 2, p.nr, p.nr, 1, true, n);
  } else {
    clock_start(0, 0, 0, 1, true, n);
  }
  return true;
}

bool core_lock(void) {
  if (!core.clock)
    core.locked = (hal_synth_read(0x12) & 2u) != 0;
  return core.locked;
}

uint16_t core_temp(void) {
  if (!core.clock)
    core.temp = hal_temp_raw();
  return core.temp;
}

static void load_serial(void) {
  uint8_t mark = 0, b[2] = {0, 0};
  unsigned v = CORE_SERIAL_DEF, i;
  if (hal_eeprom_read(0x00FE, &mark, 1) && mark == 0x6F &&
      hal_eeprom_read(0x0100, b, 2) && (unsigned)(b[0] | b[1] << 8) < 10000u)
    v = (unsigned)(b[0] | b[1] << 8);
  for (i = 4; i--; v /= 10u)
    core.serial[i] = (char)('0' + v % 10u);
  core.serial[4] = 0;
}

static void self_test(void) {
  uint32_t expect =
      CORE_SELFTEST_MS * (uint32_t)(SYNTH_FREF / 1000) / core.rdiv;
  uint32_t got;
  size_t i;
  for (i = 0; i < sizeof gpo_select && !core.gpo; i++) {
    hal_synth_write(0x0F, gpo_select[i] | REG0F_EXPORT);
    got = hal_edges(CORE_SELFTEST_MS);
    if ((got > expect ? got - expect : expect - got) <= expect / 50u + 2u)
      core.gpo = gpo_select[i];
  }
  hal_synth_write(0x0F, REG0F_IDLE);
  core.clock_ok = core.gpo != 0;
}

void radar_init(void) {
  memset(&core, 0, sizeof core);
  memset(S, 0, sizeof *S);
  core.f0 = 2.4;
  core.f1 = 2.5;
  core.ramp_ms = 16.0;
  core.dwell_ms = 1.0;
  core.type = TYPE_AUTO;
  core.rdiv = SYNTH_R_DEF;
  core.refdiv = 1;
  core.timeout_s = CORE_TIMEOUT_DEF;
  hal_rf(false);
  hal_synth_read(0x00);
  hal_synth_write(0x01, 0x000002u);
  hal_synth_write(0x02, core.rdiv);
  hal_synth_write(0x06, synth_reg06(SD_FRAC));
  hal_synth_write(0x07, REG07);
  hal_synth_write(0x07, REG07 | REG07_TRAIN);
  hal_synth_write(0x08, REG08);
  hal_synth_write(0x09, synth_reg09(synth_nq(core.f0)));
  hal_synth_write(0x0B, REG0B);
  hal_synth_write(0x0E, 0);
  synth_word(0x03, synth_nq(core.f0));
  self_test();
  load_serial();
  core.temp = hal_temp_raw();
  core_scpi_init();
  if (!core.clock_ok)
    SCPI_ErrorPush(&core.scpi, RADAR_ERR_CLOCK);
  core.last_rx = hal_ms();
}

void radar_poll(void) {
  if (core.clock && S->acq == ACQ_DONE) {
    if (S->end_pending && sched_fill(S) < SCHED_NSLOT) {
      sched_open(S, radar_slots);
      sched_close(S, radar_slots, BLOCK_END);
      S->end_pending = 0;
    }
    if (!S->end_pending && !(core.running && core.continuous))
      clock_stop();
  }
  if (core.rf && core.timeout_s && !acquiring() &&
      hal_ms() - core.last_rx >= core.timeout_s * 1000ul) {
    hal_rf(false);
    core.rf = false;
  }
}

void radar_rx(const uint8_t *data, size_t len, bool eom) {
  core.last_rx = hal_ms();
  if (!core.in_msg)
    drop_output();
  core.in_msg = !eom;
  if (len)
    SCPI_Input(&core.scpi, (const char *)data, (int)len);
  if (eom)
    SCPI_Input(&core.scpi, NULL, 0);
}

size_t core_text(const char *s, size_t n) {
  if (core.out == OUT_BLOCK || core.out == OUT_HEX)
    return n;
  if (core.out != OUT_TEXT) {
    core.out = OUT_TEXT;
    core.text_len = core.sent = 0;
  }
  if (n > CORE_TEXT - core.text_len)
    n = CORE_TEXT - core.text_len;
  memcpy(core.text + core.text_len, s, n);
  core.text_len += n;
  return n;
}

static size_t serve(const uint8_t *buf, size_t total, const uint8_t **data,
                    uint32_t max, bool *eom) {
  size_t n = total - core.sent;
  if (n > max)
    n = max;
  *data = buf + core.sent;
  *eom = core.sent + n == total;
  return n;
}

static sched_slot_t *slot(uint8_t i) {
  return &radar_slots[(core.rd_slot + i) % SCHED_NSLOT];
}

static size_t block_total(void) { return SCHED_HDR + 2u * slot(0)->count + 1u; }

static bool block_ready(void) {
  sched_slot_t *b;
  if (sched_fill(S) == 0) {
    if (acquiring() || S->end_pending)
      return false;
    drop_output();
    core_text("#6000000\n", 9);
    return true;
  }
  if (core.sent == 0) {
    char d[8];
    size_t len;
    b = slot(0);
    len = fmt_fixed(d, (int64_t)(SCHED_HDR - SCHED_PREFIX + 2u * b->count), 0);
    memcpy(b->prefix, "#6000000", SCHED_PREFIX);
    memcpy(b->prefix + SCHED_PREFIX - len, d, len);
    ((uint8_t *)b)[SCHED_HDR + 2u * b->count] = '\n';
  }
  return true;
}

static bool hex_ready(void) {
  uint8_t i = 0, fill = sched_fill(S);
  uint16_t off = core.hex_off, n;
  size_t len = 0;
  bool end = false;
  const sched_slot_t *b;
  while (len < 4u * CORE_HEX_CHUNK && i < fill) {
    b = slot(i);
    n = b->count;
    if (off < n) {
      fmt_hex16(core.text + len, b->s[off]);
      len += 4;
      off++;
    } else if (b->flags & BLOCK_END) {
      end = true;
      break;
    } else {
      i++;
      off = 0;
    }
  }
  if (len < 4u * CORE_HEX_CHUNK && !end && (acquiring() || S->end_pending))
    return false;
  core.out = OUT_HEX;
  if (len == 0) {
    memcpy(core.text, "Not Ready\n", 10);
    core.text_len = 10;
  } else {
    core.text[len] = '\n';
    core.text_len = len + 1;
  }
  core.hex_blocks = i;
  core.hex_next = off;
  return true;
}

size_t radar_tx_peek(const uint8_t **data, uint32_t max, bool *eom) {
  switch (core.out) {
  case OUT_BLOCK:
    if (!block_ready())
      return 0;
    if (core.out == OUT_BLOCK)
      return serve((const uint8_t *)slot(0), block_total(), data, max, eom);
    break;
  case OUT_HEX:
    if (core.text_len == 0 && !hex_ready())
      return 0;
    break;
  case OUT_TEXT:
    break;
  default:
    return 0;
  }
  return serve((const uint8_t *)core.text, core.text_len, data, max, eom);
}

static void pop(uint8_t blocks) {
  while (blocks-- && sched_fill(S)) {
    S->rd_count = (uint8_t)(S->rd_count + 1u);
    core.rd_slot =
        (uint8_t)(core.rd_slot + 1u == SCHED_NSLOT ? 0 : core.rd_slot + 1u);
  }
}

void radar_tx_done(size_t len) {
  size_t total;
  if (core.out == OUT_NONE)
    return;
  total = core.out == OUT_BLOCK ? block_total() : core.text_len;
  core.sent += len;
  if (core.sent < total)
    return;
  if (core.out == OUT_BLOCK) {
    pop(1);
    core.hex_off = 0;
  } else if (core.out == OUT_HEX) {
    pop(core.hex_blocks);
    core.hex_off = core.hex_next;
  }
  drop_output();
}

void radar_clear(void) {
  core_abort();
  drop_output();
  core.in_msg = false;
  core.scpi.buffer.position = 0;
}

uint8_t radar_stb(void) {
  return (uint8_t)(SCPI_RegGet(&core.scpi, SCPI_REG_STB) |
                   (core.out != OUT_NONE ? STB_MAV : 0));
}

const char *radar_serial(void) { return core.serial; }
