/* State shared by the core's translation units. */
#ifndef RADAR_CORE_H
#define RADAR_CORE_H

#include "radar.h"
#include "scpi/scpi.h"
#include "synth.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CORE_TEXT 160u
#define CORE_HEX_CHUNK 31u
#define CORE_LOCK_MS 50u
#define CORE_ACK_MS 5u
#define CORE_SELFTEST_MS 20u
#define CORE_SERIAL_DEF 21u
#define CORE_TIMEOUT_DEF 30u
#define CORE_TIMEOUT_MAX 3600u
#define CORE_FRAME_MAX 4096u

enum { OUT_NONE, OUT_TEXT, OUT_BLOCK, OUT_HEX };

typedef struct {
  double f0, f1, ramp_ms, dwell_ms;
  uint16_t rdiv, refdiv, timeout_s;
  uint8_t type, gpo;
  bool rf, running, clock, continuous, locked, clock_ok;
  uint16_t temp;
  uint32_t last_rx;
  char serial[5];
  uint8_t out;
  bool in_msg;
  char text[CORE_TEXT];
  size_t text_len, sent;
  uint8_t rd_slot;
  uint16_t hex_off;
  uint8_t hex_blocks;
  uint16_t hex_next;
  scpi_t scpi;
} core_t;

extern core_t core;

void core_scpi_init(void);
size_t core_text(const char *s, size_t n);

/* Sweep and acquisition control (radar.c). */
bool core_plan(synth_plan_t *p);
bool core_start(void);
void core_stop(bool rf_off);
void core_reapply(void);
bool core_set_rate(uint16_t r);
bool core_acquire(uint32_t n);
void core_abort(void);
bool core_lock(void);
uint16_t core_temp(void);

#ifdef __cplusplus
}
#endif

#endif
