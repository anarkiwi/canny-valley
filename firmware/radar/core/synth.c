#include "synth.h"

#include <math.h>

int32_t synth_nq(double ghz) {
  return (int32_t)floor(ghz * 50.0 * 16777216.0 + 0.5);
}

uint32_t synth_samples(double ms, uint16_t r) {
  double x = ms * (double)(SYNTH_FREF / 1000L) / (double)r;
  return x > 0.0 && x < 4.0e9 ? (uint32_t)floor(x + 0.5) : 0u;
}

int64_t synth_div_round(int64_t num, int64_t den) {
  return num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
}

bool synth_plan(synth_plan_t *p, double f0, double f1, double ramp_ms,
                double dwell_ms, uint16_t r) {
  int64_t step;
  p->n0 = synth_nq(f0);
  p->nr = synth_samples(ramp_ms, r);
  p->nd = synth_samples(dwell_ms, r);
  p->k = p->nr * r;
  step = p->nr ? synth_div_round((int64_t)synth_nq(f1) - p->n0, p->k) : 0;
  p->step = (int32_t)step;
  p->n1 = (int32_t)(p->n0 + step * p->k);
  p->ns = (int32_t)(p->n0 + step * (SYNTH_MARGIN / 2));
  p->ne = (int32_t)(p->n0 + step * (p->k - SYNTH_MARGIN / 2));
  return p->nr >= SYNTH_GAP_MIN && step != 0 && step <= SYNTH_STEP_MAX &&
         step >= -SYNTH_STEP_MAX;
}

uint32_t synth_hz(int32_t nq) {
  return (uint32_t)(((uint64_t)(uint32_t)nq * 78125u + 32768u) >> 16);
}

uint8_t synth_cp_offset(int32_t centre_nq) {
  return (uint8_t)(20 + synth_div_round(26843545600LL, centre_nq));
}

uint32_t synth_reg09(int32_t centre_nq) {
  return REG09_BASE | (uint32_t)synth_cp_offset(centre_nq) << 14 | REG09_SRC;
}

uint32_t synth_reg06(uint8_t mode) { return REG06_BASE | (uint32_t)mode << 5; }

uint8_t synth_mode(uint8_t type) {
  return type == TYPE_CW                         ? SD_FRAC
         : type == TYPE_RAMP || type == TYPE_SAW ? SD_ONEWAY
                                                 : SD_TWOWAY;
}
