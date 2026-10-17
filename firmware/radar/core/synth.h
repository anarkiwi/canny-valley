/* HMC703 frequency words, sweep plan and register values (pure functions).
   Frequencies are Q24 multiples of f_PD: N = f / f_PD, Nq = N * 2^24. */
#ifndef RADAR_SYNTH_H
#define RADAR_SYNTH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SYNTH_FREF 20000000L
/* Sample period R in reference cycles: Timer2 counts the reference (GPO
   select 8) through its 1:8 prescaler, so R is a multiple of 8. */
#define SYNTH_R_STEP 8u
#define SYNTH_R_MIN 504u
#define SYNTH_R_MAX 16376u
#define SYNTH_R_DEF 800u
#define SYNTH_GPO_XTAL 8u
#define SYNTH_MARGIN 16u
/* TCY (FCY 16 MHz = 1.25 PD cycles) from the CNV rising edge to the 32nd
   SCK2 edge of the trigger frame in the sample ISR. */
#define SYNTH_DELAY_TCY 18u
#define SYNTH_GAP_MIN 4u
#define SYNTH_NQ_MIN 1929379840L
#define SYNTH_NQ_MAX 2097152000L
#define SYNTH_STEP_MAX 0x7FFFFFL
#define SYNTH_RAMP_MS_MAX 65536.0

enum { SD_FRAC = 0, SD_ONEWAY = 5, SD_TWOWAY = 6 };
enum { TYPE_RAMP = 0, TYPE_TRI = 1, TYPE_AUTO = 2, TYPE_CW = 3, TYPE_SAW = 4 };

#define REG06_BASE 0x001D1FUL
#define REG07_TRAIN 0x100000UL
#define REG07 0x204865UL
#define REG08 0x016FFFUL
#define REG09_BASE 0x803264UL
#define REG09_SRC 0x200000UL
#define REG0B 0x01EC71UL
#define REG0F_IDLE 0x000001UL
#define REG0F_EXPORT 0x0000C0UL

typedef struct {
  int32_t n0;   /* ideal start apex */
  int32_t n1;   /* ideal stop apex, n0 + step * k */
  int32_t step; /* MODSTEP */
  int32_t ns;   /* programmed start */
  int32_t ne;   /* programmed stop */
  uint32_t nr;  /* samples per ramp */
  uint32_t nd;  /* samples of sawtooth rest */
  uint32_t k;   /* PD cycles per ramp, nr * R */
} synth_plan_t;

int32_t synth_nq(double ghz);
uint32_t synth_samples(double ms, uint16_t r);
int64_t synth_div_round(int64_t num, int64_t den);
/* False if the ramp is shorter than SYNTH_GAP_MIN samples, the slope rounds
   to zero or the step overflows MODSTEP. */
bool synth_plan(synth_plan_t *p, double f0, double f1, double ramp_ms,
                double dwell_ms, uint16_t r);
uint32_t synth_hz(int32_t nq);
uint8_t synth_cp_offset(int32_t centre_nq);
uint32_t synth_reg09(int32_t centre_nq);
uint32_t synth_reg06(uint8_t mode);
uint8_t synth_mode(uint8_t type);

#ifdef __cplusplus
}
#endif

#endif
