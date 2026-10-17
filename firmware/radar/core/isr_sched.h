/* Per-sample scheduler run from the sample-clock ISR: synthesiser trigger
   pipeline, acquisition alignment and block slots. One call per sample
   clock edge, after the word of the previous conversion has been read. */
#ifndef RADAR_SCHED_H
#define RADAR_SCHED_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCHED_S 1024u
#define SCHED_NSLOT 5u
#define SCHED_PREFIX 8u
#define SCHED_HDR 20u
#define SCHED_FOREVER 0xFFFFu

/* HMC703 write frame Reg 0Eh <- d as four SPI2 bytes, d = 1 for SCHED_TRIG
   and 0 for SCHED_UNTRIG. The chip registers the frame on the 32nd SCK
   rising edge, i.e. at the end of the last byte. */
#define SCHED_B1 0x1Cu
#define SCHED_B2 0x00u
#define SCHED_B3 0x00u
#define SCHED_B4(action) ((action) == SCHED_TRIG ? 0x02u : 0x00u)

enum { SCHED_NONE = 0, SCHED_TRIG = 1, SCHED_UNTRIG = 2 };
enum { BLOCK_OVERFLOW = 1, BLOCK_END = 2 };
enum { ACQ_IDLE = 0, ACQ_WAIT = 1, ACQ_RUN = 2, ACQ_DONE = 3 };
enum { REQ_NONE = 0, REQ_ARM = 1, REQ_ABORT = 2 };

/* One block as sent: "#6dddddd", header, samples, LF (both targets are
   little-endian, so the fields are the wire format). */
typedef struct {
  char prefix[SCHED_PREFIX];
  uint32_t seq;
  uint32_t first;
  uint16_t count;
  uint16_t flags;
  uint16_t s[SCHED_S + 1u];
} __attribute__((aligned(4))) sched_slot_t;

typedef char sched_slot_layout[offsetof(sched_slot_t, s) == SCHED_HDR ? 1 : -1];

typedef struct {
  uint32_t idx;       /* sample whose conversion starts in this call */
  uint32_t trig_next; /* sample whose ISR registers the next trigger */
  uint32_t untrig_at; /* sample whose ISR starts the trigger-clear frame */
  uint32_t gap[2];    /* alternating trigger spacing, samples */
  uint32_t period;    /* acquisition start grid, samples */
  uint32_t phase;     /* idx modulo period */
  uint32_t n;         /* samples to acquire, 0 unbounded */
  uint32_t rel;       /* acquisition index of the next stored word */
  uint32_t seq;
  uint32_t arm_n;
  uint16_t *dst;
  uint16_t pos;
  uint16_t trig_left;
  uint8_t gap_sel;
  uint8_t take;
  uint8_t ovf;
  uint8_t wr_slot;
  volatile uint8_t wr_count;
  volatile uint8_t rd_count;
  volatile uint8_t acq;
  volatile uint8_t req;
  volatile uint8_t end_pending;
} sched_t;

static inline uint8_t sched_fill(const sched_t *s) {
  return (uint8_t)(s->wr_count - s->rd_count);
}

static inline void sched_open(sched_t *s, sched_slot_t *slots) {
  sched_slot_t *b;
  if (sched_fill(s) >= SCHED_NSLOT) {
    s->ovf = 1;
    return;
  }
  b = &slots[s->wr_slot];
  b->seq = s->seq;
  b->first = s->rel;
  b->flags = s->ovf;
  s->ovf = 0;
  s->dst = b->s;
  s->pos = 0;
}

static inline void sched_close(sched_t *s, sched_slot_t *slots, uint8_t end) {
  sched_slot_t *b = &slots[s->wr_slot];
  b->count = s->pos;
  b->flags |= end;
  s->dst = 0;
  s->seq++;
  s->wr_slot = (uint8_t)(s->wr_slot + 1u == SCHED_NSLOT ? 0 : s->wr_slot + 1u);
  s->wr_count = (uint8_t)(s->wr_count + 1u);
}

/* `w` is the word of the conversion started in the previous call. Returns
   the frame whose first three bytes are clocked out now, with SEN raised
   (SCHED_TRIG or SCHED_UNTRIG), or SCHED_NONE; the board layer sends its
   last byte right after the next CNV rising edge and lowers SEN in the ISR
   after that. A trigger at sample t is therefore loaded at t - 1 and
   registered at t; its clear frame is loaded at t + 1. Triggers must be at
   least 4 samples apart. */
static inline uint8_t sched_step(sched_t *s, sched_slot_t *slots, uint16_t w) {
  uint8_t action = SCHED_NONE;
  uint32_t i = s->idx;
  if (s->take) {
    uint8_t end;
    if (!s->dst)
      sched_open(s, slots);
    s->rel++;
    end = s->n && s->rel == s->n ? BLOCK_END : 0;
    if (s->dst) {
      s->dst[s->pos++] = w;
      if (s->pos == SCHED_S || end)
        sched_close(s, slots, end);
    } else {
      s->ovf = 1;
      s->end_pending = end;
    }
    if (end) {
      s->take = 0;
      s->acq = ACQ_DONE;
    }
  }
  if (s->req) {
    if (s->req == REQ_ARM) {
      s->n = s->arm_n;
      s->rel = 0;
      s->seq = 0;
      s->ovf = 0;
      s->end_pending = 0;
      s->acq = ACQ_WAIT;
    } else {
      s->acq = ACQ_IDLE;
    }
    s->dst = 0;
    s->take = 0;
    s->req = REQ_NONE;
  }
  if (s->acq == ACQ_WAIT && s->phase == 0)
    s->acq = ACQ_RUN;
  s->take = s->acq == ACQ_RUN;
  if (i == s->untrig_at)
    action = SCHED_UNTRIG;
  if (s->trig_left) {
    if (i == s->trig_next) {
      s->untrig_at = i + 1u;
      s->trig_next += s->gap[s->gap_sel];
      s->gap_sel ^= 1u;
      if (s->trig_left != SCHED_FOREVER)
        s->trig_left--;
    } else if (i + 1u == s->trig_next) {
      action = SCHED_TRIG;
    }
  }
  s->phase = s->phase + 1u == s->period ? 0 : s->phase + 1u;
  s->idx = i + 1u;
  return action;
}

#ifdef __cplusplus
}
#endif

#endif
