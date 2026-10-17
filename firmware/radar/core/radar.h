/* Portable radar controller core: SCPI command set, HMC703 programming,
   sample/trigger scheduling and block framing. The board layer (pic/) calls
   the functions below; the core calls hal.h. */
#ifndef RADAR_RADAR_H
#define RADAR_RADAR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "isr_sched.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RADAR_COMMIT
#define RADAR_COMMIT "unknown"
#endif
#define RADAR_FIRMWARE "CV1.0.0-" RADAR_COMMIT

/* Program the synthesiser (RF off), load the serial number and run the
   sample-clock self-test. */
void radar_init(void);

/* Main-loop work: acquisition completion, RF auto-mute. */
void radar_poll(void);

/* Bulk-OUT payload of a USBTMC DEV_DEP_MSG_OUT; `eom` on its last part. */
void radar_rx(const uint8_t *data, size_t len, bool eom);

/* Bulk-IN request for up to `max` bytes: returns the bytes available now
   (0 means defer: ask again after radar_poll), with *eom set on the last
   part of the response. The data stays valid until radar_tx_done. */
size_t radar_tx_peek(const uint8_t **data, uint32_t max, bool *eom);
void radar_tx_done(size_t len);

/* USBTMC device clear / bulk abort: drop pending output and acquisitions. */
void radar_clear(void);

/* IEEE 488.2 status byte for USB488 READ_STATUS_BYTE. */
uint8_t radar_stb(void);

/* Four-digit serial number, valid after radar_init. */
const char *radar_serial(void);

extern sched_t radar_sched;
extern sched_slot_t radar_slots[SCHED_NSLOT];

/* Sample ISR body: `w` is the ADC word of the previous conversion; returns
   the SPI2 frame to load (SCHED_TRIG / SCHED_UNTRIG) or SCHED_NONE. */
static inline uint8_t radar_isr_step(uint16_t w) {
  return sched_step(&radar_sched, radar_slots, w);
}

#ifdef __cplusplus
}
#endif

#endif
