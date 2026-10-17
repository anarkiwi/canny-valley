/* Exact decimal and hex formatting without floating point. */
#ifndef RADAR_FMT_H
#define RADAR_FMT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* v / 10^decimals with exactly `decimals` fraction digits; returns length. */
size_t fmt_fixed(char *out, int64_t v, uint8_t decimals);
/* Four upper-case hex digits, no terminator. */
void fmt_hex16(char *out, uint16_t v);

#ifdef __cplusplus
}
#endif

#endif
