#include "fmt.h"

size_t fmt_fixed(char *out, int64_t v, uint8_t decimals) {
  char tmp[24];
  size_t n = 0, len = 0;
  uint64_t u = v < 0 ? (uint64_t)-v : (uint64_t)v;
  do {
    tmp[n++] = (char)('0' + u % 10u);
    u /= 10u;
  } while (u || n <= decimals);
  if (v < 0)
    out[len++] = '-';
  while (n) {
    out[len++] = tmp[--n];
    if (n == decimals && decimals)
      out[len++] = '.';
  }
  out[len] = 0;
  return len;
}

static const char hex[] = "0123456789ABCDEF";

void fmt_hex16(char *out, uint16_t v) {
  out[0] = hex[v >> 12];
  out[1] = hex[(v >> 8) & 15u];
  out[2] = hex[(v >> 4) & 15u];
  out[3] = hex[v & 15u];
}
