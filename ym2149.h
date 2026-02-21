#ifndef YM2149_H
#define YM2149_H

#include <stdint.h>

struct ym2149;
extern struct ym2149 *ym2149_create(uint16_t);
extern void ym2149_write(struct ym2149 *, uint8_t, uint8_t);
extern void ym2149_destroy(struct ym2149 *);

#endif
