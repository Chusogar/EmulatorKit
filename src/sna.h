#ifndef SNA_H
#define SNA_H

#include <stdint.h>
#include <stdbool.h>
#include "../libz80/z80.h"

/*
 * Context passed to load_sna() containing all external state it needs.
 * SNA format refs:
 *   https://sinclair.wiki.zxnet.co.uk/wiki/SNA_format
 *   https://worldofspectrum.net/zx-modules/fileformats/snaformat.html
 */
typedef struct {
    Z80Context  *cpu;
    uint8_t    (*ram)[16384];           /* ram[16][16384] */
    uint8_t     *border_color;
    uint8_t     *mlatch;
    void       (*mem_write)(int, uint16_t, uint8_t);
    uint8_t    (*mem_read)(int, uint16_t);
    void       (*recalc_mmu)(void);
} sna_context_t;

#ifdef __cplusplus
extern "C" {
#endif

bool load_sna(const char *filename, const sna_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* SNA_H */
