#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "ym2149.h"

struct ym2149 {
	unsigned dummy;
};

struct ym2149 *ym2149_create(uint16_t clk)
{
    struct ym2149 *sn = malloc(sizeof(struct ym2149));
    if (sn == NULL)
    {
        fprintf(stderr, "Out of memory\n");
        exit(1);
    }

    return sn;
}

void ym2149_write(struct ym2149 *sn, uint8_t reg, uint8_t val)
{
}

void ym2149_destroy(struct ym2149 *sn)
{
    free(sn);
}
