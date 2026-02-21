#ifndef DIVIDE_H
#define DIVIDE_H

#include <stdint.h>
#include "../ide.h"

/*
 * DivIDE / DivIDE+ memory map and control for ZX Spectrum emulator.
 *
 * divide == 0  → not present
 * divide == 1  → DivIDE  (8K ROM)
 * divide == 2  → DivIDE+ (512K ROM)
 */

typedef struct {
    uint8_t  divmem[524288]; /* 512 K RAM */
    uint8_t  divrom[524288]; /* 512 K ROM */
    uint8_t  divide_latch;
    unsigned divide_mapped;  /* 0 = not mapped, 1 = mapped */
    unsigned divide_oe;
    unsigned divide_pair;    /* latches other half of IDE word-stream */
    unsigned divide;         /* 0 = none, 1 = DivIDE, 2 = DivIDE+ */
    uint8_t  divplus_latch;
    unsigned divplus_128k;
    unsigned divplus_7ffd;
} divide_state_t;

/*
 * Context passed to divide functions that need external emulator state.
 * spectrum.c provides the callbacks so that divide.c never accesses
 * spectrum.c static variables directly.
 */
typedef struct {
    divide_state_t        *state;
    struct ide_controller *ide;
    unsigned (*get_model)(void);   /* public wrapper in spectrum.c */
    uint8_t  (*get_mlatch)(void);  /* public wrapper in spectrum.c */
} divide_ctx_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise state to safe defaults (divplus_128k = 1, rest = 0) */
void divide_init(divide_state_t *s);

/* Load ROM from path; sets s->divide to 1 (DivIDE) or 2 (DivIDE+).
 * Returns 0 on success, -1 on error. */
int  divide_load_rom(divide_state_t *s, const char *path);

/* Returns non-zero if a DivIDE variant is active */
int  divide_is_enabled(const divide_state_t *s);

/* Returns non-zero if the DivIDE page is currently mapped at 0x0000-0x3FFF */
int  divide_is_mapped(const divide_state_t *s);

/* Read / write through the DivIDE memory window (addr < 0x4000) */
uint8_t divide_mem_read(const divide_state_t *s, uint16_t addr);
void    divide_mem_write(divide_state_t *s, uint16_t addr, uint8_t val);

/* M1 hooks – call from mem_read() when cpu->M1 is set.
 *   divide_m1_pre  : before the actual RAM/ROM fetch (may set divide_mapped).
 *   divide_m1_post : after  the fetch (ROM-paging automap / unmap).          */
void divide_m1_pre(divide_ctx_t *ctx, uint16_t addr);
void divide_m1_post(divide_state_t *s, uint16_t addr);

/* I/O port handling.
 *   divide_io_read  : returns 0-255 if the port is handled, -1 otherwise.
 *   divide_io_write : handles the port if applicable.                        */
int  divide_io_read(divide_ctx_t *ctx, uint16_t addr);
void divide_io_write(divide_ctx_t *ctx, uint16_t addr, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* DIVIDE_H */
