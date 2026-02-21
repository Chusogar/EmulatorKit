/*
 * divide.c – DivIDE / DivIDE+ memory map and control for ZX Spectrum emulator.
 *
 * Extracted from spectrum.c so that the divide logic lives in its own module.
 * All external emulator state is accessed through divide_ctx_t callbacks;
 * no static variables from spectrum.c are referenced directly.
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "divide.h"

/* Model constants – must match the definitions in spectrum.c */
#define ZX_48K_2    0
#define ZX_48K_3    1
#define ZX_128K     2
#define ZX_PLUS3    3

/* ── Internal helpers ─────────────────────────────────────────────────────── */

static uint8_t *divbank(divide_state_t *s, unsigned bank, unsigned page, unsigned off)
{
    bank <<= 2;
    bank |= page;
    return &s->divmem[bank * 0x2000 + (off & 0x1FFF)];
}

static uint8_t *divide_getmap(divide_state_t *s, unsigned addr, unsigned w)
{
    unsigned bank = 0;
    if (s->divide == 2) {
        switch (s->divplus_latch & 0xC0) {
        case 0x00:  /* DivIDE mode */
            bank = (s->divplus_latch >> 1) & 0x0F;
            break;
        case 0x40:
            if (w & (s->divplus_latch & 0x20))
                return NULL;
            return &s->divmem[((s->divplus_latch & 0x1F) << 14) + (addr & 0x3FFF)];
        case 0x80:
            if (w)
                return NULL;
            return &s->divrom[((s->divplus_latch & 0x1F) << 14) + (addr & 0x3FFF)];
        }
    }
    /* TODO mapmem should probably stop RAM 3 writes without CONMEM
       even if 2000-3FFF */
    if (addr & 0x2000)
        return divbank(s, bank, s->divide_latch & 3, addr);
    /* CONMEM */
    if (s->divide_latch & 0x80) {
        if (w)
            return NULL;
        if (s->divide == 2)
            return s->divrom + bank * 0x8000 + 0x6000 + (addr & 0x1FFF);
        else
            return s->divrom + (addr & 0x1FFF);
    }
    /* MAPMEM */
    if (s->divide_latch & 0x40) {
        if (w)
            return NULL;
        return divbank(s, bank, 3, addr);
    }
    if (s->divide == 2)
        return s->divrom + bank * 0x8000 + 0x6000 + (addr & 0x1FFF);
    else
        return s->divrom + (addr & 0x1FFF);
}

static void divplus_ctrl(divide_state_t *s, uint8_t val)
{
    s->divplus_latch = val;
    switch (val & 0xE0) {
    case 0xC0:
    case 0xE0:
        s->divide_latch   = 0;
        s->divplus_latch  = 0;
        s->divplus_7ffd   = 0;
        s->divide_mapped  = 0;
        break;
    case 0x00:
        /* DivIDE mode – bits 4-1 select the extended banking */
        break;
    case 0x20:
        /* Enable 128K mode */
        s->divplus_128k = 1;
        break;
        /* RAM mode: 10WAAAAA */
        /* 32K pages x 16K replace Spectrum ROM, DivIDE traps off */
    case 0x40:
    case 0x60:
        break;
    case 0x80:
        /* ROM mode: as above for 16K ROM banks */
    case 0xA0:
        break;
    }
}

/* ── Public API ───────────────────────────────────────────────────────────── */

void divide_init(divide_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->divplus_128k = 1;
}

int divide_load_rom(divide_state_t *s, const char *path)
{
    int fd = open(path, O_RDONLY);
    if (fd == -1) {
        perror(path);
        return -1;
    }
    ssize_t l = read(fd, s->divrom, sizeof(s->divrom));
    close(fd);

    if (l == 8192) {
        s->divide = 1;
    } else if (l == 524288) {
        s->divide = 2;
    } else {
        fprintf(stderr, "[DivIDE] ROM size invalid: expected 8192 or 524288 bytes.\n");
        return -1;
    }
    return 0;
}

int divide_is_enabled(const divide_state_t *s)
{
    return s->divide != 0;
}

int divide_is_mapped(const divide_state_t *s)
{
    return s->divide_mapped == 1;
}

uint8_t divide_mem_read(const divide_state_t *s, uint16_t addr)
{
    /* divide_getmap modifies state only for write; cast away const for read */
    return *divide_getmap((divide_state_t *)s, addr, 0);
}

void divide_mem_write(divide_state_t *s, uint16_t addr, uint8_t val)
{
    uint8_t *p = divide_getmap(s, addr, 1);
    if (p)
        *p = val;
}

void divide_m1_pre(divide_ctx_t *ctx, uint16_t addr)
{
    divide_state_t *s = ctx->state;

    if (s->divplus_latch & 0xC0)
        return;

    /* Immediate map on access to 0x3D00-0x3DFF */
    if (s->divide == 1 && addr >= 0x3D00 && addr <= 0x3DFF) {
        s->divide_mapped = 1;
        return;
    }
    if (s->divide == 2) {
        unsigned model   = ctx->get_model();
        uint8_t  mlatch  = ctx->get_mlatch();
        if ((model <= ZX_48K_3 || !s->divplus_128k || (mlatch & 0x10)) &&
            addr >= 0x3D00 && addr <= 0x3DFF) {
            s->divide_mapped = 1;
        }
    }
}

void divide_m1_post(divide_state_t *s, uint16_t addr)
{
    if (s->divplus_latch & 0xC0)
        return;

    /* ROM paging: unmap on 0x1FF8-0x1FFF */
    if (s->divide && addr >= 0x1FF8 && addr <= 0x1FFF) {
        s->divide_mapped = 0;
        return;
    }
    /* Automap on interrupt / restart vectors */
    if (s->divide && (addr == 0x0000 || addr == 0x0008 || addr == 0x0038 ||
                      addr == 0x0066 || addr == 0x04C6 || addr == 0x0562)) {
        s->divide_mapped = 1;
    }
}

int divide_io_read(divide_ctx_t *ctx, uint16_t addr)
{
    divide_state_t *s = ctx->state;

    if (!s->divide)
        return -1;

    if ((addr & 0xE3) == 0xA3) {
        unsigned r = (addr >> 2) & 0x07;
        if (r) {
            s->divide_oe = 1;  /* Odd mode */
            return (int)(uint8_t)ide_read16(ctx->ide, r);
        }
        if (s->divide_oe == 0) {
            s->divide_oe = 1;
            return (int)s->divide_pair;
        }
        unsigned rv = ide_read16(ctx->ide, 0);
        s->divide_pair = rv >> 8;
        s->divide_oe = 0;
        return (int)(uint8_t)(rv & 0xFF);
    }
    return -1;
}

void divide_io_write(divide_ctx_t *ctx, uint16_t addr, uint8_t val)
{
    divide_state_t *s = ctx->state;

    if (!s->divide)
        return;

    if ((addr & 0xE3) == 0xA3) {
        uint8_t r = (addr >> 2) & 0x07;
        if (r) {
            s->divide_oe = 1;  /* Odd mode */
            ide_write16(ctx->ide, r, val);
        } else if (s->divide_oe == 1) {
            s->divide_oe = 0;
            s->divide_pair = val;
        } else {
            ide_write16(ctx->ide, 0, s->divide_pair | (val << 8));
            s->divide_oe = 0;
        }
    }
    if ((addr & 0xE3) == 0xE3) {
        /* MAPRAM cannot be cleared */
        val |= s->divide_latch & 0x40;
        s->divide_latch = val;
        if (val & 0x80)
            s->divide_mapped = 1;
    }
    if (s->divide == 2 && (addr & 0xFF) == 0x17)
        divplus_ctrl(s, val);
}
