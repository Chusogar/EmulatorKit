/*
 *  ColecoVision memory map implementation
 *
 *  coleco_mem.c
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "coleco_mem.h"

#define RAM_SIZE    0x2000u     /* 8 KB */
#define BIOS_SIZE   0x2000u     /* 8 KB */
#define CART_SIZE   0x8000u     /* 32 KB max */

static uint8_t ram[RAM_SIZE];
static uint8_t bios[BIOS_SIZE];
static uint8_t cart[CART_SIZE];
static size_t  cart_len;       /* actual bytes loaded */
static int     bios_loaded;
static int     cart_loaded;

void coleco_mem_init(void)
{
    memset(ram,  0x00, sizeof(ram));
    memset(bios, 0xFF, sizeof(bios));
    memset(cart, 0xFF, sizeof(cart));
    bios_loaded = 0;
    cart_loaded = 0;
    cart_len    = 0;
}

bool coleco_load_bios(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "coleco: cannot open BIOS '%s'\n", path);
        return false;
    }
    size_t n = fread(bios, 1, BIOS_SIZE, f);
    fclose(f);
    if (n == 0) {
        fprintf(stderr, "coleco: BIOS file empty or unreadable\n");
        return false;
    }
    bios_loaded = 1;
    fprintf(stderr, "coleco: loaded %zu bytes of BIOS\n", n);
    return true;
}

bool coleco_load_cart(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "coleco: cannot open cartridge '%s'\n", path);
        return false;
    }
    cart_len = fread(cart, 1, CART_SIZE, f);
    fclose(f);
    if (cart_len == 0) {
        fprintf(stderr, "coleco: cartridge file empty or unreadable\n");
        return false;
    }
    cart_loaded = 1;
    fprintf(stderr, "coleco: loaded %zu bytes of cartridge ROM\n", cart_len);
    return true;
}

uint8_t coleco_mem_read(int param, uint16_t addr)
{
    (void)param;

    if (addr < 0x2000u)                     /* 0x0000–0x1FFF : 8 KB RAM */
        return ram[addr];

    if (addr < 0x6000u)                     /* 0x2000–0x5FFF : unmapped */
        return 0xFF;

    if (addr < 0x8000u)                     /* 0x6000–0x7FFF : BIOS */
        return bios_loaded ? bios[addr - 0x6000u] : 0xFF;

    /* 0x8000–0xFFFF : cartridge ROM (mirror if smaller than 32 KB) */
    if (cart_loaded && cart_len > 0)
        return cart[(addr - 0x8000u) % cart_len];

    return 0xFF;
}

void coleco_mem_write(int param, uint16_t addr, uint8_t val)
{
    (void)param;

    if (addr < 0x2000u) {                   /* RAM is writable */
        ram[addr] = val;
        return;
    }
    /* All other regions are read-only; writes are silently ignored */
}
