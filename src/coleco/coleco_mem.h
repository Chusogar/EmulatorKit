/*
 *  ColecoVision memory map – header
 *
 *  coleco_mem.h
 *
 *  Memory layout:
 *    0x0000 – 0x1FFF   8 KB RAM
 *    0x2000 – 0x5FFF   (mirrors of RAM / unmapped, returns 0xFF)
 *    0x6000 – 0x7FFF   ROM BIOS (8 KB)
 *    0x8000 – 0xFFFF   Cartridge ROM (up to 32 KB, mirrored if smaller)
 */

#ifndef COLECO_MEM_H
#define COLECO_MEM_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Initialise RAM/ROM state.  Call before coleco_load_bios / coleco_load_cart.
 */
void coleco_mem_init(void);

/*
 * Load the ColecoVision BIOS image from <path> into the BIOS region.
 * Returns true on success.
 */
bool coleco_load_bios(const char *path);

/*
 * Load a cartridge ROM from <path>.
 * Returns true on success.
 */
bool coleco_load_cart(const char *path);

/* libz80 memory callbacks */
uint8_t coleco_mem_read(int param, uint16_t addr);
void    coleco_mem_write(int param, uint16_t addr, uint8_t val);

#endif /* COLECO_MEM_H */
