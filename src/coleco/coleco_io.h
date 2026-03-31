/*
 *  ColecoVision I/O port dispatch – header
 *
 *  coleco_io.h
 *
 *  ColecoVision I/O port summary (all ports are 8-bit; upper address lines
 *  are not fully decoded on real hardware – the masks below reflect the
 *  conventional decoding used by most software):
 *
 *    Port (write) 0xBE  → VDP data
 *    Port (write) 0xBF  → VDP control register
 *    Port (read)  0xBE  → VDP data
 *    Port (read)  0xBF  → VDP status
 *    Port (write) 0xFF  → PSG (SN76489) data
 *    Port (read)  0xFC  → Controller port 1
 *    Port (read)  0xFF  → Controller port 2
 *
 *  NOTE: The exact port decoding may require adjustment once real-world ROM
 *        behaviour is tested.
 */

#ifndef COLECO_IO_H
#define COLECO_IO_H

#include <stdint.h>

/* libz80 I/O callbacks */
uint8_t coleco_io_read(int param, uint16_t port);
void    coleco_io_write(int param, uint16_t port, uint8_t val);

#endif /* COLECO_IO_H */
