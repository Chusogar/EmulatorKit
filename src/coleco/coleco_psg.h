/*
 *  ColecoVision PSG skeleton (SN76489) – header
 *
 *  coleco_psg.h
 *
 *  The SN76489 is write-only; its register is selected by the high nibble
 *  of the first byte written.  This module wraps the sn76489 driver already
 *  present in the repository.
 *
 *  ColecoVision PSG I/O port: writes to 0xFF.
 */

#ifndef COLECO_PSG_H
#define COLECO_PSG_H

#include <stdint.h>
#include <stdbool.h>

/* Initialise the PSG.  Returns true on success. */
bool coleco_psg_init(void);

/* Release PSG resources. */
void coleco_psg_shutdown(void);

/* Reset the PSG to a silent state. */
void coleco_psg_reset(void);

/* Write a byte to the PSG (port 0xFF on ColecoVision). */
void coleco_psg_write(uint8_t val);

#endif /* COLECO_PSG_H */
