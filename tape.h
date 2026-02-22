#ifndef TAPE_H
#define TAPE_H

#include <stdint.h>
#include <stdbool.h>
#include "libz80/z80.h"

/* T-state clock rate (PAL ~3.5469 MHz; tolerable for 48K too) */
#define TSTATES_CPU     3546900.0

/* Timing constants for TAP pulse generation (t-states) */
#define T_PILOT         2168
#define T_SYNC1          667
#define T_SYNC2          735
#define T_BIT0           855
#define T_BIT1          1710
#define PILOT_HDR       8063
#define PILOT_DATA      3223
#define T_PAUSE_MS      1000
#define T_MS(ms)        ((uint64_t)((ms) * (TSTATES_CPU / 1000.0)))

typedef struct {
    uint16_t len;      /* bytes: flag + payload + checksum */
    uint8_t *data;
} tap_block_t;

typedef enum {
    TP_IDLE = 0, TP_PILOT, TP_SYNC1, TP_SYNC2, TP_BITS,
    TP_PAUSE, TP_NEXTBLOCK, TP_DONE
} tape_phase_t;

typedef struct {
    tap_block_t *blk;
    int nblk;

    int i_blk;
    int i_byte;
    int bit_mask;
    int subpulse;

    tape_phase_t phase;
    uint64_t next_edge_at;
    uint64_t pause_end_at;
    int pilot_left;

    int ear_level;            /* 0/1: señal en la entrada EAR */
    uint64_t frame_origin;
    uint64_t slice_origin;
    int playing;              /* 1=PLAY, 0=PAUSE */
} tape_player_t;

/* Memory-write callback type used by the fast TAP loader */
typedef void (*tape_mem_write_fn)(int, uint16_t, uint8_t);

#ifdef __cplusplus
extern "C" {
#endif

/* ── Pulse player API ── */
int     tape_load_tap_pulses(tape_player_t *t, const char *path);
void    tape_free(tape_player_t *t);
void    tape_begin_slice(tape_player_t *t);
void    tape_end_slice(tape_player_t *t, const Z80Context *cpu);
int     tape_active(const tape_player_t *t);
uint8_t tape_ear_bit6(const tape_player_t *t);

/* Optional callback invoked just before each EAR level transition.
 * t_abs is the absolute t-state of the edge; new_level is 0 or 1. */
typedef void (*tape_ear_notify_fn)(uint64_t t_abs, int new_level);
void tape_set_ear_notify(tape_ear_notify_fn fn);

/* ── Fast TAP loader API ── */
void tap_list(const char *path);
bool load_tap_fast(const char *path, int auto_start,
                   Z80Context *cpu, tape_mem_write_fn mem_write);

#ifdef __cplusplus
}
#endif

#endif /* TAPE_H */