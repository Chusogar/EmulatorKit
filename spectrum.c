/*
 *  Very basic ZX Spectrum set up for debugging stuff. This does not
 *  do all the timing related magic required to run games correctly
 *  with effects and stuff.
 *
 *  TODO: ZXCF, Simple CF cards
 *
 *  Additions:
 *   - SNA loader (48K & 128K) fixed
 *   - Kempston joystick on port 0x1F (arrow keys + Ctrl/Space/Enter = FIRE)
 *   - TAP fast loader: injects CODE/SCREEN$ blocks to param1 address, no EAR/timing
 *   - TAP pulse player (ROM-accurate): pilot/sync/bits/pauses on EAR input
 *   - Beeper (EAR|MIC) audio via SDL2 (queue mode)
 *   - Hotkeys: F6 (reload TAP & autostart fast), F7 (list TAP),
 *              F8 (Play/Pause tape pulses), F9 (Rewind tape pulses)
 *
 *  TAP format refs:
 *    - https://sinclair.wiki.zxnet.co.uk/wiki/TAP_format
 *    - https://sinclair.wiki.zxnet.co.uk/wiki/Spectrum_tape_interface
 *  SNA format refs:
 *    - https://sinclair.wiki.zxnet.co.uk/wiki/SNA_format
 *    - https://worldofspectrum.net/zx-modules/fileformats/snaformat.html
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/stat.h>
#include "libz80/z80.h"
#include "z80dis.h"
#include "ide.h"
#include "lib765/include/765.h"

#include <SDL2/SDL.h>
#include "event.h"
#include "keymatrix.h"

static SDL_Window *window;
static SDL_Renderer *render;
static SDL_Texture *texture;

#define BORDER  32
#define WIDTH   (256 + 2 * BORDER)
#define HEIGHT  (192 + 2 * BORDER)

static uint32_t texturebits[WIDTH * HEIGHT];

uint8_t border_color   = 7;

static uint32_t palette[16] = {
    0xFF000000, 0xFF0000D8, 0xFFD80000, 0xFFD800D8,
    0xFF00D800, 0xFF00D8D8, 0xFFD8D800, 0xFFD8D8D8,
    0xFF000000, 0xFF0000FF, 0xFFFF0000, 0xFFFF00FF,
    0xFF00FF00, 0xFF00FFFF, 0xFFFFFF00, 0xFFFFFFFF
};

/* Keep all the memory in one map for now. 128K will need to do a bit
   more work later */
static uint8_t ram[16][16384];
#define ROM(x)  (x)
#define RAM(x)  ((x) + 8)

static struct keymatrix *matrix;
static Z80Context cpu_z80;
static FDC_PTR fdc;
static FDRV_PTR drive_a, drive_b;
static struct ide_controller *ide;

//static int tape = -1;          /* Tape file handle (unused by fast TAP) */
static unsigned mem = 48;      /* First byte above RAM (defaults to 48K) */
static uint8_t ula;            /* ULA state */
static uint8_t frames;         /* Flash counter */
static uint8_t mlatch;
static uint8_t p3latch;
static unsigned map[4] = { ROM(0), RAM(5), RAM(2), RAM(0) };
static unsigned vram = RAM(5);

static unsigned drawline;      /* If rasterising */
static unsigned blanked;       /* True if blanked */

static uint8_t divmem[524288];/* Full 512K emulated */
static uint8_t divrom[524288];
static uint8_t divide_latch;
static unsigned divide_mapped; /* 0 no, 1 yes */
static unsigned divide_oe;
static unsigned divide_pair;   /* Latches other half of wordstream for IDE */
static unsigned divide;

static uint8_t divplus_latch;
static unsigned divplus_128k = 1;
static unsigned divplus_7ffd;

#define ZX_48K_2    0
#define ZX_48K_3    1
#define ZX_128K     2
#define ZX_PLUS3    3
static unsigned model = ZX_48K_3;

static volatile int emulator_done;
static unsigned fast;
static unsigned int_recalc;
/* static unsigned live_irq; */

#define TRACE_MEM   1
#define TRACE_IO    2
#define TRACE_IRQ   4
#define TRACE_KEY   8
#define TRACE_CPU   16
#define TRACE_FDC   32

static int trace = 0;

static void reti_event(void);

/* ─────────────────────────────────────────────────────────────
 * Beeper (EAR/MIC) + SDL2 audio queue (mono, S16)
 * ───────────────────────────────────────────────────────────── */
#define TSTATES_CPU 3546900.0 /* PAL 128K ~3.5469 MHz; 48K ~3.5 MHz, diferencia tolerable */

static SDL_AudioDeviceID audio_dev = 0;
static SDL_AudioSpec have;
static int audio_rate = 44100;
static float beeper_volume = 0.30f;

static uint64_t beeper_frame_origin = 0;
static uint64_t beeper_slice_origin = 0;
static uint64_t beeper_last_tstate  = 0;
static int      beeper_level        = 0;   /* 0 o 1 (onda cuadrada) */
static double   beeper_frac_acc     = 0.0;

static int audio_init_sdl(int rate)
{
    SDL_AudioSpec want;
    SDL_zero(want);
    want.freq = rate;
    want.format = AUDIO_S16SYS;
    want.channels = 1;
    want.samples = 2048;

    audio_dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (!audio_dev) {
        fprintf(stderr, "SDL audio: unable to open: %s\n", SDL_GetError());
        return -1;
    }
    audio_rate = have.freq;
    SDL_PauseAudioDevice(audio_dev, 0);
    return 0;
}

static inline void beeper_advance_to(uint64_t t_now)
{
    if (!audio_dev) return;
    if (t_now <= beeper_last_tstate) return;

    uint64_t dt = t_now - beeper_last_tstate;
    beeper_last_tstate = t_now;

    double add = (double)dt * (double)audio_rate / TSTATES_CPU;
    beeper_frac_acc += add;
    int nsamp = (int)beeper_frac_acc;
    if (nsamp <= 0) return;
    beeper_frac_acc -= nsamp;

    int16_t val = beeper_level ? (int16_t)(+beeper_volume * 32767.0f)
                               : (int16_t)(-beeper_volume * 32767.0f);
    enum { CHUNK = 4096 };
    static int16_t buf[CHUNK];
    while (nsamp > 0) {
        int n = (nsamp > CHUNK) ? CHUNK : nsamp;
        for (int i = 0; i < n; ++i) buf[i] = val;
        SDL_QueueAudio(audio_dev, buf, n * (int)sizeof(int16_t));
        nsamp -= n;
    }
}

static inline void beeper_begin_slice(void)
{
    beeper_slice_origin = beeper_frame_origin;
}

static inline void beeper_end_slice(void)
{
    uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
    beeper_advance_to(t_now);
    beeper_frame_origin = t_now;
}

static inline void beeper_set_level(int level_now)
{
    uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
    beeper_advance_to(t_now);
    beeper_level = level_now ? 1 : 0;
}

/* EAR=b4, MIC=b3 → modelado sencillo como OR */
static inline void beeper_set_from_ula(uint8_t v)
{
    int level = ((v >> 4) & 1) | ((v >> 3) & 1);
    beeper_set_level(level);
}

/* ─────────────────────────────────────────────────────────────
 * Kempston joystick (cursores + fire)
 * Puerto 0x1F (31 dec). Bits activos en alto:
 * bit0 Right, bit1 Left, bit2 Down, bit3 Up, bit4 Fire
 * ───────────────────────────────────────────────────────────── */
static inline uint8_t kempston_state_from_sdl(void)
{
    const Uint8 *ks = SDL_GetKeyboardState(NULL);
    uint8_t v = 0;

    if (ks[SDL_SCANCODE_RIGHT]) v |= 0x01; // Right
    if (ks[SDL_SCANCODE_LEFT])  v |= 0x02; // Left
    if (ks[SDL_SCANCODE_DOWN])  v |= 0x04; // Down
    if (ks[SDL_SCANCODE_UP])    v |= 0x08; // Up

    // Fire: teclas cómodas
    if (ks[SDL_SCANCODE_LCTRL] || ks[SDL_SCANCODE_RCTRL] ||
        ks[SDL_SCANCODE_SPACE] || ks[SDL_SCANCODE_RETURN]) {
        v |= 0x10;
    }
    // Bits 5..7 no usados → 0
    return v;
}

static uint8_t *divbank(unsigned bank, unsigned page, unsigned off)
{
    bank <<= 2;
    bank |= page;
    return &divmem[bank * 0x2000 + (off & 0x1FFF)];
}

static uint8_t *divide_getmap(unsigned addr, unsigned w)
{
    unsigned bank = 0;
    if (divide == 2) {
        switch(divplus_latch & 0xC0) {
        case 0x00:  /* DivIDE mode */
            bank = (divplus_latch >> 1) & 0x0F;
            break;
        case 0x40:
            if (w & (divplus_latch & 0x20))
                return NULL;
            return &divmem[((divplus_latch & 0x1F) << 14) +
                (addr & 0x3FFF)];
        case 0x80:
            if (w)
                return NULL;
            return &divrom[((divplus_latch & 0x1F) << 14) +
                (addr & 0x3FFF)];
        }
    }
    /* TODO mapmem should probably stop RAM 3 writes without CONMEM
       even if 2000-3FFF */
    if (addr & 0x2000)
        return divbank(bank, divide_latch & 3, addr);
    /* CONMEM */
    if (divide_latch & 0x80) {
        if (w)
            return NULL;
        if (divide == 2)
            return divrom + bank * 0x8000 + 0x6000 + (addr & 0x1FFF);
        else
            return divrom + (addr & 0x1FFF);
    }
    /* MAPMEM */
    if (divide_latch & 0x40) {
        if (w)
            return NULL;
        return divbank(bank, 3, addr);
    }
    if (divide == 2)
        return divrom + bank * 0x8000 + 0x6000 + (addr & 0x1FFF);
    else
        return divrom + (addr & 0x1FFF);
}

static void divide_write(uint16_t addr, uint8_t val)
{
    uint8_t *p = divide_getmap(addr, 1);
    if (p)
        *p = val;
}

static uint8_t divide_read(uint16_t addr)
{
    return *divide_getmap(addr, 0);
}

/* TODO: memory contention */
static uint8_t do_mem_read(uint16_t addr, unsigned debug)
{
    unsigned bank = map[addr >> 14];
    /* For now until banked stuff */
    if (addr >= mem)
        return 0xFF;
    if (addr < 0x4000 && divide_mapped == 1)
        return divide_read(addr);
    return ram[bank][addr & 0x3FFF];
}

static void mem_write(int unused, uint16_t addr, uint8_t val)
{
    unsigned bank = map[addr >> 14];
    if (addr >= mem)
        return;
    if (addr < 0x4000 && divide_mapped == 1) {
        divide_write(addr, val);
        return;
    }
    /* ROM is read only */
    if (bank >= RAM(0))
        ram[bank][addr & 0x3FFF] = val;
}

static uint8_t mem_read(int unused, uint16_t addr)
{
    static uint8_t rstate;
    uint8_t r;

    /* DivIDE+ modes other than 00 don't autopage */
    if (cpu_z80.M1 && !(divplus_latch & 0xC0)) {
        /* Immediate map */
        if (divide == 1 && addr >= 0x3D00 && addr <= 0x3DFF)
            divide_mapped = 1;
        /* TODO: correct this based on the B4 latch and 128K flag */
        if (divide == 2 && (model <= ZX_48K_3 || !divplus_128k || (mlatch & 0x10)) && addr >= 0x3D00 && addr <= 0x3DFF)
            divide_mapped = 1;
    }

    r = do_mem_read(addr, 0);

    /* Look for ED with M1, followed directly by 4D and if so trigger
       the interrupt chain */
    if (cpu_z80.M1) {
        if (!(divplus_latch & 0xC0)) {
            /* ROM paging logic */
            if (divide && addr >= 0x1FF8 && addr <= 0x1FFF)
                divide_mapped = 0;
            if (divide && (addr == 0x0000 || addr == 0x0008 || addr == 0x0038 ||
                addr == 0x0066 || addr == 0x04C6 || addr == 0x0562))
                divide_mapped = 1;
        }
        /* DD FD CB see the Z80 interrupt manual */
        if (r == 0xDD || r == 0xFD || r == 0xCB) {
            rstate = 2;
            return r;
        }
        /* Look for ED with M1, followed directly by 4D and if so trigger
           the interrupt chain */
        if (r == 0xED && rstate == 0) {
            rstate = 1;
            return r;
        }
    }
    if (r == 0x4D && rstate == 1)
        reti_event();
    rstate = 0;
    return r;
}

static void recalc_mmu(void)
{
    map[3] = RAM(mlatch & 7);
    if (mlatch & 0x08)
        vram = RAM(7);
    else
        vram = RAM(5);
    if (model == ZX_128K) {
        if (mlatch & 0x10)
            map[0] = ROM(1);
        else
            map[0] = ROM(0);
    }
    if (model == ZX_PLUS3) {
        unsigned rom = (mlatch & 0x10) ? 1 : 0;
        if (p3latch & 0x04)
            rom |= 2;
        map[0] = ROM(rom);
        switch(p3latch & 0x07) {
        case 1:
            map[0] = RAM(0);
            map[1] = RAM(1);
            map[2] = RAM(2);
            map[3] = RAM(3);
            break;
        case 3:
            map[0] = RAM(4);
            map[1] = RAM(5);
            map[2] = RAM(6);
            map[3] = RAM(7);
            break;
        case 5:
            map[0] = RAM(4);
            map[1] = RAM(5);
            map[2] = RAM(6);
            map[3] = RAM(3);
            break;
        case 7:
            map[0] = RAM(4);
            map[1] = RAM(7);
            map[2] = RAM(6);
            map[3] = RAM(3);
            break;
        }
    }
}

static void repaint_border(unsigned colour)
{
    border_color   = colour;

    uint32_t *p = texturebits;
    unsigned x,y;
    uint32_t border = palette[colour];

    for(y = 0; y < BORDER; y++)
        for(x = 0; x < WIDTH; x++)
            *p++ = border;
    for(y = BORDER; y < BORDER + 192; y++) {
        for(x = 0; x < BORDER; x++)
            *p++ = border;
        p += 256;
        for(x = 0; x < BORDER; x++)
            *p++ = border;
    }
    for(y = 0; y < BORDER; y++)
        for (x = 0; x < WIDTH; x++)
            *p++ = border;
}

static void fdc_log(int debuglevel, char *fmt, va_list ap)
{
    if ((trace & TRACE_FDC) || debuglevel == 0) {
        fprintf(stderr, "fdc: ");
        vfprintf(stderr, fmt, ap);
    }
}

/* ─────────────────────────────────────────────────────────────
 * TAP por pulsos (ROM estándar) - Motor de cinta
 * ───────────────────────────────────────────────────────────── */
#define T_PILOT        2168
#define T_SYNC1         667
#define T_SYNC2         735
#define T_BIT0          855
#define T_BIT1         1710
#define PILOT_HDR     8063
#define PILOT_DATA    3223
#define T_PAUSE_MS    1000
#define T_MS(ms)      ((uint64_t)((ms) * (TSTATES_CPU / 1000.0)))

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

static tape_player_t tape = {0};

static inline int tape_active(void) { return tape.playing && tape.phase != TP_DONE; }
static inline uint8_t tape_ear_bit6(void) { return (tape_active() && tape.ear_level) ? 0x40 : 0x00; }

/* Pequeño lector LE16 local para TAP pulses */
static inline uint16_t tape_rd_le16(FILE* f)
{
    int lo = fgetc(f), hi = fgetc(f);
    if (lo == EOF || hi == EOF) return 0;
    return (uint16_t)(lo | (hi << 8));
}

static void tape_free(void)
{
    if (tape.blk) {
        for (int i = 0; i < tape.nblk; ++i) free(tape.blk[i].data);
        free(tape.blk);
        tape.blk = NULL; tape.nblk = 0;
    }
}

/* Carga TAP en memoria para reproducción por pulsos (ROM estándar) */
static int tape_load_tap_pulses(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (!f) { perror(path); return -1; }

    tape_free();

    int count = 0;
    while (1) {
        uint16_t blen = tape_rd_le16(f);
        if (feof(f)) break;
        if (blen == 0 && ferror(f)) { perror("[TAP]"); fclose(f); return -1; }
        fseek(f, blen, SEEK_CUR);
        count++;
    }
    if (count == 0) { fclose(f); fprintf(stderr, "[TAP] vacío.\n"); return -1; }

    tape.blk = (tap_block_t*)calloc(count, sizeof(tap_block_t));
    tape.nblk = count;

    fseek(f, 0, SEEK_SET);
    for (int i = 0; i < count; ++i) {
        uint16_t blen = tape_rd_le16(f);
        uint8_t* buf = (uint8_t*)malloc(blen);
        if (!buf || fread(buf, 1, blen, f) != blen) {
            fclose(f); tape_free(); return -1;
        }
        tape.blk[i].len  = blen;
        tape.blk[i].data = buf;
    }
    fclose(f);

    tape.i_blk = 0; tape.i_byte = 0; tape.bit_mask = 0x80; tape.subpulse = 0;
    tape.phase = TP_NEXTBLOCK; tape.ear_level = 1;
    tape.frame_origin = tape.slice_origin = 0;
    tape.next_edge_at = 0; tape.pause_end_at = 0;
    tape.playing = 1;
    fprintf(stdout, "TAP (pulsos) cargado: %s (%d bloques)\n", path, tape.nblk);
    return 0;
}

static inline void tape_begin_slice(void)
{
    tape.slice_origin = tape.frame_origin;
}

static void tape_advance_to(uint64_t t_now)
{
    if (!tape_active()) return;

    while (1) {
        switch (tape.phase) {
        case TP_NEXTBLOCK: {
            if (tape.i_blk >= tape.nblk) { tape.phase = TP_DONE; return; }
            uint8_t flag = tape.blk[tape.i_blk].data[0];
            tape.pilot_left = (flag == 0x00) ? PILOT_HDR : PILOT_DATA;
            tape.phase = TP_PILOT;
            tape.next_edge_at = t_now + T_PILOT;
            tape.ear_level ^= 1;
            tape.i_byte = 0; tape.bit_mask = 0x80; tape.subpulse = 0;
            break;
        }
        case TP_PILOT:
            if (t_now < tape.next_edge_at) return;
            tape.ear_level ^= 1;
            if (--tape.pilot_left > 0) {
                tape.next_edge_at += T_PILOT;
            } else {
                tape.phase = TP_SYNC1;
                tape.next_edge_at += T_SYNC1;
            }
            break;

        case TP_SYNC1:
            if (t_now < tape.next_edge_at) return;
            tape.ear_level ^= 1;
            tape.phase = TP_SYNC2;
            tape.next_edge_at += T_SYNC2;
            break;

        case TP_SYNC2:
            if (t_now < tape.next_edge_at) return;
            tape.ear_level ^= 1;
            tape.phase = TP_BITS;
            tape.i_byte = 0; tape.bit_mask = 0x80; tape.subpulse = 0;
            {
                uint8_t b = tape.blk[tape.i_blk].data[tape.i_byte];
                int bit = (b & tape.bit_mask) ? 1 : 0;
                tape.next_edge_at += bit ? T_BIT1 : T_BIT0;
            }
            break;

        case TP_BITS: {
            if (t_now < tape.next_edge_at) return;
            tape.ear_level ^= 1;
            uint8_t b = tape.blk[tape.i_blk].data[tape.i_byte];
            int bit = (b & tape.bit_mask) ? 1 : 0;
            int tlen = bit ? T_BIT1 : T_BIT0;

            if (tape.subpulse == 0) {
                tape.subpulse = 1;
                tape.next_edge_at += tlen;
            } else {
                tape.subpulse = 0;
                if (tape.bit_mask == 0x01) {
                    tape.bit_mask = 0x80;
                    tape.i_byte++;
                } else {
                    tape.bit_mask >>= 1;
                }
                if (tape.i_byte >= tape.blk[tape.i_blk].len) {
                    tape.phase = TP_PAUSE;
                    tape.pause_end_at = t_now + T_MS(T_PAUSE_MS);
                } else {
                    b = tape.blk[tape.i_blk].data[tape.i_byte];
                    bit = (b & tape.bit_mask) ? 1 : 0;
                    tlen = bit ? T_BIT1 : T_BIT0;
                    tape.next_edge_at += tlen;
                }
            }
            break;
        }

        case TP_PAUSE:
            if (t_now < tape.pause_end_at) return;
            tape.i_blk++;
            tape.phase = TP_NEXTBLOCK;
            break;

        default:
            return;
        }
    }
}

static inline void tape_end_slice(void)
{
    uint64_t t_now = tape.slice_origin + (uint64_t)cpu_z80.tstates;
    tape_advance_to(t_now);
    tape.frame_origin = t_now;
}

/* ─────────────────────────────────────────────────────────────
 * ULA I/O
 * ───────────────────────────────────────────────────────────── */

static void ula_write(uint8_t v)
{
    /* ear is bit 4 mic is bit 3, border low bits */
    ula = v;

    /* ▶️ Actualiza beeper (EAR|MIC) al instante actual */
    beeper_set_from_ula(v);

    repaint_border(v & 7);
}

static uint8_t ula_read(uint16_t addr)
{
    uint8_t r = 0xA0;  /* Fixed bits */

    /* bit6 = EAR (entrada):
       - Si la cinta por pulsos está activa, domina la línea
       - Si no, conserva tu comportamiento previo para Issue 3/48K_2 */
    uint8_t ear_b6 = 0x00;
    if (tape_active()) {
        ear_b6 = tape_ear_bit6();
    } else if (model != ZX_PLUS3) {
        if (ula & 0x10)     /* Issue 3 and later */
            ear_b6 = 0x40;
        if (model == ZX_48K_2 && (ula & 0x08))
            ear_b6 = 0x40;
    }
    r = (r & ~0x40) | ear_b6;

    /* Low 5 bits are keyboard matrix map */
    r |= ~keymatrix_input(matrix, ~(addr >> 8)) & 0x1F;
    return r;
}

static uint8_t floating(void)
{
    unsigned n;
    if (blanked || model == ZX_PLUS3)
        return 0xFF;
    n = cpu_z80.tstates;
    n /= 4;
    if (n < 32)
        return ram[vram][0x1800 + 32 * drawline + n];
    return 0xFF;
}

static void divplus_ctrl(uint8_t val)
{
    divplus_latch = val;
    switch(val & 0xE0) {
    case 0xC0:
    case 0xE0:
        divide_latch = 0;
        divplus_latch = 0;
        divplus_7ffd = 0;
        divide_mapped = 0;
        break;
    case 0x00:
        /* DivIDE mode */
        /* bits 4-1 selects the extended banking */
        break;
    case 0x20:
        /* Enable 128K mode */
        divplus_128k = 1;
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

static uint8_t io_read(int unused, uint16_t addr)
{
    unsigned r;

    /* Kempston joystick: puerto 0x1F */
    if ((addr & 0xFF) == 0x1F) {
        return kempston_state_from_sdl();
    }

    /* Timex checks XXFE, Sinclair just the low bit */
    if ((addr & 0x01) == 0) /* ULA */
        return ula_read(addr);
    if (model == ZX_PLUS3) {
        if ((addr & 0xF002) == 0x2000)
            return fdc_read_ctrl(fdc);
        if ((addr & 0xF002) == 0x3000)
            return fdc_read_data(fdc);
    }
    if (divide) {
        if ((addr & 0xE3) == 0xA3) {
            r = (addr >> 2) & 0x07;
            if (r) {
                divide_oe = 1;  /* Odd mode */
                return ide_read16(ide, r);
            }
            if (divide_oe == 0) {
                divide_oe = 1;
                return divide_pair;
            }
            r = ide_read16(ide, 0);
            divide_pair = r >> 8;
            divide_oe = 0;
            return r & 0xFF;
        }
    }
    return floating();
}

static void io_write(int unused, uint16_t addr, uint8_t val)
{
    if (trace & TRACE_IO)
        fprintf(stderr, "write %02x <- %02x\n", addr, val);
    if ((addr & 1) == 0)
        ula_write(val);
    if (model == ZX_128K && (addr & 0x8002) == 0) {
        if ((mlatch & 0x20) == 0) {
            mlatch = val;
            recalc_mmu();
        }
    }
    if (model == ZX_PLUS3 && (addr & 0xF002) == 0x3000) {
        fdc_write_data(fdc, val);
    }
    if (model == ZX_PLUS3 && (addr & 0xC002) == 0x4000) {
        if ((mlatch & 0x20) == 0) {
            mlatch = val;
            recalc_mmu();
        }
    }
    if (model == ZX_PLUS3 && (addr & 0xF002) == 0x1000) {
        /* Does the memory latch lock this too ? TODO */
        p3latch = val;
        if (p3latch & 0x08)
            fdc_set_motor(fdc, 3);
        else
            fdc_set_motor(fdc, 0);
        recalc_mmu();
    }
    if (divide) {
        if ((addr & 0xE3) == 0xA3) {
            uint8_t r = (addr >> 2) & 0x07;
            if (r) {
                divide_oe = 1;  /* Odd mode */
                ide_write16(ide, r, val);
            } else if (divide_oe == 1) {
                divide_oe = 0;
                divide_pair = val;
            } else {
                ide_write16(ide, 0, divide_pair | (val << 8));
                divide_oe = 0;
            }
        }
        if ((addr & 0xE3) == 0xE3) {
            /* MAPRAM cannot be cleared */
            val |= divide_latch & 0x40;
            divide_latch = val;
            if (val & 0x80)
                divide_mapped = 1;
        }
        if (divide ==2 && (addr & 0xFF) == 0x17)
            divplus_ctrl(val);
    }
}

static unsigned int nbytes;

uint8_t z80dis_byte(uint16_t addr)
{
    uint8_t r = do_mem_read(addr, 1);
    fprintf(stderr, "%02X ", r);
    nbytes++;
    return r;
}

uint8_t z80dis_byte_quiet(uint16_t addr)
{
    return do_mem_read(addr, 1);
}

static void z80_trace(unsigned unused)
{
    static uint32_t lastpc = -1;
    char buf[256];

    if ((trace & TRACE_CPU) == 0)
        return;
    nbytes = 0;
    /* Spot XXXR repeating instructions and squash the trace */
    if (cpu_z80.M1PC == lastpc
        && z80dis_byte_quiet(lastpc) == 0xED
        && (z80dis_byte_quiet(lastpc + 1) & 0xF4) == 0xB0) {
        return;
    }
    lastpc = cpu_z80.M1PC;
    fprintf(stderr, "%04X: ", lastpc);
    z80_disasm(buf, lastpc);
    while (nbytes++ < 6)
        fprintf(stderr, "   ");
    fprintf(stderr, "%-16s ", buf);
    fprintf(stderr,
        "[ %02X:%02X %04X %04X %04X %04X %04X %04X ]\n",
        cpu_z80.R1.br.A, cpu_z80.R1.br.F, cpu_z80.R1.wr.BC,
        cpu_z80.R1.wr.DE, cpu_z80.R1.wr.HL,
        cpu_z80.R1.wr.IX, cpu_z80.R1.wr.IY, cpu_z80.R1.wr.SP);
}

static void poll_irq_event(void)
{
}

static void reti_event(void)
{
}

static void raster_byte(unsigned lines, unsigned cols, uint8_t byte, uint8_t attr)
{
    uint32_t *pixp;
    unsigned x;
    unsigned paper = (attr >> 3) & 0x0F;
    unsigned ink = attr & 7;
    if (attr & 0x40)
        paper |= 0x08;

    /* Flash swaps every 16 frames */
    if ((attr & 0x80) && (frames & 0x10)) {
        x = ink;
        ink = paper;
        paper = x;
    }

    pixp = texturebits + (lines + BORDER) * WIDTH + cols * 8 + BORDER;

    for (x = 0; x < 8; x++) {
        if (byte & 0x80)
            *pixp++ = palette[ink];
        else
            *pixp++ = palette[paper];
        byte <<= 1;
    }
}

static void raster_block(unsigned ybase, unsigned off, unsigned aoff)
{
    unsigned c,l,w;
    uint8_t *ptr = ram[vram] + off;
    uint8_t *aptr = ram[vram] + aoff;
    for (l = 0; l < 8; l++) {
        for (c = 0; c < 8; c++)
            for (w = 0; w < 32; w++)
                raster_byte(ybase + c * 8 + l, w, *ptr++, *aptr++);
        aptr -= 0x100;
    }
}

static void spectrum_rasterize(void)
{
    raster_block(0, 0x0000, 0x1800);
    raster_block(64, 0x0800, 0x1900);
    raster_block(128, 0x1000, 0x1A00);
}

static void spectrum_render(void)
{
    SDL_Rect rect;

    rect.x = rect.y = 0;
    rect.w = WIDTH;
    rect.h = HEIGHT;

    SDL_UpdateTexture(texture, NULL, texturebits, WIDTH * 4);
    SDL_RenderClear(render);
    SDL_RenderCopy(render, texture, NULL, &rect);
    SDL_RenderPresent(render);
}

/*
 *  Keyboard mapping.
 *  TODO:
 */

static SDL_Keycode keyboard[] = {
    SDLK_LSHIFT, SDLK_z, SDLK_x, SDLK_c, SDLK_v,
    SDLK_a, SDLK_s, SDLK_d, SDLK_f, SDLK_g,
    SDLK_q, SDLK_w, SDLK_e, SDLK_r, SDLK_t,
    SDLK_1, SDLK_2, SDLK_3, SDLK_4, SDLK_5,
    SDLK_0, SDLK_9, SDLK_8, SDLK_7, SDLK_6,
    SDLK_p, SDLK_o, SDLK_i, SDLK_u, SDLK_y,
    SDLK_RETURN, SDLK_l, SDLK_k, SDLK_j, SDLK_h,
    SDLK_SPACE, SDLK_RSHIFT, SDLK_m, SDLK_n, SDLK_b
};

static void run_scanlines(unsigned lines, unsigned blank)
{
    unsigned i;
    unsigned n = 224;   /* T States per op */

    blanked = blank;

    if (!blanked)
        drawline = 0;
    /* Run scanlines */
    for (i = 0; i < lines; i++) {
        /* Delimitamos porciones (beeper + cinta) alrededor de la ejecución */
        beeper_begin_slice();
        tape_begin_slice();

        n = 224 + 224 - Z80ExecuteTStates(&cpu_z80, n);

        beeper_end_slice();
        tape_end_slice();

        if (!blanked)
            drawline++;
    }
    if (ui_event())
        emulator_done = 1;

    if (int_recalc) {
        /* If there is no pending Z80 vector IRQ but we think
           there now might be one we use the same logic as for
           reti */
        poll_irq_event();
        /* Clear this after because reti_event may set the
           flags to indicate there is more happening. We will
           pick up the next state changes on the reti if so */
        if (!(cpu_z80.IFF1 | cpu_z80.IFF2))
            int_recalc = 0;
    }
}

/* ─────────────────────────────────────────────────────────────
 * SNA loader (48K & 128K)
 * 48K: PC se toma de la pila (RETN implícito) y SP += 2.
 * 128K: PC explícito tras 48K; 7FFD y bancos restantes.
 * Refs: Sinclair Wiki / Claus Jahn
 * ───────────────────────────────────────────────────────────── */
bool load_sna(const char* filename)
{
    FILE* f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "No se pudo abrir .sna: %s\n", filename);
        return false;
    }

    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t header[27];
    if (fread(header, 1, 27, f) != 27) {
        fclose(f);
        fprintf(stderr, "Archivo .sna incompleto (header)\n");
        return false;
    }

    /* Registros (layout 48K) */
    cpu_z80.I          = header[0];
    cpu_z80.R2.wr.HL   = (header[2]  << 8) | header[1];
    cpu_z80.R2.wr.DE   = (header[4]  << 8) | header[3];
    cpu_z80.R2.wr.BC   = (header[6]  << 8) | header[5];
    cpu_z80.R2.wr.AF   = (header[8]  << 8) | header[7];
    cpu_z80.R1.wr.HL   = (header[10] << 8) | header[9];
    cpu_z80.R1.wr.DE   = (header[12] << 8) | header[11];
    cpu_z80.R1.wr.BC   = (header[14] << 8) | header[13];
    cpu_z80.R1.wr.IY   = (header[16] << 8) | header[15];
    cpu_z80.R1.wr.IX   = (header[18] << 8) | header[17];
    /* Byte 19: docs clásicos indican IFF2; usamos bit2 o cualquier no-cero como habilitado */
    cpu_z80.IFF2       = (header[19] & 0x04) ? 1 : (header[19] ? 1 : 0);
    cpu_z80.R          = header[20];
    cpu_z80.R1.wr.AF   = (header[22] << 8) | header[21];
    cpu_z80.R1.wr.SP   = (header[24] << 8) | header[23];
    cpu_z80.IM         = header[25];
    border_color       = header[26] & 0x07;

    /* RAM 48K */
    uint8_t ram48k[49152];
    if (fread(ram48k, 1, sizeof(ram48k), f) != sizeof(ram48k)) {
        fclose(f);
        fprintf(stderr, "Archivo .sna incompleto (RAM 48K)\n");
        return false;
    }

    bool is128k = (fsz == 131103L || fsz == 147487L);

    if (!is128k) {
        /* 48K puro → copiar a 0x4000..0xFFFF */
        for (uint32_t off = 0; off < sizeof(ram48k); ++off)
            mem_write(0, 0x4000u + off, ram48k[off]);

        /* PC desde la pila; SP += 2 */
        uint16_t sp = cpu_z80.R1.wr.SP;
        uint8_t pcl = mem_read(0, sp);
        uint8_t pch = mem_read(0, sp + 1);
        cpu_z80.PC = ((uint16_t)pch << 8) | pcl;
        cpu_z80.R1.wr.SP = sp + 2;

        cpu_z80.IFF1 = cpu_z80.IFF2;

        fclose(f);
        printf("Snapshot .sna (48K) cargado: %s\n", filename);
        printf("PC=0x%04X  SP=0x%04X  Border=%d  IM=%d\n",
               cpu_z80.PC, cpu_z80.R1.wr.SP, border_color, cpu_z80.IM);
        return true;
    }

    /* 128K */
    int pcl = fgetc(f), pch = fgetc(f);
    if (pcl == EOF || pch == EOF) { fclose(f); fprintf(stderr, "SNA 128K: error leyendo PC\n"); return false; }
    cpu_z80.PC = (uint16_t)(pcl | (pch << 8));

    int last7ffd = fgetc(f);
    int trdos    = fgetc(f);
    if (last7ffd == EOF || trdos == EOF) { fclose(f); fprintf(stderr, "SNA 128K: error leyendo 7FFD/TRDOS\n"); return false; }
    uint8_t latch7ffd = (uint8_t)last7ffd;
    uint8_t bank_n = latch7ffd & 0x07;

    /* 48K iniciales → bancos 5 / n / 2 */
    memcpy(&ram[RAM(5)][0],        &ram48k[0x0000], 16384);
    memcpy(&ram[RAM(bank_n)][0],   &ram48k[0x4000], 16384);
    memcpy(&ram[RAM(2)][0],        &ram48k[0x8000], 16384);

    /* Rellenar el resto de bancos (5 o 6) en orden de archivo */
    int remain[8] = {0,1,2,3,4,5,6,7};
    remain[5] = -1; remain[2] = -1; remain[bank_n] = -1;

    for (int b = 0; b < 8; ++b) {
        if (remain[b] >= 0) {
            if (fread(&ram[RAM(remain[b])][0], 1, 16384, f) != 16384) {
                fclose(f);
                fprintf(stderr, "SNA 128K: error leyendo banco %d\n", remain[b]);
                return false;
            }
        }
    }

    /* Ajustar latch de paginación / MMU */
    mlatch = latch7ffd & 0x3F;
    recalc_mmu();

    /* Sincronizar ventana visible con snapshot */
    for (uint32_t off = 0; off < 16384; ++off)
        mem_write(0, 0x4000u + off, ram[RAM(5)][off]);
    for (uint32_t off = 0; off < 16384; ++off)
        mem_write(0, 0x8000u + off, ram[RAM(2)][off]);
    for (uint32_t off = 0; off < 16384; ++off)
        mem_write(0, 0xC000u + off, ram[RAM(bank_n)][off]);

    cpu_z80.IFF1 = cpu_z80.IFF2;

    fclose(f);
    printf("Snapshot .sna (128K) cargado: %s  (7FFD=0x%02X, n=%u, TR-DOS=%u)\n",
           filename, latch7ffd, bank_n, trdos);
    printf("PC=0x%04X  SP=0x%04X  Border=%d  IM=%d\n",
           cpu_z80.PC, cpu_z80.R1.wr.SP, border_color, cpu_z80.IM);

    return true;
}

/* ─────────────────────────────────────────────────────────────
 * TAP fast loader + listado
 *   - Formato: [len_lo len_hi][flag][payload][checksum XOR]
 *   - Header ROM: 17 bytes -> type, name(10), len_data, p1, p2
 *   - CODE/SCREEN$: carga a p1 (start address) con len_data bytes.
 * Refs: Sinclair Wiki TAP format / Spectrum tape interface
 * ───────────────────────────────────────────────────────────── */
static inline uint16_t rd_le16_file(FILE* f)
{
    int lo = fgetc(f);
    int hi = fgetc(f);
    if (lo == EOF || hi == EOF) return 0;
    return (uint16_t)(lo | (hi << 8));
}

typedef struct {
    uint8_t  type;      /* 0,1,2,3 (PROGRAM, NUM, CHAR, CODE) */
    char     name[11];  /* 10 chars + NUL */
    uint16_t len_data;
    uint16_t p1;        /* para CODE: dirección de carga */
    uint16_t p2;        /* para CODE: normalmente 32768 (0x8000) */
} tap_header_t;

static bool tap_read_header(FILE* f, long block_start_pos, uint16_t blk_len, tap_header_t* out)
{
    uint8_t hdr[17];
    if (fread(hdr, 1, 17, f) != 17) return false;

    /* checksum (1 byte) */
    int chks = fgetc(f); (void)chks;

    out->type = hdr[0];
    memcpy(out->name, &hdr[1], 10);
    out->name[10] = 0;
    out->len_data = (uint16_t)(hdr[11] | (hdr[12] << 8));
    out->p1       = (uint16_t)(hdr[13] | (hdr[14] << 8));
    out->p2       = (uint16_t)(hdr[15] | (hdr[16] << 8));

    if (blk_len != 19) {
        fprintf(stderr, "[TAP] Advertencia: cabecera con len=%u (esperado 19)\n", blk_len);
    }
    return true;
}

/* Lista TAP por consola (no carga) */
static void tap_list(const char* path)
{
    if (!path) { fprintf(stderr, "[TAP] No hay fichero (-t)\n"); return; }

    FILE* f = fopen(path, "rb");
    if (!f) { perror(path); return; }

    long block_start = 0;
    
    printf("Bloque: %ld", block_start);

    printf("=== TAP LIST: %s ===\n", path);
    int index = 0;
    while (1) {
        uint16_t blk_len = rd_le16_file(f);
        if (feof(f)) break;
        if (blk_len == 0 && ferror(f)) { perror("[TAP]"); break; }

        block_start = ftell(f);
        int flag = fgetc(f);
        if (flag == EOF) break;

        if (flag == 0x00) {
            uint8_t hdr[17];
            if (fread(hdr, 1, 17, f) != 17) { fprintf(stderr, "[TAP] header truncado\n"); break; }
            int ch = fgetc(f); (void)ch;

            uint8_t type = hdr[0];
            char name[11]; memcpy(name, &hdr[1], 10); name[10] = 0;
            uint16_t len_data = (uint16_t)(hdr[11] | (hdr[12] << 8));
            uint16_t p1       = (uint16_t)(hdr[13] | (hdr[14] << 8));
            uint16_t p2       = (uint16_t)(hdr[15] | (hdr[16] << 8));

            const char* tname = (type==0?"PROGRAM":(type==1?"NUMARRAY":(type==2?"CHARARRAY":(type==3?"CODE":"?"))));
            printf(" [%03d] HEADER  len=%u  type=%s  name=\"%.*s\"  data=%u  p1=%u  p2=%u\n",
                   index++, blk_len, tname, 10, name, len_data, p1, p2);
        } else if (flag == 0xFF) {
            size_t toread = (blk_len >= 1) ? (blk_len - 1) : 0;
            if (toread >= 1) {
                fseek(f, (long)(toread - 1), SEEK_CUR);
                int cs = fgetc(f); (void)cs;
            }
            printf(" [%03d] DATA    len=%u\n", index++, blk_len);
        } else {
            size_t skip = (blk_len >= 1) ? (blk_len - 1) : 0;
            if (skip) fseek(f, (long)skip, SEEK_CUR);
            printf(" [%03d] FLAG=0x%02X (saltado) len=%u\n", index++, flag, blk_len);
        }
    }

    fclose(f);
}

/* Carga CODE/SCREEN$ de un TAP. Autostart opcional (PC := start address del último CODE) */
bool load_tap_fast(const char* path, int auto_start)
{
    FILE* f = fopen(path, "rb");
    if (!f) { perror(path); return false; }

    long block_start = 0;

    printf("=== TAP: %s ===\n", path);

    tap_header_t last_hdr = {0};
    bool have_hdr = false;
    bool loaded_any_code = false;
    uint16_t last_code_start = 0;

    while (1) {
        uint16_t blk_len = rd_le16_file(f);
        if (feof(f)) break;
        if (blk_len == 0 && ferror(f)) { fclose(f); return false; }

        block_start = ftell(f);
        int flag = fgetc(f);
        if (flag == EOF) break;

        if (flag == 0x00) {
            tap_header_t H;
            if (!tap_read_header(f, block_start, blk_len, &H)) { fclose(f); return false; }
            have_hdr = true; last_hdr = H;

            const char* tname = (H.type==0?"PROGRAM":(H.type==1?"NUMARRAY":(H.type==2?"CHARARRAY":(H.type==3?"CODE":"?"))));
            printf(" - HEADER: type=%s  name=\"%.*s\"  len_data=%u  p1=%u  p2=%u\n",
                   tname, 10, last_hdr.name, last_hdr.len_data, last_hdr.p1, last_hdr.p2);
        }
        else if (flag == 0xFF) {
            if (!have_hdr) {
                size_t toread = (blk_len >= 1) ? (blk_len - 1) : 0; /* payload + checksum */
                if (toread > 0) fseek(f, (long)toread, SEEK_CUR);
                printf(" - DATA sin header previo: saltado (%u bytes)\n", blk_len);
                continue;
            }

            /* payload (dsz = blk_len-2 bytes de datos + 1 checksum al final) */
            uint16_t payload_len = (blk_len >= 2) ? (blk_len - 2) : 0;
            if (payload_len < 1) { fclose(f); fprintf(stderr, "[TAP] DATA tamaño inválido.\n"); return false; }

            uint8_t* buf = (uint8_t*)malloc(payload_len);
            if (!buf) { fclose(f); return false; }

            size_t rd = fread(buf, 1, payload_len, f);
            if (rd != payload_len) { free(buf); fclose(f); return false; }

            uint8_t *data = buf;
            size_t   dsz  = (payload_len >= 1) ? (payload_len - 1) : 0; /* sin checksum final */
            uint16_t declared = last_hdr.len_data;

            if (last_hdr.type == 3) {
                uint16_t start = last_hdr.p1;
                size_t tocopy = (declared <= dsz) ? declared : dsz;
                printf("   · Cargando CODE en 0x%04X (%zu bytes)\n", start, tocopy);

                for (size_t i = 0; i < tocopy; ++i) {
                    mem_write(0, (uint16_t)(start + i), data[i]);
                }
                loaded_any_code = true;
                last_code_start = start;
            } else {
                printf("   · DATA type=%u no cargado (soportado solo CODE por ahora)\n", last_hdr.type);
            }

            free(buf);
            have_hdr = false;
        }
        else {
            size_t skip = (blk_len >= 1) ? (blk_len - 1) : 0;
            if (skip) fseek(f, (long)skip, SEEK_CUR);
            printf(" - FLAG 0x%02X no estándar: saltado (%u bytes)\n", flag, blk_len);
        }
    }

    fclose(f);

    if (loaded_any_code) {
        if (auto_start) {
            cpu_z80.PC = last_code_start;
            printf("AUTO-START: PC := 0x%04X\n", cpu_z80.PC);
        }
    } else {
        fprintf(stderr, "[TAP] No se cargó ningún bloque CODE.\n");
        return false;
    }

    return true;
}

/* ─────────────────────────────────────────────────────────────
 * Hotkeys (SDL): F6 = Reload TAP & Auto-Start; F7 = List TAP
 *                F8 = Play/Pause tape pulses; F9 = Rewind tape
 * ───────────────────────────────────────────────────────────── */
static void handle_hotkeys(const char* tap_path)
{
    SDL_PumpEvents(); /* actualiza estado */

    const Uint8* ks = SDL_GetKeyboardState(NULL);
    static int prev_f6 = 0, prev_f7 = 0, prev_f8 = 0, prev_f9 = 0, prev_f12 = 0;

    int f6 = ks[SDL_SCANCODE_F6] ? 1 : 0;
    int f7 = ks[SDL_SCANCODE_F7] ? 1 : 0;
    int f8 = ks[SDL_SCANCODE_F8] ? 1 : 0; /* Play/Pause pulses */
    int f9 = ks[SDL_SCANCODE_F9] ? 1 : 0; /* Rewind pulses */
	int f11 = ks[SDL_SCANCODE_F11] ? 1: 0; //slow
	int f12 = ks[SDL_SCANCODE_F12] ? 1: 0; //fast

    if (f6 && !prev_f6) {
        if (!tap_path) {
            fprintf(stderr, "[F6] No hay TAP (usa -t <fichero.tap>)\n");
        } else {
            printf("[F6] Reload TAP & Auto-Start: %s\n", tap_path);
            if (!load_tap_fast(tap_path, /*auto_start=*/1))
                fprintf(stderr, "[TAP] Fallo al recargar: %s\n", tap_path);
        }
    }
    if (f7 && !prev_f7) {
        if (!tap_path) {
            fprintf(stderr, "[F7] No hay TAP (usa -t <fichero.tap>)\n");
        } else {
            tap_list(tap_path);
        }
    }

    if (f8 && !prev_f8) {
        tape.playing = !tape.playing;
        fprintf(stdout, "[F8] Tape %s\n", tape.playing ? "PLAY" : "PAUSE");
    }
    if (f9 && !prev_f9) {
        tape.i_blk = 0; tape.phase = TP_NEXTBLOCK; tape.frame_origin = tape.slice_origin = 0;
        tape.next_edge_at = 0; tape.pause_end_at = 0; tape.ear_level = 1; tape.playing = 1;
        fprintf(stdout, "[F9] Tape REWIND\n");
    }

	if (f11) {
        fast = 0;
		fprintf(stdout, "[F12] SLOW!\n");
		return;
    }
	
	if (f12 && !prev_f12) {
        fast = !fast;
		if (fast)
		{
			fprintf(stdout, "[F12] SPEED!\n");
		} else {
			fprintf(stdout, "[F12] SLOW!\n");
		}
    }
	

    prev_f6 = f6;
    prev_f7 = f7;
    prev_f8 = f8;
    prev_f9 = f9;
	prev_f12 = f12;
}

static void usage(void)
{
    fprintf(stderr, "spectrum: [-f] [-r path] [-d debug] [-A disk] [-B disk]\n"
            "          [-i idedisk] [-I dividerom] [-t tap] [-s sna] [-T tap_pulses]\n");
    exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
    static struct timespec tc;
    int opt;
    int fd;
    int l;
    char *rompath = (char*)"spectrum.rom";
    char *divpath = (char*)"divide.rom";
    char *idepath = NULL;
    char *tapepath = NULL;
    char *patha = NULL;
    char *pathb = NULL;
    char *snapath = NULL;
    char *tap_pulses_path = NULL;

    /* Añadimos 't:' (tap fast) y 'T:' (tap pulses) */
    while ((opt = getopt(argc, argv, "d:f:r:m:i:I:A:B:s:t:T:")) != -1) {
        switch (opt) {
        case 'r':
            rompath = optarg;
            break;
        case 'd':
            trace = atoi(optarg);
            break;
        case 'f':
            fast = 1;
            break;
        case 't':
            tapepath = optarg;
            break;
        case 'T':
            tap_pulses_path = optarg;
            break;
        case 'm':
            mem = atoi(optarg);
            break;
        case 'i':
            idepath = optarg;
            break;
        case 'I':
            divpath = optarg;
            break;
        case 'A':
            patha = optarg;
            break;
        case 'B':
            pathb = optarg;
            break;
        case 's':
            snapath = optarg;
            break;
        default:
            usage();
        }
    }
    if (optind < argc)
        usage();

    if (mem < 16 || mem > 48) {
        fprintf(stderr, "spectrum: base memory %dK is out of range.\n", mem);
        exit(1);
    }

    mem *= 1024;
    mem += 16384;

    fd = open(rompath, O_RDONLY);
    if (fd == -1) {
        perror(rompath);
        exit(EXIT_FAILURE);
    }
    l = read(fd, ram, 0x10000);
    switch(l) {
    case 0x4000:
        break;
    case 0x8000:
        model = ZX_128K;
        break;
    case 0x10000:
        model = ZX_PLUS3;
        break;
    default:
        fprintf(stderr, "spectrum: invalid rom '%s'.\n", rompath);
        exit(EXIT_FAILURE);
    }
    close(fd);

    if (model == ZX_PLUS3) {
        fdc = fdc_new();

        lib765_register_error_function(fdc_log);

        if (patha) {
            drive_a = fd_newdsk();
            fd_settype(drive_a, FD_30);
            fd_setheads(drive_a, 1);
            fd_setcyls(drive_a, 40);
            fdd_setfilename(drive_a, patha);
            printf("Attached disk '%s' as A\n", patha);
        } else
            drive_a = fd_new();

        if (pathb) {
            drive_b = fd_newdsk();
            fd_settype(drive_a, FD_35);
            fd_setheads(drive_a, 2);
            fd_setcyls(drive_a, 80);
            fdd_setfilename(drive_a, pathb);
        } else
            drive_b = fd_new();

        fdc_reset(fdc);
        fdc_setisr(fdc, NULL);

        fdc_setdrive(fdc, 0, drive_a);
        fdc_setdrive(fdc, 1, drive_b);
    }

    ui_init();

    window = SDL_CreateWindow("ZX Spectrum",
                  SDL_WINDOWPOS_UNDEFINED,
                  SDL_WINDOWPOS_UNDEFINED,
                  WIDTH,
                  HEIGHT, SDL_WINDOW_RESIZABLE);
    if (window == NULL) {
        fprintf(stderr,
            "spectrum: unable to open window: %s\n",
            SDL_GetError());
        exit(1);
    }
    render = SDL_CreateRenderer(window, -1, 0);
    if (render == NULL) {
        fprintf(stderr,
            "spectrum: unable to create renderer: %s\n",
            SDL_GetError());
        exit(1);
    }
    texture =
        SDL_CreateTexture(render,
                  SDL_PIXELFORMAT_ARGB8888,
                  SDL_TEXTUREACCESS_STREAMING,
                  WIDTH, HEIGHT);
    if (texture == NULL) {
        fprintf(stderr,
            "spectrum: unable to create texture: %s\n",
            SDL_GetError());
        exit(1);
    }
    SDL_SetRenderDrawColor(render, 0, 0, 0, 255);
    SDL_RenderClear(render);
    SDL_RenderPresent(render);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
    SDL_RenderSetLogicalSize(render, WIDTH, HEIGHT);

    matrix = keymatrix_create(8, 5, keyboard);
    keymatrix_trace(matrix, trace & TRACE_KEY);
    keymatrix_add_events(matrix);

    tc.tv_sec = 0;
    tc.tv_nsec = 20000000L; /* 20ms (50Hz frame rate) */

    Z80RESET(&cpu_z80);
    cpu_z80.ioRead = io_read;
    cpu_z80.ioWrite = io_write;
    cpu_z80.memRead = mem_read;
    cpu_z80.memWrite = mem_write;
    cpu_z80.trace = z80_trace;

    /* Audio beeper */
    if (audio_init_sdl(44100) != 0) {
        fprintf(stderr, "Aviso: audio deshabilitado (SDL_OpenAudioDevice falló).\n");
    } else {
        beeper_frame_origin = 0;
        beeper_slice_origin = 0;
        beeper_last_tstate  = 0;
        beeper_level        = 0;
        beeper_frac_acc     = 0.0;
    }

    /* TAP (fast) – inyección directa */
    if (tapepath) {
        if (!load_tap_fast(tapepath, /*auto_start=*/1)) {
            fprintf(stderr, "Fallo al cargar TAP (fast): %s\n", tapepath);
        } else {
            printf("TAP (fast) cargado correctamente.\n");
        }
    }

    /* TAP por pulsos – reproducción en EAR para la ROM */
    if (tap_pulses_path) {
        if (tape_load_tap_pulses(tap_pulses_path) != 0) {
            fprintf(stderr, "Fallo al cargar TAP (pulsos): %s\n", tap_pulses_path);
        } else {
            printf("Reproduciendo cinta por pulsos (ROM std): %s\n", tap_pulses_path);
        }
    }

    if (snapath)
    {
        load_sna(snapath);
    }

    if (idepath) {
        ide = ide_allocate("divide0");
        fd = open(idepath, O_RDWR);
        if (fd == -1) {
            perror(idepath);
            exit(1);
        }
        if (ide_attach(ide, 0, fd) == 0)
            ide_reset_begin(ide);
        else {
            fprintf(stderr, "ide: attach failed.\n");
            exit(1);
        }
        fd = open(divpath, O_RDONLY);
        if (fd == -1) {
            perror(divpath);
            exit(1);
        }
        l = read(fd, divrom, 524288);
        if (l == 8192)
            divide = 1;
        else if (l == 524288)
            divide = 2;
        else {
            fprintf(stderr, "spectrum: divide.rom invalid.\n");
            exit(1);
        }
    }

    while (!emulator_done) {
        /* Hotkeys: F6 (reload TAP & autostart), F7 (list TAP),
                    F8 (play/pause pulses), F9 (rewind pulses) */
        handle_hotkeys(tapepath);

        /* 192 scanlines framebuffer */
        run_scanlines(192, 1);
        /* and border */
        run_scanlines(56, 0);
        spectrum_rasterize();
        spectrum_render();
        Z80INT(&cpu_z80, 0xFF);
        /* 64 scan lines of 224 T states for vblank etc */
        run_scanlines(64, 0);
        poll_irq_event();
        frames++;
        /* Do a small block of I/O and delays */
        if (!fast)
            nanosleep(&tc, NULL);
        if (fdc)
            fdc_tick(fdc);
    }

    if (audio_dev) {
        SDL_CloseAudioDevice(audio_dev);
        audio_dev = 0;
    }
    exit(0);
}