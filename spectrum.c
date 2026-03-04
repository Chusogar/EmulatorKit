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
 *   - TZX pulse player: full TZX support via tzx.c (blocks 0x10-0x19, control)
 *   - Beeper (EAR|MIC) audio via SDL2 (queue mode)
 *   - Hotkeys: F6 (reload TAP & autostart fast), F7 (list TAP),
 *              F8 (Play/Pause tape/TZX pulses), F9 (Rewind tape/TZX pulses)
 *
 *  TAP format refs:
 *    - https://sinclair.wiki.zxnet.co.uk/wiki/TAP_format
 *    - https://sinclair.wiki.zxnet.co.uk/wiki/Spectrum_tape_interface
 *  TZX format refs:
 *    - https://sinclair.wiki.zxnet.co.uk/wiki/TZX_format
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
#include <SDL2/SDL.h>
#include "libz80/z80.h"
#include "z80dis.h"
#include "ide.h"
#include "lib765/include/765.h"
#include "sna.h"
#include "event.h"
#include "keymatrix.h"
#include "ay8912.h"

static SDL_Window *window;
static SDL_Renderer *render;
static SDL_Texture *texture;

#define BORDER  32
#define WIDTH   (256 + 2 * BORDER)
#define HEIGHT  (192 + 2 * BORDER)
#define SCALE	1

/* T-state clock rate (PAL ~3.5469 MHz; tolerable for 48K too) */
#define TSTATES_CPU     3546900.0

static uint32_t texturebits[WIDTH * HEIGHT];

uint8_t border_color   = 7;

static uint32_t palette[16] = {
    0xFF000000, 0xFF0000D8, 0xFFD80000, 0xFFD800D8,
    0xFF00D800, 0xFF00D8D8, 0xFFD8D800, 0xFFD8D8D8,
    0xFF000000, 0xFF0000FF, 0xFFFF0000, 0xFFFF00FF,
    0xFF00FF00, 0xFF00FFFF, 0xFFFFFF00, 0xFFFFFFFF
};

// ─────────────────────────────────────────────────────────────
// Utilidades lectura LE
// ─────────────────────────────────────────────────────────────
static inline uint8_t rd_u8(FILE* f) { int c = fgetc(f); return (c == EOF) ? 0 : (uint8_t)c; }
static inline uint16_t rd_u16(FILE* f) { uint16_t lo = rd_u8(f), hi = rd_u8(f); return (uint16_t)(lo | (hi << 8)); }
static inline uint32_t rd_u24(FILE* f) { uint32_t b0 = rd_u8(f), b1 = rd_u8(f), b2 = rd_u8(f); return (b0 | (b1 << 8) | (b2 << 16)); }
static inline uint32_t rd_u32(FILE* f) { uint32_t b0 = rd_u8(f), b1 = rd_u8(f), b2 = rd_u8(f), b3 = rd_u8(f); return (b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)); }
#define MS_TO_TSTATES(ms) ((uint64_t)((ms) * 3500ULL)) // 3.5MHz

// ─────────────────────────────────────────────────────────────
// Motor de cinta unificado (TAP/TZX)
// ─────────────────────────────────────────────────────────────
typedef enum { TAPE_FMT_NONE=0, TAPE_FMT_TAP=1, TAPE_FMT_TZX=2 } tape_fmt_t;
typedef enum { PH_IDLE, PH_PILOT, PH_SYNC1, PH_SYNC2, PH_DATA, PH_PURE_TONE, PH_PULSE_SEQ, PH_DIRECT_REC, PH_PAUSE } pulse_phase_t;

typedef struct {
    FILE*   f;
    long    file_size;
    long    file_pos;

    // Formato
    tape_fmt_t fmt;

    // Estado de reproducción
    pulse_phase_t phase;
    int      pulses_left;            // pilot/pure tone en medias ondas
    uint32_t halfwave_ts;            // duración media onda (T-states)
    uint64_t next_edge_cycle;        // T-state del próximo flanco
    bool     level;                  // EAR actual (true=1)

    // Datos de bits
    uint8_t* blk;
    uint32_t blk_len;
    uint32_t data_pos;
    uint8_t  cur_byte;
    int      cur_bit;                // 7..0
    int      pulse_of_bit;           // 0/1

    // Parámetros activos (TZX/TAP parametrizadas)
    uint16_t t_pilot, t_sync1, t_sync2, t_bit0, t_bit1;
    uint16_t pilot_pulses;           // en ondas completas
    uint16_t used_bits_last;         // 0 => 8
    uint32_t pause_ms;

    // Secuencias (0x13 / 0x18 / 0x19)
    uint16_t* pulse_seq;
    int       pulse_seq_n;
    int       pulse_seq_i;

    // Direct recording (0x15)
    uint16_t dr_tstates_per_sample;
    uint32_t dr_total_bits;
    uint32_t dr_bit_index;

    // CSW (0x18) — como secuencia de medias ondas ya convertidas
    uint32_t csw_freq_hz;
    uint8_t  csw_compression;        // 0=raw sin compresión (soportado)
    uint32_t csw_data_len;

    // Control
    double speed;                    // solo TAP (escala)
    bool   playing;

    // Nivel inicial (0x2B)
    bool   initial_level_known;
    bool   initial_level;

    // Loop (0x24/0x25)
    struct { long file_pos_at_loop; uint16_t remaining; int active; } loop;

    // Grupos (0x21/0x22) - no afecta reproducción; útil para validar
    int group_depth;

} tape_t;

static tape_t tape = {0};
static const char* tape_filename = NULL;
uint64_t global_cycles = 0;

bool load_tap(const char* filename);

// ─────────────────────────────────────────────────────────────
// Timings TAP por defecto (T-states) @3.5MHz
// ─────────────────────────────────────────────────────────────
//static const int TS_PILOT = 2168;
//static const int TS_SYNC1 = 667;
//static const int TS_SYNC2 = 735;
//static const int TS_BIT0  = 855;
//static const int TS_BIT1  = 1710;

static inline uint32_t halfwave_for_bit(bool bit1) { return bit1 ? tape.t_bit1 : tape.t_bit0; }

// ─────────────────────────────────────────────────────────────
// Listado de bloques TAP/TZX (para mostrar al cargar)
// ─────────────────────────────────────────────────────────────
static void list_tap_blocks(const char* filename) {
    FILE* f = fopen(filename, "rb"); if (!f) { printf("No se pudo abrir TAP para listar: %s\n", filename); return; }
    fseek(f, 0, SEEK_END); long fsz = ftell(f); fseek(f, 0, SEEK_SET);
    printf("=== LISTA TAP: %s (%ld bytes) ===\n", filename, fsz);
    int idx=0;
    while (1) {
        uint8_t len_le[2];
        if (fread(len_le,1,2,f)!=2) break;
        uint16_t len = (uint16_t)(len_le[0] | (len_le[1]<<8));
        if (len==0 || len > 65535) { printf("Bloque %d: longitud inválida %u\n", idx, len); break; }
        uint8_t first=0xFF;
        long pos=ftell(f);
        if (len>=1) { fread(&first,1,1,f); fseek(f, pos, SEEK_SET); }
        printf("Bloque %3d: len=%5u  flag=0x%02X (%s)\n", idx, len, first, (first==0x00?"HEADER/flag=0x00":(first==0xFF?"DATA/flag=0xFF":"?")));
        fseek(f, len, SEEK_CUR);
        idx++;
    }
    fclose(f);
}

static const char* tzx_name(uint8_t id) {
    switch (id) {
        case 0x00: return "Standard Speed Data (legacy alias)";
        case 0x02: return "Pure Tone (legacy alias)";
        case 0x10: return "Standard Speed Data";
        case 0x11: return "Turbo Speed Data";
        case 0x12: return "Pure Tone";
        case 0x13: return "Pulse Sequence";
        case 0x14: return "Pure Data";
        case 0x15: return "Direct Recording";
        case 0x18: return "CSW Recording";
        case 0x19: return "Generalized Data";
        case 0x20: return "Pause";
        case 0x21: return "Group Start";
        case 0x22: return "Group End";
        case 0x24: return "Loop Start";
        case 0x25: return "Loop End";
        case 0x2A: return "Stop if 48K";
        case 0x2B: return "Set Signal Level";
        case 0x30: return "Text Description";
        case 0x31: return "Message";
        case 0x32: return "Archive Info";        // añadido
        case 0x33: return "Hardware Type";
        case 0x35: return "Custom Info";
        case 0x5A: return "Glue";
        default:   return "Desconocido/No soportado";
    }
}

// Nombres amistosos para algunos campos de Archive Info (0x32)
static const char* tzx_archive_field_name(uint8_t id) {
    switch (id) {
        case 0x00: return "Título";
        case 0x01: return "Editorial/Publisher";
        case 0x02: return "Autor";
        case 0x03: return "Año";
        case 0x04: return "Idioma";
        case 0x05: return "Tipo/Género";
        case 0x06: return "Precio";
        case 0x07: return "Protección";
        case 0x08: return "Origen";
        case 0x09: return "Comentario";
        default:   return "Campo";
    }
}

static void list_tzx_blocks(const char* filename) {
    FILE* f = fopen(filename, "rb"); if (!f) { printf("No se pudo abrir TZX para listar: %s\n", filename); return; }
    fseek(f, 0, SEEK_END); long fsz = ftell(f); fseek(f, 0, SEEK_SET);
    char hdr[10]={0};
    if (fread(hdr,1,10,f)<10 || memcmp(hdr,"ZXTape!\x1A",8)!=0) { printf("TZX inválido: %s\n", filename); fclose(f); return; }
    printf("=== LISTA TZX: %s (%ld bytes) v%d.%02d ===\n", filename, fsz, (unsigned char)hdr[8], (unsigned char)hdr[9]);

    int idx=0; long file_pos=10;
    while (file_pos < fsz) {
        int id = rd_u8(f); file_pos++;
        printf("Bloque %3d: 0x%02X  %-22s", idx, id, tzx_name((uint8_t)id));
        // Intento de salto según formato conocido (solo para listar; omite parse fino):
        switch (id) {
            case 0x00: // alias 0x10
            case 0x10: { uint16_t pause=rd_u16(f); uint16_t dlen=rd_u16(f); file_pos+=4; fseek(f, dlen, SEEK_CUR); file_pos+=dlen; printf("  (pause=%ums, len=%u)\n", pause, dlen); } break;
            case 0x02: // alias 0x12
            case 0x12: { uint16_t tone=rd_u16(f); uint16_t pulses=rd_u16(f); file_pos+=4; printf("  (tone=%u, pulses=%u)\n", tone,pulses); } break;
            case 0x11: { fseek(f, 2+2+2+2+2+2+1+2, SEEK_CUR); file_pos += 2+2+2+2+2+2+1+2; uint32_t dlen=rd_u24(f); file_pos+=3; fseek(f,dlen,SEEK_CUR); file_pos+=dlen; printf("  (turbo)\n"); } break;
            case 0x13: { uint8_t n=rd_u8(f); file_pos++; fseek(f, n*2, SEEK_CUR); file_pos+=n*2; printf("  (seq=%u)\n", n); } break;
            
			case 0x14: { // Pure Data
                uint16_t zero = rd_u16(f);
                uint16_t one  = rd_u16(f);
                uint8_t  used = rd_u8(f);
                uint16_t pause= rd_u16(f);
                uint32_t dlen = rd_u24(f);
                file_pos += 2+2+1+2+3;
                fseek(f, dlen, SEEK_CUR); file_pos += dlen;
                printf("  (pure data: 0=%u 1=%u usedLast=%u pause=%u len=%u)\n",
                       zero, one, used, pause, dlen);
            } break;

			case 0x15: { fseek(f, 2+2+1, SEEK_CUR); file_pos+=2+2+1; uint32_t dlen=rd_u24(f); file_pos+=3; fseek(f,dlen,SEEK_CUR); file_pos+=dlen; printf("  (direct rec len=%u)\n", dlen); } break;
            case 0x18: { uint16_t pause=rd_u16(f); uint32_t freq=rd_u32(f); uint8_t comp=rd_u8(f); uint32_t dlen=rd_u32(f); file_pos+=2+4+1+4; fseek(f,dlen,SEEK_CUR); file_pos+=dlen; printf("  (CSW: pause=%ums, %uHz, comp=%u, data=%u)\n", pause, freq, comp, dlen); } break;
            case 0x19: { uint32_t blen=rd_u32(f); file_pos+=4; fseek(f,blen,SEEK_CUR); file_pos+=blen; printf("  (GDB len=%u)\n", blen); } break;
            case 0x20: { uint16_t ms=rd_u16(f); file_pos+=2; printf("  (pause=%u)\n", ms); } break;
            case 0x21: { uint8_t l=rd_u8(f); file_pos++; fseek(f,l,SEEK_CUR); file_pos+=l; printf("  (group)\n"); } break;
            case 0x22: { printf("\n"); } break;
            case 0x24: { uint16_t c=rd_u16(f); file_pos+=2; printf("  (loop start x%u)\n", c); } break;
            case 0x25: { printf("  (loop end)\n"); } break;
            case 0x2A: { printf("  (stop if 48K)\n"); } break;
            case 0x2B: { uint8_t lvl=rd_u8(f); file_pos++; printf("  (level=%u)\n", lvl); } break;
            case 0x30: { uint8_t l=rd_u8(f); file_pos++; fseek(f,l,SEEK_CUR); file_pos+=l; printf("  (text)\n"); } break;
            case 0x31: { uint8_t d=rd_u8(f); uint8_t l=rd_u8(f); file_pos+=2; fseek(f,l,SEEK_CUR); file_pos+=l; printf("  (message %us)\n", d); } break;

            case 0x32: { // Archive Info: listar sus campos
                uint16_t blen = rd_u16(f); file_pos += 2;
#if 1
                long end = file_pos + blen;
                if (end > fsz) end = fsz;

                if (file_pos >= end) { printf("  (archive info vacio)\n"); break; }

                uint8_t n = rd_u8(f); file_pos += 1;
                printf("  (archive info, %u campo%s)\n", n, (n==1?"":"s"));

                for (uint8_t i=0; i<n && file_pos < end; ++i) {
                    if (file_pos + 1 > end) break;
                    uint8_t tid = rd_u8(f); file_pos += 1;

                    if (file_pos + 1 > end) break;
                    uint16_t slen = rd_u8(f); file_pos += 1;

                    long remain = end - file_pos; if (remain < 0) remain = 0;
                    uint16_t toread = (slen > (uint16_t)remain) ? (uint16_t)remain : slen;

                    char* buf = (toread > 0) ? (char*)malloc((size_t)toread) : NULL;
                    if (buf && toread > 0) { size_t rd = fread(buf, 1, toread, f); (void)rd; }
                    if (toread < slen) fseek(f, slen - toread, SEEK_CUR);

                    file_pos += slen;

                    const char* fname = tzx_archive_field_name(tid);
                    if (buf && toread > 0)
                        printf("           - %s [0x%02X]: %.*s\n", fname, tid, (int)toread, buf);
                    else
                        printf("           - %s [0x%02X]: <vacío>\n", fname, tid);
                    free(buf);
                }

                if (file_pos < end) { fseek(f, end - file_pos, SEEK_CUR); file_pos = end; }
#endif
            } break;

            case 0x33: { uint8_t n=rd_u8(f); file_pos++; fseek(f, n*3, SEEK_CUR); file_pos+=n*3; printf("  (hw %u)\n", n); } break;
            case 0x35: { fseek(f, 16, SEEK_CUR); file_pos+=16; { uint32_t l=rd_u32(f); file_pos+=4; fseek(f,l,SEEK_CUR); file_pos+=l; } printf("  (custom)\n"); } break;
            case 0x5A: { uint32_t l=rd_u32(f); file_pos+=4; fseek(f,l,SEEK_CUR); file_pos+=l; printf("  (glue)\n"); } break;
            default: { printf("  (no sé saltarlo; paro listado)\n"); fclose(f); return; }
        }
        idx++;
    }
    fclose(f);
}

// ─────────────────────────────────────────────────────────────
// TAP (integrado en motor unificado) + Trazas
// ─────────────────────────────────────────────────────────────
static bool tap_read_next_block();
static void  start_block_emission(uint64_t now_cycle);
static bool  tap_ear_level_until(uint64_t now_cycle);

static bool tap_read_next_block() {
    if (!tape.f) return false;
    if (tape.file_pos >= tape.file_size) return false;

    uint8_t len_le[2];
    if (fread(len_le, 1, 2, tape.f) != 2) return false;
    tape.file_pos += 2;

    uint16_t len = (uint16_t)(len_le[0] | (len_le[1] << 8));
    if (len == 0) return false;

    free(tape.blk);
    tape.blk = (uint8_t*)malloc(len);
    if (!tape.blk) return false;

    if (fread(tape.blk, 1, len, tape.f) != len) return false;
    tape.file_pos += len;
    tape.blk_len = len;

    tape.t_pilot       = 2168;
    tape.t_sync1       = 667;
    tape.t_sync2       = 735;
    tape.t_bit0        = 855;
    tape.t_bit1        = 1710;
    tape.used_bits_last= 8;
    tape.pilot_pulses  = (tape.blk[0] == 0x00) ? 8063 : 3223;
    tape.pause_ms      = 1000;

    printf("[TAP] Nuevo bloque: len=%u flag=0x%02X pilot=%u pause=%ums\n",
           len, tape.blk[0], tape.pilot_pulses, tape.pause_ms);
    return true;
}

static void start_block_emission(uint64_t now_cycle) {
    tape.phase       = PH_PILOT;
    tape.pulses_left = tape.pilot_pulses * 2;
    tape.halfwave_ts = tape.t_pilot;
    if (tape.fmt == TAPE_FMT_TAP && tape.speed > 0.0)
        tape.halfwave_ts = (uint32_t)(tape.halfwave_ts / tape.speed);
    tape.next_edge_cycle = now_cycle + tape.halfwave_ts;
    tape.level = tape.initial_level_known ? tape.initial_level : true;
    tape.data_pos = 0;
    tape.cur_bit  = 7;
    tape.pulse_of_bit = 0;
}

static void start_pause(uint64_t now_cycle) {
    tape.phase = PH_PAUSE;
    uint32_t pause_ts = (tape.pause_ms == 0) ? MS_TO_TSTATES(0) : MS_TO_TSTATES(tape.pause_ms);
    tape.next_edge_cycle = now_cycle + pause_ts;
    tape.level = true;
}

static bool tap_ear_level_until(uint64_t now_cycle) {
    if (!tape.playing || !tape.f || tape.phase == PH_IDLE) return true;
    while (now_cycle >= tape.next_edge_cycle) {
        tape.level = !tape.level;
        switch (tape.phase) {
            case PH_PILOT:
                if (--tape.pulses_left > 0) {
                    tape.next_edge_cycle += tape.halfwave_ts;
                } else {
                    tape.phase = PH_SYNC1;
                    tape.halfwave_ts = (tape.fmt==TAPE_FMT_TAP && tape.speed>0.0) ? (uint32_t)(tape.t_sync1 / tape.speed) : tape.t_sync1;
                    tape.next_edge_cycle += tape.halfwave_ts;
                }
                break;
            case PH_SYNC1:
                tape.phase = PH_SYNC2;
                tape.halfwave_ts = (tape.fmt==TAPE_FMT_TAP && tape.speed>0.0) ? (uint32_t)(tape.t_sync2 / tape.speed) : tape.t_sync2;
                tape.next_edge_cycle += tape.halfwave_ts;
                break;
            case PH_SYNC2:
                tape.phase = PH_DATA;
                tape.data_pos = 0; tape.cur_bit = 7; tape.pulse_of_bit = 0;
                tape.cur_byte = tape.blk[tape.data_pos++];
                {
                    bool b = (tape.cur_byte & 0x80) != 0;
                    tape.halfwave_ts = halfwave_for_bit(b);
                    if (tape.fmt==TAPE_FMT_TAP && tape.speed>0.0) tape.halfwave_ts = (uint32_t)(tape.halfwave_ts / tape.speed);
                    tape.next_edge_cycle += tape.halfwave_ts;
                }
                break;
            case PH_DATA: {
                tape.pulse_of_bit ^= 1;
                if (tape.pulse_of_bit == 1) {
                    tape.next_edge_cycle += tape.halfwave_ts;
                } else {
                    if (--tape.cur_bit < 0) {
                        if (tape.data_pos >= tape.blk_len) {
                            start_pause(now_cycle);
                            break;
                        }
                        tape.cur_bit = 7;
                        tape.cur_byte = tape.blk[tape.data_pos++];
                    }
                    bool b = ((tape.cur_byte >> tape.cur_bit) & 1) != 0;
                    tape.halfwave_ts = halfwave_for_bit(b);
                    if (tape.fmt==TAPE_FMT_TAP && tape.speed>0.0) tape.halfwave_ts = (uint32_t)(tape.halfwave_ts / tape.speed);
                    tape.next_edge_cycle += tape.halfwave_ts;
                }
            } break;
            case PH_PAUSE:
                if (!tap_read_next_block()) {
                    tape.phase = PH_IDLE; tape.playing = false; tape.level = true;
                } else {
                    start_block_emission(now_cycle);
                }
                break;
            default: break;
        }
        if (tape.phase == PH_IDLE) break;
    }
    return tape.level;
}

// ─────────────────────────────────────────────────────────────
// Helpers específicos para 0x19 (Generalized Data)
// ─────────────────────────────────────────────────────────────
static inline int ceil_log2_u16(int v) {
    if (v <= 1) return 1;
    int n = 0, p = 1;
    while (p < v) { p <<= 1; n++; }
    return n;
}

typedef struct {
    uint8_t  flags;    // b0..b1: 00 edge, 01 same, 10 force low, 11 force high
    uint16_t *pulses;  // lista de medias-ondas (T-states) sin los ceros de terminación
    int      npulses;  // número de medias-ondas válidas
} tzx19_symdef_t;

// Añade o fusiona una media-onda en la secuencia de salida
static bool push_or_merge_halfwave(uint16_t **seq, int *cap, int *n, uint16_t dur, bool merge) {
    if (dur == 0) return true; // 0 en definición indica fin; no se añade
    if (merge && *n > 0) {
        uint32_t ext = (uint32_t)(*seq)[*n - 1] + (uint32_t)dur;
        (*seq)[*n - 1] = (ext > 65535) ? 65535 : (uint16_t)ext;
        return true;
    }
    if (*n >= *cap) {
        int ncap = (*cap == 0) ? 512 : (*cap * 2);
        uint16_t *tmp = (uint16_t*)realloc(*seq, sizeof(uint16_t)*ncap);
        if (!tmp) return false;
        *seq = tmp; *cap = ncap;
    }
    (*seq)[(*n)++] = dur ? dur : 1;
    return true;
}

// ─────────────────────────────────────────────────────────────
// TZX  (incluye alias 0x00→0x10, 0x02→0x12, 0x18 básico, 0x19 completo + Trazas)
// ─────────────────────────────────────────────────────────────
static bool tzx_read_and_prepare_next_block(uint64_t now);

static void tzx_prepare_standard_or_turbo(uint64_t now) {
    // Nivel base (EAR alto salvo que se especifique con 0x2B)
    tape.level = tape.initial_level_known ? tape.initial_level : true;

    // Si hay piloto, se emite; si no, saltamos a SYNC o DATA
    if (tape.pilot_pulses > 0 && tape.t_pilot > 0) {
        tape.phase       = PH_PILOT;
        tape.pulses_left = tape.pilot_pulses * 2;   // medias ondas
        tape.halfwave_ts = tape.t_pilot;
        tape.next_edge_cycle = now + tape.halfwave_ts;
    } else if (tape.t_sync1 > 0) {
        tape.phase       = PH_SYNC1;
        tape.halfwave_ts = tape.t_sync1;
        tape.next_edge_cycle = now + tape.halfwave_ts;
    } else if (tape.t_bit0 || tape.t_bit1) {
        tape.phase      = PH_DATA;
        tape.data_pos   = 0;
        tape.cur_bit    = 7;
        tape.pulse_of_bit = 0;
        tape.cur_byte   = (tape.blk_len > 0) ? tape.blk[tape.data_pos++] : 0x00;
        bool b          = (tape.cur_byte & 0x80) != 0;
        tape.halfwave_ts= halfwave_for_bit(b);
        tape.next_edge_cycle = now + tape.halfwave_ts;
    } else {
        // No hay nada que emitir: solo pausa (si existe)
        tape.phase = PH_PAUSE;
        tape.next_edge_cycle = now + MS_TO_TSTATES(tape.pause_ms);
    }
}

static bool tzx_ear_level_until(uint64_t now_cycle) {
    if (!tape.playing || !tape.f || tape.phase == PH_IDLE) return true;

    while (now_cycle >= tape.next_edge_cycle) {
        // En PAUSE no hay flancos: nivel estable
        if (tape.phase != PH_PAUSE) {
            tape.level = !tape.level;
        }

        switch (tape.phase) {
            case PH_PILOT:
            case PH_PURE_TONE:
                if (--tape.pulses_left > 0) {
                    tape.next_edge_cycle += tape.halfwave_ts;
                } else {
                    if (tape.t_sync1) {
                        tape.phase = PH_SYNC1;
                        tape.halfwave_ts = tape.t_sync1;
                        tape.next_edge_cycle += tape.halfwave_ts;
                    } else {
                        if (tape.t_bit0 || tape.t_bit1) {
                            tape.phase = PH_DATA;
                            tape.data_pos = 0; tape.cur_bit = 7; tape.pulse_of_bit = 0;
                            tape.cur_byte = (tape.blk_len > 0) ? tape.blk[tape.data_pos++] : 0x00;
                            bool b = (tape.cur_byte & 0x80) != 0;
                            tape.halfwave_ts = halfwave_for_bit(b);
                            tape.next_edge_cycle += tape.halfwave_ts;
                        } else {
                            tape.phase = PH_PAUSE;
                            tape.next_edge_cycle += MS_TO_TSTATES(tape.pause_ms);
                        }
                    }
                }
                break;

            case PH_SYNC1:
                tape.phase = tape.t_sync2 ? PH_SYNC2 : PH_DATA;
                if (tape.phase == PH_SYNC2) {
                    tape.halfwave_ts = tape.t_sync2;
                    tape.next_edge_cycle += tape.halfwave_ts;
                } else {
                    tape.data_pos = 0; tape.cur_bit = 7; tape.pulse_of_bit = 0;
                    tape.cur_byte = (tape.blk_len > 0) ? tape.blk[tape.data_pos++] : 0x00;
                    bool b = (tape.cur_byte & 0x80) != 0;
                    tape.halfwave_ts = halfwave_for_bit(b);
                    tape.next_edge_cycle += tape.halfwave_ts;
                }
                break;

            case PH_SYNC2:
                tape.phase = PH_DATA;
                tape.data_pos = 0; tape.cur_bit = 7; tape.pulse_of_bit = 0;
                tape.cur_byte = (tape.blk_len > 0) ? tape.blk[tape.data_pos++] : 0x00;
                {
                    bool b = (tape.cur_byte & 0x80) != 0;
                    tape.halfwave_ts = halfwave_for_bit(b);
                    tape.next_edge_cycle += tape.halfwave_ts;
                }
                break;

            case PH_DATA: {
				tape.pulse_of_bit ^= 1;
				if (tape.pulse_of_bit == 1) {
					tape.next_edge_cycle += tape.halfwave_ts;
				} else {
					if (--tape.cur_bit < 0) {
						if (tape.data_pos >= tape.blk_len) {
							tape.phase = PH_PAUSE;
							tape.next_edge_cycle += MS_TO_TSTATES(tape.pause_ms);
							break;
						}
						tape.cur_bit = 7;
						tape.cur_byte = tape.blk[tape.data_pos++];
					}

					// Parada precisa en el último byte, usando used_bits_last:
					if (tape.data_pos == tape.blk_len && tape.used_bits_last && tape.used_bits_last != 8) {
						int emitted_bits = 7 - tape.cur_bit;
						if (emitted_bits >= tape.used_bits_last) {
							tape.phase = PH_PAUSE;
							tape.next_edge_cycle += MS_TO_TSTATES(tape.pause_ms);
							break;
						}
					}

					bool b = ((tape.cur_byte >> tape.cur_bit) & 1) != 0;
					tape.halfwave_ts = halfwave_for_bit(b);
					tape.next_edge_cycle += tape.halfwave_ts;
				}
			} break;

            case PH_PULSE_SEQ:
                if (tape.pulse_seq_i < tape.pulse_seq_n) {
                    tape.halfwave_ts = tape.pulse_seq[tape.pulse_seq_i++];
                    tape.next_edge_cycle += tape.halfwave_ts;
                } else {
                    tape.phase = PH_PAUSE;
                    tape.next_edge_cycle += MS_TO_TSTATES(tape.pause_ms);
                }
                break;

            case PH_DIRECT_REC: {
                if (tape.dr_bit_index >= tape.dr_total_bits) {
                    tape.phase = PH_PAUSE;
                    tape.next_edge_cycle += MS_TO_TSTATES(tape.pause_ms);
                    break;
                }
                uint32_t byte_i = tape.dr_bit_index >> 3;
                int      bit_i  = 7 - (tape.dr_bit_index & 7);
                uint8_t  b      = tape.blk[byte_i];
                bool     lvl    = ((b >> bit_i) & 1) != 0;
                tape.next_edge_cycle += tape.dr_tstates_per_sample;
                if (lvl != tape.level) { /* mantener el toggle ya aplicado */ }
                else { tape.level = !tape.level; } // revertir toggle artificial
                tape.dr_bit_index++;
            } break;

            case PH_PAUSE:
                // Pausa consumida → siguiente bloque (sin conmutar nivel)
                if (!tzx_read_and_prepare_next_block(now_cycle)) {
                    tape.phase = PH_IDLE; tape.playing = false; tape.level = true;
                }
                break;

            case PH_IDLE:
            default:
                return tape.level;
        }
        if (tape.phase == PH_IDLE) break;
    }
    return tape.level;
}

static bool tzx_read_and_prepare_next_block(uint64_t now) {
    if (tape.file_pos >= tape.file_size) return false;
    int id = rd_u8(tape.f); tape.file_pos++;

    switch (id) {
        // ── Aliases legacy: 0x00→0x10, 0x02→0x12
        case 0x00: // Standard Speed Data (legacy)
            printf("[TZX] Bloque 0x00 (alias 0x10 Standard Speed)\n");
            // fall-through a 0x10
        case 0x10: { // Standard Speed Data
            tape.pause_ms = rd_u16(tape.f);  tape.file_pos += 2;
            uint16_t dlen = rd_u16(tape.f);  tape.file_pos += 2;
            free(tape.blk); tape.blk = (uint8_t*)malloc(dlen);
            if (!tape.blk) return false;
            fread(tape.blk, 1, dlen, tape.f); tape.file_pos += dlen;
            tape.blk_len = dlen;
            tape.t_pilot = 2168; tape.t_sync1 = 667; tape.t_sync2 = 735;
            tape.t_bit0  = 855;  tape.t_bit1  = 1710;
            tape.used_bits_last = 8;
            tape.pilot_pulses = (tape.blk_len>0 && tape.blk[0]==0x00) ? 8063 : 3223;

            printf("[TZX] 0x10 std: pause=%ums len=%u pilot=%u\n", tape.pause_ms, dlen, tape.pilot_pulses);
            tzx_prepare_standard_or_turbo(now);
        } return true;

        case 0x02: // Pure Tone (legacy)
            printf("[TZX] Bloque 0x02 (alias 0x12 Pure Tone)\n");
            // fall-through a 0x12
    
        case 0x12: { // Pure Tone
            uint16_t tone   = rd_u16(tape.f);  tape.file_pos += 2; // duración media-onda (T-states)
            uint16_t pulses = rd_u16(tape.f);  tape.file_pos += 2; // # de ondas completas

            printf("[TZX] 0x12 pure-tone: tone=%u pulses=%u\n", tone, pulses);

            // Si no hay nada que emitir, saltamos directamente al siguiente bloque
            if (tone == 0 || pulses == 0) {
                return tzx_read_and_prepare_next_block(now);
            }

            // Este bloque NO tiene pausa implícita
            tape.pause_ms = 0;

            // Limpia sincronías y datos: el bloque es sólo un tono puro
            tape.t_sync1 = tape.t_sync2 = 0;
            tape.t_bit0  = tape.t_bit1  = 0;
            tape.pilot_pulses = 0;

            // Preparar emisión de tono puro (en medias ondas)
            tape.phase       = PH_PURE_TONE;
            tape.pulses_left = pulses * 2;             // ondas completas => 2 medias-ondas por onda
            tape.halfwave_ts = tone;
            tape.level       = tape.initial_level_known ? tape.initial_level : true;
            tape.next_edge_cycle = now + (tape.halfwave_ts ? tape.halfwave_ts : 1);

        } return true;

        case 0x11: { // Turbo
            tape.t_pilot = rd_u16(tape.f);      tape.file_pos += 2;
            tape.t_sync1 = rd_u16(tape.f);      tape.file_pos += 2;
            tape.t_sync2 = rd_u16(tape.f);      tape.file_pos += 2;
            tape.t_bit0  = rd_u16(tape.f);      tape.file_pos += 2;
            tape.t_bit1  = rd_u16(tape.f);      tape.file_pos += 2;
            tape.pilot_pulses = rd_u16(tape.f); tape.file_pos += 2;            
            { uint8_t u = rd_u8(tape.f); tape.used_bits_last = (u == 0) ? 8 : u; } tape.file_pos += 1;
            tape.pause_ms = rd_u16(tape.f);     tape.file_pos += 2;
            uint32_t dlen = rd_u24(tape.f);     tape.file_pos += 3;
            free(tape.blk); tape.blk = (uint8_t*)malloc(dlen);
            if (!tape.blk) return false;
            fread(tape.blk, 1, dlen, tape.f); tape.file_pos += dlen;
            tape.blk_len = dlen;
            printf("[TZX] 0x11 turbo: len=%u pilot=%u bit0=%u bit1=%u usedLast=%u pause=%u\n",
                   dlen, tape.pilot_pulses, tape.t_bit0, tape.t_bit1, tape.used_bits_last, tape.pause_ms);
            tzx_prepare_standard_or_turbo(now);
        } return true;

		case 0x13: { // Pulse Sequence
            uint8_t n = rd_u8(tape.f); 
            tape.file_pos += 1;

            // Sin pausa implícita en 0x13 (según especificación)
            tape.pause_ms = 0;

            // Si no hay pulsos, saltamos inmediatamente al siguiente bloque
            if (n == 0) {
                printf("[TZX] 0x13 pulse-seq: vacío → continuar\n");
                return tzx_read_and_prepare_next_block(now);
            }

            // Preparar/leer la secuencia de medias-ondas
            free(tape.pulse_seq);
            tape.pulse_seq = (uint16_t*)malloc(sizeof(uint16_t) * n);
            if (!tape.pulse_seq) return false;

            for (uint8_t i = 0; i < n; ++i) {
                uint16_t d = rd_u16(tape.f); 
                tape.file_pos += 2;
                // Defensa: evitar duración 0 (bloquearía el avance de tiempo)
                if (d == 0) d = 1;
                tape.pulse_seq[i] = d;
            }

            tape.pulse_seq_n = n;
            tape.pulse_seq_i = 0;

            // Este bloque no usa tape.blk; liberamos por limpieza
            free(tape.blk);
            tape.blk = NULL;
            tape.blk_len = 0;

            // Nivel inicial: 0x2B (Set Signal Level) si se usó antes; si no, alto
            tape.level = tape.initial_level_known ? tape.initial_level : true;

            // Activar reproducción como secuencia de medias-ondas
            tape.phase = PH_PULSE_SEQ;
            tape.halfwave_ts = (n > 0) ? tape.pulse_seq[0] : 1;
            tape.next_edge_cycle = now + tape.halfwave_ts;

            printf("[TZX] 0x13 pulse-seq: pulses=%u (sin pausa)\n", n);
        } return true;


        case 0x14: { // Pure Data (sin piloto/sync)
			tape.t_bit0  = rd_u16(tape.f);      tape.file_pos += 2;
			tape.t_bit1  = rd_u16(tape.f);      tape.file_pos += 2;
			uint8_t u    = rd_u8(tape.f);       tape.file_pos += 1;
			tape.used_bits_last = (u == 0) ? 8 : u;
			tape.pause_ms = rd_u16(tape.f);     tape.file_pos += 2;
			uint32_t dlen = rd_u24(tape.f);     tape.file_pos += 3;

			free(tape.blk); tape.blk = (uint8_t*)malloc(dlen);
			if (!tape.blk) return false;
			fread(tape.blk, 1, dlen, tape.f); tape.file_pos += dlen;
			tape.blk_len = dlen;

			// Sin piloto ni sync en 0x14, solo datos puros.
			tape.t_pilot = 0; tape.t_sync1 = 0; tape.t_sync2 = 0; tape.pilot_pulses = 0;

			tape.phase = PH_DATA;
			tape.data_pos = 0;
			tape.cur_bit = 7;
			tape.pulse_of_bit = 0;
			tape.cur_byte = (dlen > 0) ? tape.blk[tape.data_pos++] : 0;
			bool b = (tape.cur_byte & 0x80) != 0;
			tape.halfwave_ts = halfwave_for_bit(b);
			tape.level = tape.initial_level_known ? tape.initial_level : true;
			tape.next_edge_cycle = now + tape.halfwave_ts;
			printf("[TZX] 0x14 pure data: len=%u bit0=%u bit1=%u usedLast=%u pause=%u\n",
				   dlen, tape.t_bit0, tape.t_bit1, tape.used_bits_last, tape.pause_ms);
			return true;
		}

        case 0x15: { // Direct Recording
            tape.dr_tstates_per_sample = rd_u16(tape.f); tape.file_pos += 2;
            tape.pause_ms = rd_u16(tape.f);  tape.file_pos += 2;
            uint8_t used_last = rd_u8(tape.f); tape.file_pos += 1;
            uint32_t dlen = rd_u24(tape.f);  tape.file_pos += 3;
            free(tape.blk); tape.blk=(uint8_t*)malloc(dlen);
            if (!tape.blk) return false;
            fread(tape.blk, 1, dlen, tape.f); tape.file_pos += dlen;
            tape.blk_len = dlen;
            tape.dr_total_bits = (dlen-1)*8 + ((used_last==0)? 8 : used_last);
            tape.dr_bit_index = 0;
            tape.phase = PH_DIRECT_REC;
            tape.level = tape.initial_level_known ? tape.initial_level : true;
            tape.next_edge_cycle = now + tape.dr_tstates_per_sample;
            printf("[TZX] 0x15 direct-rec: bitTs=%u pause=%u len=%u usedLast=%u\n",
                   tape.dr_tstates_per_sample, tape.pause_ms, dlen, used_last);
        } return true;

        case 0x18: { // CSW Recording (soporte base: compresión=0 raw)
            uint16_t pause_ms = rd_u16(tape.f); tape.file_pos += 2;
            uint32_t freq_hz  = rd_u32(tape.f); tape.file_pos += 4;
            uint8_t  comp     = rd_u8(tape.f);  tape.file_pos += 1;  // 0=raw sin compresión (asumido)
            uint32_t data_len = rd_u32(tape.f); tape.file_pos += 4;

            free(tape.blk); tape.blk = (uint8_t*)malloc(data_len);
            if (!tape.blk) return false;
            fread(tape.blk,1,data_len,tape.f); tape.file_pos += data_len;

            tape.pause_ms = pause_ms;
            tape.csw_freq_hz = freq_hz;
            tape.csw_compression = comp;
            tape.csw_data_len = data_len;

            // Convertimos CSW raw a secuencia de medias ondas si compresión=0:
            if (comp == 0 && data_len >= 2) {
                uint32_t pairs = data_len / 2;
                free(tape.pulse_seq); tape.pulse_seq = (uint16_t*)malloc(sizeof(uint16_t)*pairs*4);
                if (!tape.pulse_seq) return false;

                int n=0;
                for (uint32_t i=0;i<pairs;i++) {
                    uint16_t samples = (uint16_t)(tape.blk[2*i] | (tape.blk[2*i+1]<<8));
                    if (samples==0) continue; // ignora silencios nulos
                    uint32_t ts = (uint32_t)(((uint64_t)samples * 3500000ULL) / (freq_hz ? freq_hz : 1));
                    if (ts==0) ts=1;
                    while (ts > 0) {
                        uint16_t chunk = (ts > 65535) ? 65535 : (uint16_t)ts;
                        tape.pulse_seq[n++] = chunk;
                        ts -= chunk;
                    }
                }
                tape.pulse_seq_n = n;
                tape.pulse_seq_i = 0;
                tape.phase = PH_PULSE_SEQ;
                tape.halfwave_ts = (n>0)? tape.pulse_seq[0] : 1;
                tape.level = tape.initial_level_known ? tape.initial_level : true;
                tape.next_edge_cycle = now + tape.halfwave_ts;

                printf("[TZX] 0x18 CSW(raw): pause=%ums freq=%uHz pulses=%d (from %u bytes)\n",
                       pause_ms, freq_hz, n, data_len);
            } else {
                printf("[TZX] 0x18 CSW comp=%u NO soportado; se salta (pause=%ums, data=%u)\n",
                       comp, pause_ms, data_len);
                tape.phase = PH_PAUSE;
                tape.next_edge_cycle = now + MS_TO_TSTATES(pause_ms);
                tape.level = true;
            }
        } return true;

        case 0x19: { // Generalized Data Block (implementación completa)
            uint32_t blen = rd_u32(tape.f); tape.file_pos += 4;
            long block_end = tape.file_pos + blen;

            tape.pause_ms = rd_u16(tape.f);        tape.file_pos += 2;
            uint32_t TOTP  = rd_u32(tape.f);        tape.file_pos += 4; // pilot/sync total symbols
            uint8_t  NPP   = rd_u8(tape.f);         tape.file_pos += 1; // max pulses per pilot/sync symbol
            uint8_t  ASPx  = rd_u8(tape.f);         tape.file_pos += 1; // alphabet size (0=256)
            uint32_t TOTD  = rd_u32(tape.f);        tape.file_pos += 4; // data total symbols
            uint8_t  NPD   = rd_u8(tape.f);         tape.file_pos += 1; // max pulses per data symbol
            uint8_t  ASDx  = rd_u8(tape.f);         tape.file_pos += 1; // alphabet size (0=256)

            int ASP = (ASPx == 0) ? 256 : ASPx;
            int ASD = (ASDx == 0) ? 256 : ASDx;

            // Salida: secuencia de medias ondas
            uint16_t *seq = NULL; int seq_cap = 0, seq_n = 0;

            // Nivel inicial
            bool init_level = tape.initial_level_known ? tape.initial_level : true;

            // Tablas de símbolos para Pilot/Sync
            tzx19_symdef_t *pilot = NULL;
            if (TOTP > 0) {
                pilot = (tzx19_symdef_t*)calloc((size_t)ASP, sizeof(tzx19_symdef_t));
                if (!pilot) goto tzx19_fail;

                for (int i = 0; i < ASP; ++i) {
                    pilot[i].flags = rd_u8(tape.f); tape.file_pos += 1;
                    pilot[i].pulses = (uint16_t*)malloc(sizeof(uint16_t) * NPP);
                    if (!pilot[i].pulses) goto tzx19_fail;
                    pilot[i].npulses = 0;
                    for (int j = 0; j < NPP; ++j) {
                        uint16_t d = rd_u16(tape.f); tape.file_pos += 2;
                        if (d) pilot[i].pulses[pilot[i].npulses++] = d;
                    }
                }

                // PRLE (TOTP entradas): símbolo + repeticiones
                for (uint32_t k = 0; k < TOTP; ++k) {
                    uint8_t sym = rd_u8(tape.f);     tape.file_pos += 1;
                    uint16_t rep = rd_u16(tape.f);   tape.file_pos += 2;
                    if (sym >= ASP) sym %= ASP;

                    for (uint16_t r = 0; r < rep; ++r) {
                        tzx19_symdef_t *S = &pilot[sym];

                        uint8_t pol = (S->flags & 0x03);

                        // nivel actual según sec_n
                        bool current_level = (seq_n % 2 == 0) ? init_level : !init_level;

                        bool merge_first = false;
                        if      (pol == 0x01) merge_first = (seq_n > 0);                     // same level → sin flanco
                        else if (pol == 0x02) merge_first = (seq_n > 0) && (current_level == false); // force low
                        else if (pol == 0x03) merge_first = (seq_n > 0) && (current_level == true);  // force high

                        for (int p = 0; p < S->npulses; ++p) {
                            if (!push_or_merge_halfwave(&seq, &seq_cap, &seq_n, S->pulses[p], (p==0) && merge_first))
                                goto tzx19_fail;
                        }
                    }
                }
            }

            // Tabla de símbolos de datos
            tzx19_symdef_t *data = NULL;
            uint32_t bytes_consumed = 0;
            if (TOTD > 0) {
                data = (tzx19_symdef_t*)calloc((size_t)ASD, sizeof(tzx19_symdef_t));
                if (!data) goto tzx19_fail;

                for (int i = 0; i < ASD; ++i) {
                    data[i].flags = rd_u8(tape.f); tape.file_pos += 1;
                    data[i].pulses = (uint16_t*)malloc(sizeof(uint16_t) * NPD);
                    if (!data[i].pulses) goto tzx19_fail;
                    data[i].npulses = 0;
                    for (int j = 0; j < NPD; ++j) {
                        uint16_t d = rd_u16(tape.f); tape.file_pos += 2;
                        if (d) data[i].pulses[data[i].npulses++] = d;
                    }
                }

                int NB = ceil_log2_u16(ASD);
                uint32_t DS = (uint32_t)((NB * (uint64_t)TOTD + 7) / 8);

                uint8_t cur = 0; int rem_bits = 0;

                for (uint32_t k = 0; k < TOTD; ++k) {
                    uint32_t sym = 0;
                    for (int i = 0; i < NB; ++i) {
                        if (rem_bits == 0) {
                            cur = rd_u8(tape.f); tape.file_pos += 1;
                            rem_bits = 8;
                            bytes_consumed++;
                        }
                        sym = (sym << 1) | ((cur >> (rem_bits - 1)) & 1u);
                        rem_bits--;
                    }

                    if (sym >= (uint32_t)ASD) sym %= ASD; // defensa
                    tzx19_symdef_t *S = &data[sym];

                    uint8_t pol = (S->flags & 0x03);
                    bool current_level = (seq_n % 2 == 0) ? init_level : !init_level;

                    bool merge_first = false;
                    if      (pol == 0x01) merge_first = (seq_n > 0);
                    else if (pol == 0x02) merge_first = (seq_n > 0) && (current_level == false);
                    else if (pol == 0x03) merge_first = (seq_n > 0) && (current_level == true);

                    for (int p = 0; p < S->npulses; ++p) {
                        if (!push_or_merge_halfwave(&seq, &seq_cap, &seq_n, S->pulses[p], (p==0) && merge_first))
                            goto tzx19_fail;
                    }
                }

                // Saltar bytes de padding (si los hubiera) hasta consumir DS
                if (bytes_consumed < DS) {
                    uint32_t skip = DS - bytes_consumed;
                    fseek(tape.f, skip, SEEK_CUR);
                    tape.file_pos += skip;
                }
            } else {
                // Si no hay TOTD, saltar a block_end si queda
                if (tape.file_pos < block_end) {
                    fseek(tape.f, block_end - tape.file_pos, SEEK_CUR);
                    tape.file_pos = block_end;
                }
            }

            // Liberar tablas auxiliares
            if (pilot) {
                for (int i = 0; i < ((TOTP>0)? ((ASPx==0)?256:ASPx) : 0); ++i) free(pilot[i].pulses);
                free(pilot);
            }
            if (data) {
                for (int i = 0; i < ((TOTD>0)? ((ASDx==0)?256:ASDx) : 0); ++i) free(data[i].pulses);
                free(data);
            }

            // Activar emisión como secuencia de medias ondas
            free(tape.pulse_seq);
            tape.pulse_seq   = seq;
            tape.pulse_seq_n = seq_n;
            tape.pulse_seq_i = 0;

            tape.phase = PH_PULSE_SEQ;
            tape.halfwave_ts = (seq_n > 0) ? tape.pulse_seq[0] : 1;
            tape.level = init_level;
            tape.next_edge_cycle = now + tape.halfwave_ts;

            printf("[TZX] 0x19 GDB: pulses=%d pause=%ums (ASP=%d,NPP=%d; ASD=%d,NPD=%d; TOTP=%u; TOTD=%u)\n",
                   seq_n, tape.pause_ms, (ASPx==0)?256:ASPx, NPP, (ASDx==0)?256:ASDx, NPD, TOTP, TOTD);

            return true;

tzx19_fail:
            fprintf(stderr, "[TZX] 0x19: error de memoria o lectura; se intenta continuar.\n");
            if (pilot) { for (int i = 0; i < ((ASPx==0)?256:ASPx); ++i) free(pilot[i].pulses); free(pilot); }
            if (data)  { for (int i = 0; i < ((ASDx==0)?256:ASDx); ++i) free(data[i].pulses);  free(data);  }
            free(seq);
            // Saltar al final del bloque e intentar seguir
            fseek(tape.f, block_end, SEEK_SET); tape.file_pos = block_end;
            return tzx_read_and_prepare_next_block(now);
        }

        case 0x20: { // Pause
            uint16_t ms = rd_u16(tape.f); tape.file_pos += 2;
            if (ms == 0) { tape.phase = PH_IDLE; tape.playing = false; tape.level = true; printf("[TZX] 0x20 pause=0 (stop)\n"); return false; }
            tape.pause_ms = ms;
            tape.phase = PH_PAUSE; tape.next_edge_cycle = now + MS_TO_TSTATES(ms); tape.level = true;
            printf("[TZX] 0x20 pause=%u\n", ms);
        } return true;

        case 0x21: { // Group start (informativo)
            uint8_t ln = rd_u8(tape.f); tape.file_pos += 1;
            char name[ln];
            int rd = ln;//(ln < 255) ? ln : 255;
            if (rd > 0) fread(name, 1, rd, tape.f);
            tape.file_pos += rd;
            name[ (rd>0)? rd : 0 ] = 0;
            //if (ln > rd) { fseek(tape.f, ln - rd, SEEK_CUR); tape.file_pos += (ln - rd); }

            if (tape.group_depth == 0) tape.group_depth = 1;
            else fprintf(stderr, "[TZX] 0x21: grupo anidado no permitido por la spec.\n");
            printf("[TZX] 0x21 group-start: \"%s\"\n", name);
			
			//tape.file_pos += 1;

            return tzx_read_and_prepare_next_block(now);
        }

        case 0x22: { // Group end (informativo)
            if (tape.group_depth > 0) tape.group_depth = 0;
            printf("[TZX] 0x22 group-end\n");
            return tzx_read_and_prepare_next_block(now);
        }

        case 0x24: { uint16_t count = rd_u16(tape.f); tape.file_pos += 2; tape.loop.file_pos_at_loop = ftell(tape.f); tape.loop.remaining = count; tape.loop.active = 1; printf("[TZX] 0x24 loop-start x%u\n", count); return tzx_read_and_prepare_next_block(now); }
        case 0x25: { printf("[TZX] 0x25 loop-end (remain=%u)\n", tape.loop.remaining); if (tape.loop.active && tape.loop.remaining > 1) { tape.loop.remaining--; fseek(tape.f, tape.loop.file_pos_at_loop, SEEK_SET); tape.file_pos = tape.loop.file_pos_at_loop; return tzx_read_and_prepare_next_block(now); } else { tape.loop.active = 0; return tzx_read_and_prepare_next_block(now);} }

        case 0x2A: { tape.phase = PH_IDLE; tape.playing = false; tape.level = true; printf("[TZX] 0x2A stop-if-48K → STOP\n"); return false; }
        case 0x2B: { uint8_t lvl = rd_u8(tape.f); tape.file_pos += 1; tape.initial_level_known = true; tape.initial_level = (lvl != 0); printf("[TZX] 0x2B set-level=%u\n", lvl); return tzx_read_and_prepare_next_block(now); }

        case 0x30: { uint8_t ln = rd_u8(tape.f); tape.file_pos += 1; fseek(tape.f, ln, SEEK_CUR); tape.file_pos += ln; printf("[TZX] 0x30 text\n"); return tzx_read_and_prepare_next_block(now); }
        case 0x31: { uint8_t dur = rd_u8(tape.f); uint8_t ln = rd_u8(tape.f); tape.file_pos += 2; fseek(tape.f, ln, SEEK_CUR); tape.file_pos += ln; printf("[TZX] 0x31 message %us\n", dur); return tzx_read_and_prepare_next_block(now); }

        case 0x32: { // Archive Info (metadatos; se muestra y se continúa)
            uint16_t blen = rd_u16(tape.f); tape.file_pos += 2;
            printf("Longitud bloque completo: %d\n", blen);
            long end = tape.file_pos + blen;
            printf("Posicion final: %ld\n", end);
            if (end > tape.file_size) end = tape.file_size;

            printf("[TZX] 0x32 archive-info:\n");
#if 1
            if (tape.file_pos >= end) { printf("       (vacío)\n"); return tzx_read_and_prepare_next_block(now); }

            uint8_t n = rd_u8(tape.f); tape.file_pos += 1;
            printf("       %u campo%s\n", n, (n==1?"":"s"));

            for (uint8_t i = 0; (i < n) && (tape.file_pos < end); ++i) {
                if (tape.file_pos + 1 > end) break;
                uint8_t tid = rd_u8(tape.f); tape.file_pos += 1;

                if (tape.file_pos + 1 > end) break;
                uint8_t slen = rd_u8(tape.f); tape.file_pos += 1;

                long remain = end - tape.file_pos; if (remain < 0) remain = 0;
                uint16_t toread = (slen > (uint16_t)remain) ? (uint16_t)remain : slen;

                char* buf = (toread > 0) ? (char*)malloc((size_t)toread) : NULL;
                if (buf && toread > 0) { size_t rd = fread(buf, 1, toread, tape.f); (void)rd; }
                if (toread < slen) fseek(tape.f, slen - toread, SEEK_CUR);

                tape.file_pos += slen;

                const char* fname = tzx_archive_field_name(tid);
                if (buf && toread > 0)
                    printf("       - %s [0x%02X]: %.*s\n", fname, tid, (int)toread, buf);
                else
                    printf("       - %s [0x%02X]: <vacío>\n", fname, tid);

                free(buf);
            }

            if (tape.file_pos < end) { fseek(tape.f, end - tape.file_pos, SEEK_CUR); tape.file_pos = end; }
#endif
            // (NO sumar tape.file_pos += blen; ya hemos posicionado a end)
            return tzx_read_and_prepare_next_block(now);
        }

        case 0x33: { uint8_t n = rd_u8(tape.f); tape.file_pos += 1; fseek(tape.f, n*3, SEEK_CUR); tape.file_pos += n*3; printf("[TZX] 0x33 hardware x%u\n", n); return tzx_read_and_prepare_next_block(now); }
        case 0x35: { fseek(tape.f, 16, SEEK_CUR); tape.file_pos += 16; { uint32_t ln = rd_u32(tape.f); tape.file_pos += 4; fseek(tape.f, ln, SEEK_CUR); tape.file_pos += ln; } printf("[TZX] 0x35 custom\n"); return tzx_read_and_prepare_next_block(now); }
        case 0x5A: { uint32_t ln = rd_u32(tape.f); tape.file_pos += 4; fseek(tape.f, ln, SEEK_CUR); tape.file_pos += ln; printf("[TZX] 0x5A glue\n"); return tzx_read_and_prepare_next_block(now); }

        default:
            fprintf(stderr, "[TZX] Bloque 0x%02X no soportado.\n", id);
            return false;
    }
}

bool load_tzx(const char* filename) {
    list_tzx_blocks(filename); // listado completo al cargar

    if (tape.f) { fclose(tape.f); tape.f = NULL; }
    tape.f = fopen(filename, "rb");
    if (!tape.f) { fprintf(stderr, "No se pudo abrir %s\n", filename); return false; }

    fseek(tape.f, 0, SEEK_END); tape.file_size = ftell(tape.f);
    fseek(tape.f, 0, SEEK_SET); tape.file_pos = 0;

    char hdr[10]={0};
    if (fread(hdr,1,10,tape.f) < 10 || memcmp(hdr,"ZXTape!\x1A",8) != 0) {
        fprintf(stderr, "TZX: cabecera inválida.\n");
        fclose(tape.f); tape.f = NULL; return false;
    }
    tape.file_pos += 10;

    free(tape.blk); tape.blk=NULL;
    free(tape.pulse_seq); tape.pulse_seq=NULL;
    tape.blk_len = 0;
    tape.fmt = TAPE_FMT_TZX;
    tape.playing = false;
    tape.initial_level_known = false;
    tape.loop.active = 0;
    tape.group_depth = 0;

    if (!tzx_read_and_prepare_next_block(global_cycles)) { /*tape.playing=false;*/ return false; }
    border_color = 7;

    printf("TZX cargado: %s (%ld bytes) v%d.%02d\n", filename, tape.file_size, (unsigned char)hdr[8], (unsigned char)hdr[9]);
    tape_filename = filename;
    return true;
}

// ─────────────────────────────────────────────────────────────
// Carga TAP
// ─────────────────────────────────────────────────────────────
bool load_tap(const char* filename) {
    list_tap_blocks(filename); // listado completo al cargar

    if (tape.f) { fclose(tape.f); tape.f = NULL; }
    tape.f = fopen(filename, "rb");
    if (!tape.f) { printf("No se pudo abrir %s\n", filename); tape.playing = false; return false; }

    fseek(tape.f, 0, SEEK_END); tape.file_size = ftell(tape.f);
    fseek(tape.f, 0, SEEK_SET); tape.file_pos = 0;

    free(tape.blk); tape.blk = NULL;
    free(tape.pulse_seq); tape.pulse_seq = NULL;
    tape.blk_len = 0;
    tape.fmt = TAPE_FMT_TAP;
    tape.speed   = 1.0;
    tape.playing = true;
    tape.initial_level_known = false;

    if (!tap_read_next_block()) { printf("TAP vacío.\n"); tape.playing = false; return false; }
    start_block_emission(global_cycles);
    border_color = 7;

    printf("TAP cargado: %s (%ld bytes)\n", filename, tape.file_size);
    tape_filename = filename;
    return true;
}


// ─────────────────────────────────────────────────────────────
// Selector unificado EAR
// ─────────────────────────────────────────────────────────────
static inline bool get_current_ear_level_from_tape(void) {
    if (tape.fmt == TAPE_FMT_TZX) return tzx_ear_level_until(global_cycles);
    if (tape.fmt == TAPE_FMT_TAP) return tap_ear_level_until(global_cycles);
    return true; // sin cinta → EAR alto
}


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
//static unsigned int_recalc;
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
/* TSTATES_CPU is defined in tape.h */

static SDL_AudioDeviceID audio_dev = 0;
static SDL_AudioSpec have;
static int audio_rate = 44100;
static float beeper_volume = 0.30f;
static float tape_volume   = 0.15f;  /* volume for tape EAR-in signal */
static float ay_volume     = 0.50f;  /* volume for AY-3-8912 output */

static uint64_t beeper_frame_origin = 0;
static uint64_t beeper_slice_origin = 0;
static uint64_t beeper_last_tstate  = 0;
static int      beeper_level        = 0;   /* 0 o 1 (onda cuadrada) */
static int      tape_ear_level      = 0;   /* 0 o 1: EAR input from tape/TZX */
static int      tape_ear_active     = 0;   /* 1 when tape/TZX is playing */
static double   beeper_frac_acc     = 0.0;

/* AY-3-8912 PSG (128K/+3 only; NULL on 48K). */
static ay8912_t *ay = NULL;

static int audio_init_sdl(int rate)
{
    SDL_AudioSpec want;
    SDL_zero(want);
    want.freq = rate;
    want.format = AUDIO_S16SYS;
    want.channels = 1;
    want.samples = 512;

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

    double add = (double)dt * (((double)audio_rate / TSTATES_CPU)/1);
    beeper_frac_acc += add;
    int nsamp = (int)beeper_frac_acc;
    if (nsamp <= 0) return;
    beeper_frac_acc -= nsamp;

    /* Mix beeper (EAR/MIC out) with tape EAR-in signal.
     * Gate the tape contribution so silence when nothing is playing. */
    float bv = beeper_level ? beeper_volume : 0;
    tape_ear_level = get_current_ear_level_from_tape();
	float tv = tape_ear_active ? (tape_ear_level ? tape_volume : -tape_volume) : 0.0f;

	if (!fast)
	{
			
		enum { CHUNK = 4096 };
		static int16_t buf[CHUNK];
		while (nsamp > 0) {
			int n = (nsamp > CHUNK) ? CHUNK : nsamp;
			if (ay) {
				/* 128K/+3: step the AY PSG once per sample and mix. */
				for (int i = 0; i < n; ++i) {
					float mixed = bv + tv +
						(float)ay8912_calc(ay) * ay_volume / AY8912_MAX_OUTPUT;
					if (mixed >  1.0f) mixed =  1.0f;
					if (mixed < -1.0f) mixed = -1.0f;
					buf[i] = (int16_t)(mixed * 32767.0f);
				}
			} else {
				float mixed = bv + tv;
				if (mixed >  1.0f) mixed =  1.0f;
				if (mixed < -1.0f) mixed = -1.0f;
				int16_t val = (int16_t)(mixed * 32767.0f);
				for (int i = 0; i < n; ++i) buf[i] = val;
			}
			SDL_QueueAudio(audio_dev, buf, n * (int)sizeof(int16_t));
			nsamp -= n;
		}
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
	if (beeper_level != level_now)
	{
		uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
		beeper_advance_to(t_now);
		beeper_level = level_now ? 1 : 0;
	}
    
}

/* EAR=b4, MIC=b3 → modelado sencillo como OR */
static inline void beeper_set_from_ula(uint8_t v)
{
    //int level = (((v >> 4) & 1) | ((v >> 3) & 1) | ((v & 0x10) != 0)) != 0 ? 1 : 0;
	int level = (((v & 0x10) != 0)) != 0 ? 1 : 0; // ear
	//level = (level != 0) ? level : (((v & 0x08) != 0)) ? 1 : 0;
	//level |= (((v & 0x08) != 0)) ? 1 : 0; // mic
	
	//if (beeper_level != level)
	{
		beeper_set_level(level);
	}
    
}

/* ─────────────────────────────────────────────────────────────
 * Tape EAR-in audio callback.
 * Called by tape.c / tzx.c before each EAR level change so the
 * audio generator is flushed to the exact edge time first.
 * ───────────────────────────────────────────────────────────── */
#if 0
static void on_tape_ear_change(uint64_t t_abs, int new_level)
{
    beeper_advance_to(t_abs);   /* flush audio up to this edge */
    tape_ear_level = new_level;
}
#endif

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

/* ─────────────────────────────────────────────────────────────
 * Cycle-accurate (t-state) border rasterizer
 *
 * Timing model (PAL, from interrupt = t 0):
 *   48K  : 224 t-states/line, 312 lines/frame (69 888 t-states/frame)
 *   128K/+3: 228 t-states/line, 312 lines/frame (71 136 t-states/frame)
 *
 * Visible frame layout (line numbers relative to INT):
 *   lines   0 – 15  : top retrace   (invisible)
 *   lines  16 – 47  : top border    → texture rows 0 .. BORDER-1
 *   lines  48 – 63  : top overscan  (invisible)
 *   lines  64 – 255 : screen        → texture rows BORDER .. BORDER+191
 *                      (VRAM-rasterised at end of frame; only left/right
 *                       border columns are written here)
 *   lines 256 – 287 : bottom border → texture rows BORDER+192 .. HEIGHT-1
 *   lines 288 – 311 : bottom retrace (invisible)
 *
 * Horizontal timing within a line (48K, 224 t-states/line):
 *   t  0 – 23  : left border  (24 t-states → BORDER=32 pixels)
 *   t 24 – 151 : active video (128 t-states → 256 pixels; not drawn here)
 *   t 152 – 175: right border (24 t-states → BORDER=32 pixels)
 *   t 176 – 223: horizontal flyback (invisible)
 * For 128K/+3 (228 t-states/line) the left/right border spans are 26
 * t-states each; the flyback accounts for the extra 4 t-states.
 * ───────────────────────────────────────────────────────────── */

/* First visible line numbers within the 312-line frame (from INT) */
#define FIRST_TOP_LINE  16u   /* first top-border line  */
#define FIRST_SCR_LINE  64u   /* first screen line      */
#define FIRST_BOT_LINE  256u  /* first bottom-border line */

/* T-states per scanline for the active model */
static inline bool is_48k_model(void)
{
    return model == ZX_48K_2 || model == ZX_48K_3;
}

static inline unsigned tstates_per_line(void)
{
    return is_48k_model() ? 224u : 228u;
}

/* Horizontal border timing: left/right border t-state span per model.
 * Active video always spans 128 t-states (256 pixels at 2 t-states/pixel).
 * Returns 24 for 48K or 26 for 128K/+3 – never 0. */
static inline unsigned h_border_t(void)
{
    return is_48k_model() ? 24u : 26u;
}

/* Border rasterizer state */
static uint64_t brd_frame_org = 0; /* absolute t-state at frame start    */
static uint64_t brd_slice_org = 0; /* absolute t-state at slice start    */
static uint64_t brd_drawn_to  = 0; /* absolute t-state drawn up to       */

/* Advance border drawing from brd_drawn_to up to t_abs (absolute t-state).
 * Uses the current border_color for all newly drawn pixels.
 *
 * Invariant: brd_frame_org <= brd_drawn_to <= t_abs.
 * border_begin_frame() sets brd_frame_org = brd_drawn_to; after that
 * brd_drawn_to only grows, so the unsigned subtractions below never wrap. */
static void border_advance_to(uint64_t t_abs)
{
    if (t_abs <= brd_drawn_to)
        return;
    /* Safety: ensure we never underflow the frame-relative conversion */
    if (brd_drawn_to < brd_frame_org)
        brd_drawn_to = brd_frame_org;

    unsigned tpl  = tstates_per_line();
    unsigned hbt  = h_border_t();          /* left/right border span (t-states) */
    unsigned hle  = hbt;                   /* left border end within line        */
    unsigned hse  = hle + 128u;            /* screen end (= right border start)  */
    unsigned hbe  = hse + hbt;             /* right border end (flyback starts)  */

    uint32_t col  = palette[border_color & 0x0Fu];

    /* Work in frame-relative t-states */
    uint64_t ft     = brd_drawn_to - brd_frame_org;
    uint64_t ft_end = t_abs        - brd_frame_org;

    while (ft < ft_end) {
        unsigned line   = (unsigned)(ft / tpl);
        unsigned col_t  = (unsigned)(ft % tpl);

        /* End of current line in frame-relative t-states */
        uint64_t line_end_ft = (uint64_t)(line + 1u) * tpl;
        uint64_t seg_end_ft  = (ft_end < line_end_ft) ? ft_end : line_end_ft;
        unsigned col_t_end   = (unsigned)(seg_end_ft - (uint64_t)line * tpl);

        /* Map line to texture row and row type */
        int      tex_row  = -1;
        int      row_type = -1; /* 0 = full-width top/bottom, 1 = screen left+right */

        if (line >= FIRST_TOP_LINE && line < FIRST_TOP_LINE + BORDER) {
            tex_row  = (int)(line - FIRST_TOP_LINE);
            row_type = 0;
        } else if (line >= FIRST_SCR_LINE && line < FIRST_SCR_LINE + 192u) {
            tex_row  = (int)(BORDER + (line - FIRST_SCR_LINE));
            row_type = 1;
        } else if (line >= FIRST_BOT_LINE && line < FIRST_BOT_LINE + BORDER) {
            tex_row  = (int)(BORDER + 192u + (line - FIRST_BOT_LINE));
            row_type = 0;
        } else {
            /* Invisible line (retrace / overscan) – skip to next line */
            ft = line_end_ft;
            continue;
        }

        if (row_type == 0) {
            /* Full-width top/bottom border row.
             * Map t-states [0, hbe) linearly to pixels [0, WIDTH).
             * Anything in the flyback region [hbe, tpl) is not drawn. */
            if (col_t < hbe) {
                unsigned ce = (col_t_end < hbe) ? col_t_end : hbe;
                unsigned x0 = (unsigned)((uint64_t)col_t  * WIDTH / hbe);
                unsigned x1 = (unsigned)((uint64_t)ce     * WIDTH / hbe);
                if (x1 > (unsigned)WIDTH) x1 = (unsigned)WIDTH;
                uint32_t *p = texturebits + tex_row * WIDTH + (int)x0;
                for (unsigned x = x0; x < x1; x++)
                    *p++ = col;
            }
        } else {
            /* Screen row: only the left and right border columns are drawn.
             *   Left border : t [0, hle)   → pixel x [0, BORDER)
             *   Right border: t [hse, hbe) → pixel x [BORDER+256, WIDTH) */

            /* Left border */
            if (col_t < hle) {
                unsigned ce = (col_t_end < hle) ? col_t_end : hle;
                unsigned x0 = (unsigned)((uint64_t)col_t * BORDER / hle);
                unsigned x1 = (unsigned)((uint64_t)ce    * BORDER / hle);
                if (x1 > BORDER) x1 = BORDER;
                uint32_t *p = texturebits + tex_row * WIDTH + (int)x0;
                for (unsigned x = x0; x < x1; x++)
                    *p++ = col;
            }

            /* Right border */
            if (col_t_end > hse && col_t < hbe) {
                unsigned cs = (col_t  > hse) ? col_t     : hse;
                unsigned ce = (col_t_end < hbe) ? col_t_end : hbe;
                unsigned x0 = (unsigned)(BORDER + 256u +
                               (uint64_t)(cs - hse) * BORDER / hbt);
                unsigned x1 = (unsigned)(BORDER + 256u +
                               (uint64_t)(ce - hse) * BORDER / hbt);
                if (x1 > (unsigned)WIDTH) x1 = (unsigned)WIDTH;
                uint32_t *p = texturebits + tex_row * WIDTH + (int)x0;
                for (unsigned x = x0; x < x1; x++)
                    *p++ = col;
            }
        }

        ft = seg_end_ft;
    }

    brd_drawn_to = t_abs;
}

/* Called once at the start of each 312-line frame (after INT fires). */
static inline void border_begin_frame(void)
{
    brd_frame_org = brd_drawn_to;
}

/* Called before each Z80ExecuteTStates slice. */
static inline void border_begin_slice(void)
{
    brd_slice_org = brd_drawn_to;
}

/* Called after each Z80ExecuteTStates slice. */
static inline void border_end_slice(void)
{
    uint64_t t_now = brd_slice_org + (uint64_t)cpu_z80.tstates;
    border_advance_to(t_now);
}

static void fdc_log(int debuglevel, char *fmt, va_list ap)
{
    if ((trace & TRACE_FDC) || debuglevel == 0) {
        fprintf(stderr, "fdc: ");
        vfprintf(stderr, fmt, ap);
    }
}

/* ─────────────────────────────────────────────────────────────
 * TAP pulse player instance (types/functions in tape.c / tape.h)
 * ───────────────────────────────────────────────────────────── */
//static tape_player_t tape = {0};

/* ─────────────────────────────────────────────────────────────
 * TZX player (via tzx.c)
 * ───────────────────────────────────────────────────────────── */
//static tzx_player_t *tzx_player = NULL;
//static uint64_t      tzx_frame_origin = 0;

/* ─────────────────────────────────────────────────────────────
 * ULA I/O
 * ───────────────────────────────────────────────────────────── */

static void ula_write(uint8_t v)
{
    /* ear is bit 4 mic is bit 3, border low bits */
    ula = v;

    /* ▶️ Actualiza beeper (EAR|MIC) al instante actual */
    beeper_set_from_ula(v);

    /* Catch up border drawing to the current t-state, then apply new color */
    uint64_t t_now = brd_slice_org + (uint64_t)cpu_z80.tstates;
    border_advance_to(t_now);
    border_color = v & 7;
}

static uint8_t ula_read(uint16_t addr)
{
    uint8_t r = 0xA0;  /* Fixed bits */

    /* bit6 = EAR (entrada):
       - Si la cinta por pulsos está activa, domina la línea
       - Si no, conserva tu comportamiento previo para Issue 3/48K_2 */
    uint8_t ear_b6 = 0x00;
	if (tape.fmt != TAPE_FMT_NONE && tape.playing) {
		if (get_current_ear_level_from_tape())
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
    if ((addr & 0x01) == 0) { /* ULA */
		//uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
        //beeper_advance_to(t_now);
        return ula_read(addr);
	}
    /* AY-3-8912: IN 0xFFFD reads the currently selected register (128K/+3 only).
     * Advance audio to current t-state first to preserve event ordering. */
    if ((model == ZX_128K || model == ZX_PLUS3) && (addr & 0xC002) == 0xC000) {
        if (ay) {
            uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
            beeper_advance_to(t_now);
            return ay8912_read_data(ay);
        }
        return 0xFF;
    }
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
    /* AY-3-8912 ports (128K/+3 only).
     * Advance audio to the current t-state before changing AY state so
     * that the register write lands at the correct position in the stream. */
    if ((model == ZX_128K || model == ZX_PLUS3) && ay) {
        if ((addr & 0xC002) == 0xC000) {
            /* OUT 0xFFFD: select AY register */
            uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
            beeper_advance_to(t_now);
            ay8912_select_reg(ay, val);
        } else if ((addr & 0xC002) == 0x8000) {
            /* OUT 0xBFFD: write data to selected AY register */
            uint64_t t_now = beeper_slice_origin + (uint64_t)cpu_z80.tstates;
            beeper_advance_to(t_now);
            ay8912_write_data(ay, val);
        }
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

static void run_scanlines(unsigned lines, unsigned blank) {
    unsigned i;
    unsigned tpl = tstates_per_line();
    unsigned n = tpl;

    blanked = blank;
    if (!blanked) drawline = 0;

    for (i = 0; i < lines; i++) {
        beeper_begin_slice();
        border_begin_slice();

        // AVANZA EMULACIÓN Y CINTA
        n = tpl + tpl - Z80ExecuteTStates(&cpu_z80, n);

        // Avance global del ciclo y cassette
        global_cycles += n; // OJO: ¡Pon esto!
        tape_ear_active = tape.playing && (tape.fmt != TAPE_FMT_NONE);
        tape_ear_level = get_current_ear_level_from_tape();

        border_end_slice();
        beeper_end_slice();
        if (!blanked)
            drawline++;
    }
    if (ui_event())
        emulator_done = 1;
#if 0
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
#endif
    
}

/* ─────────────────────────────────────────────────────────────
 * Hotkeys (SDL): F6 = Reload TAP & Auto-Start; F7 = List TAP
 *                F8 = Play/Pause tape pulses; F9 = Rewind tape
 * ───────────────────────────────────────────────────────────── */
static void handle_hotkeys() {
    SDL_PumpEvents();
    const Uint8* ks = SDL_GetKeyboardState(NULL);
    static int prev_f6 = 0, prev_f7 = 0, prev_f8 = 0, prev_f9 = 0, prev_f12 = 0;
    int f6 = ks[SDL_SCANCODE_F6] ? 1 : 0;
    int f7 = ks[SDL_SCANCODE_F7] ? 1 : 0;
    int f8 = ks[SDL_SCANCODE_F8] ? 1 : 0;
    int f9 = ks[SDL_SCANCODE_F9] ? 1 : 0;
    int f12 = ks[SDL_SCANCODE_F12] ? 1 : 0;

    if (f6 && !prev_f6) {
        if (tape.fmt == TAPE_FMT_TAP && tape_filename)
            load_tap(tape_filename);
        else if (tape.fmt == TAPE_FMT_TZX && tape_filename)
            load_tzx(tape_filename);
    }
    if (f7 && !prev_f7) {
        if (tape.fmt == TAPE_FMT_TAP && tape_filename)
            list_tap_blocks(tape_filename);
        else if (tape.fmt == TAPE_FMT_TZX && tape_filename)
            list_tzx_blocks(tape_filename);
    }
    if (f8 && !prev_f8) {
        tape.playing = !tape.playing;
        fprintf(stdout, "[F8] Tape %s\n", tape.playing ? "PLAY" : "PAUSE");
    }
    if (f9 && !prev_f9) {
        if (tape.f) {
            fseek(tape.f, 0, SEEK_SET); tape.file_pos = 0;
            if (tape.fmt == TAPE_FMT_TAP) tap_read_next_block();
            if (tape.fmt == TAPE_FMT_TZX) tzx_read_and_prepare_next_block(global_cycles);
            tape.playing = true;
            printf("[F9] Tape REWIND\n");
        }
    }
    if (f12 && !prev_f12) {
        fast = !fast;
        fprintf(stdout, "[F12] %s!\n", fast ? "SPEED" : "SLOW");
    }
    prev_f6 = f6; prev_f7 = f7; prev_f8 = f8; prev_f9 = f9; prev_f12 = f12;
}

static void usage(void)
{
    fprintf(stderr, "spectrum: [-f] [-r path] [-d debug] [-A disk] [-B disk]\n"
            "          [-i idedisk] [-I dividerom] [-t tap] [-s sna] [-T tap_pulses]\n"
            "          [-z tzxfile]\n");
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
    //char *tap_pulses_path = NULL;
    char *tzx_path = NULL;

    /* Añadimos 't:' (tap fast), 'T:' (tap pulses) y 'z:' (TZX) */
    while ((opt = getopt(argc, argv, "d:f:r:m:i:I:A:B:s:t:T:z:")) != -1) {
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
            //tap_pulses_path = optarg;
            break;
        case 'z':
            tzx_path = optarg;
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
                  WIDTH * SCALE,
                  HEIGHT * SCALE, SDL_WINDOW_RESIZABLE);
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
        /* AY-3-8912: present on 128K/+3 only, clocked at CPU_CLK/2. */
        if (model == ZX_128K || model == ZX_PLUS3) {
            ay = ay8912_create((uint32_t)(TSTATES_CPU / 2), (uint32_t)audio_rate);
            if (!ay)
                fprintf(stderr, "Aviso: no se pudo inicializar el AY-3-8912.\n");
        }
    }

    if (tapepath) {
		if (!load_tap(tapepath)) {
			fprintf(stderr, "Fallo al cargar TAP: %s\n", tapepath);
		} else {
			printf("TAP cargado correctamente.\n");
		}
		tape_filename = tapepath;
	}
	if (tzx_path) {
		if (!load_tzx(tzx_path)) {
			fprintf(stderr, "Fallo al cargar TZX: %s\n", tzx_path);
		} else {
			printf("TZX cargado correctamente.\n");
		}
		tape_filename = tzx_path;
	}

    if (snapath)
    {
        sna_context_t sna_ctx = {
            .cpu         = &cpu_z80,
            .ram         = ram,
            .border_color = &border_color,
            .mlatch      = &mlatch,
            .mem_write   = mem_write,
            .mem_read    = mem_read,
            .recalc_mmu  = recalc_mmu,
        };
        load_sna(snapath, &sna_ctx);
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
        handle_hotkeys();

        /*
         * Run one full PAL frame (312 lines) with model-correct t-states/line.
         * Frame layout from INT (t = 0):
         *   lines   0 –  63 : top area  (retrace + BORDER=32 visible rows)
         *   lines  64 – 255 : screen    (192 lines; VRAM rasterised at end)
         *   lines 256 – 311 : bottom area (BORDER=32 visible rows + retrace)
         */
        border_begin_frame();
        run_scanlines(64, 0);
        run_scanlines(192, 1);
        run_scanlines(56, 0);
        spectrum_rasterize();
        spectrum_render();
        Z80INT(&cpu_z80, 0xFF);
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
    ay8912_destroy(ay);
    ay = NULL;
    //tzx_destroy(tzx_player);
    exit(0);
}