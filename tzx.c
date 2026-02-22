/* tzx.c - TZX player (complete file) */
#define _CRT_SECURE_NO_WARNINGS
#include "tzx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* Frecuencia de la CPU (t-states/seg). En 128K ~3.5469MHz; 48K ~3.5MHz.
   Las rutinas de carga de cinta son por flancos, la pequeña diferencia es tolerada. */
#ifndef TZX_CPU_TSTATES
#define TZX_CPU_TSTATES 3546900.0
#endif

/* ================= Debug tracing ================= */
#ifndef TZX_TRACE
#define TZX_TRACE 1
#endif

#if TZX_TRACE
#define TZX_TRACEF(...) printf(__VA_ARGS__)
#else
#define TZX_TRACEF(...) ((void)0)
#endif

/* ================= Utilidades LE ================= */
static inline uint16_t rd_le16(const uint8_t* p){ return (uint16_t)(p[0] | (p[1]<<8)); }
static inline uint32_t rd_le24(const uint8_t* p){ return (uint32_t)(p[0] | (p[1]<<8) | (p[2]<<16)); }
static inline uint32_t rd_le32(const uint8_t* p){ return (uint32_t)(p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24)); }

/* ================= Estructuras internas ================ */
typedef struct {
    uint8_t id;
    uint32_t ofs;  /* offset del bloque (ID incluido) dentro de buf */
    uint32_t len;  /* longitud total del bloque (ID + payload) */
} tzx_block_ref_t;

typedef enum {
    TZX_OK=0, TZX_ERR_IO, TZX_ERR_FMT, TZX_ERR_VER, TZX_ERR_MEM, TZX_ERR_UNSUP
} tzx_err_t;

#define TZX_MAX_PULSES 16
#define TZX_MAX_ALPHA  256

typedef struct {
    uint8_t flags;                /* 0: invert; 1: same; 2: force low; 3: force high (bits 0..1) */
    uint8_t npulses;              /* N máximo por símbolo */
    uint16_t pulses[TZX_MAX_PULSES];
} tzx_symbol_t;

struct tzx_player {
    /* buffer TZX en memoria */
    uint8_t* buf;
    size_t   len;
    uint8_t  ver_major, ver_minor;

    /* índice de bloques */
    tzx_block_ref_t* blk;
    int nblk;

    /* reproducción */
    int i_blk;
    int playing;
    int done;
    uint8_t  ear_level;        /* 0/1 */
    uint64_t frame_origin;
    uint64_t slice_origin;
    uint64_t next_edge_at;     /* siguiente flanco programado */
    uint64_t pause_end_at;     /* fin de pausa activa */

    /* callback de audio: llamado antes de cada cambio de nivel EAR */
    tzx_ear_notify_fn notify_fn;

    /* sub-estado común a varios bloques */
    uint32_t sub_ofs;
    uint32_t sub_len;
    uint16_t p_pilot_len, p_sync1, p_sync2, p_0, p_1;
    uint32_t p_pilot_count;
    uint8_t  p_used_bits;
    uint32_t p_pause_ms;
    uint32_t data_len;

    /* iteración de bits */
    uint32_t i_byte;
    uint8_t  bit_mask;
    uint8_t  subpulse;
    int      pilot_left;       /* >=1 pilot; -1 sync1; -2 sync2; -3 bits */

    /* 0x15 Direct recording */
    uint32_t dr_total_bits;
    uint32_t dr_abs_bit;
    uint16_t dr_ts_per_sample;

    /* 0x18 CSW */
    uint8_t  csw_ctype; /* 1 RLE; 2 Zero-RLE */

    /* 0x19 Generalized data */
    /* parámetros */
    uint32_t gen_blen;
    uint16_t gen_pause;
    uint32_t gen_totp, gen_totd;
    uint8_t  gen_npp, gen_asp, gen_npd, gen_asd;
    uint32_t gen_ofs_alphaP, gen_ofs_pilotStream;
    uint32_t gen_ofs_alphaD, gen_ofs_dataStream;
    int      gen_phase; /* 0 pilot, 1 data, 2 pause/done */
    /* alfabetos */
    tzx_symbol_t gen_symP[TZX_MAX_ALPHA], gen_symD[TZX_MAX_ALPHA];
    int      gen_loaded;
    /* estado símbolo en curso */
    const tzx_symbol_t* gen_cur_sym;
    int      gen_sym_ip;      /* índice de pulso dentro del símbolo */
    int      gen_sym_rem;     /* pulsos restantes */
    /* pilot stream */
    uint32_t gen_pilot_pos;   /* símbolo # actual dentro de totp */
    uint8_t  gen_pilot_sym_idx;
    uint16_t gen_pilot_rep_left;
    /* data stream */
    int      gen_bits;        /* bits por símbolo (ceil log2(asd)) */
    uint32_t gen_data_pos;    /* símbolo # consumidos */
    uint32_t gen_data_dsize;  /* bytes del stream de datos */
    uint32_t gen_data_ofs;    /* offset de lectura en dataStream */
    uint8_t  gen_data_byte;
    int      gen_data_bits_left;

    /* error */
    char     last_error[128];
};

/* ================ helpers tiempo y errores ================== */
static inline uint64_t ms_to_tstates(uint32_t ms){
    return (uint64_t)((double)ms * (TZX_CPU_TSTATES/1000.0));
}
static inline void tzx_set_error(tzx_player_t* tp, const char* msg){
    snprintf(tp->last_error, sizeof(tp->last_error), "%s", msg);
}

/* Helper: notify host then flip EAR level.
* t_edge is the exact t-state at which the transition occurs. */
#define TZX_EAR_TOGGLE(tp, t_edge) do { \
    int _nl = (tp)->ear_level ^ 1; \
    if ((tp)->notify_fn) (tp)->notify_fn((t_edge), _nl); \
    (tp)->ear_level = _nl; \
} while (0)

/* Helper: notify host then assign EAR level (only if it changes). */
#define TZX_EAR_SET(tp, t_edge, new_lv) do { \
    int _nl = (int)(new_lv); \
    if ((tp)->notify_fn && _nl != (tp)->ear_level) \
        (tp)->notify_fn((t_edge), _nl); \
    (tp)->ear_level = _nl; \
} while (0)

/* Maximum edges processed per tzx_advance_to() call.
* A full PAL frame at 50 Hz contains ~69 888 t-states; the shortest
* standard pulse is sync1 = 667 t-states, giving at most ~105 edges/frame.
* CSW / direct-recording blocks can have much shorter pulses – 200 000
* provides a safe headroom while bounding worst-case runtime. */
#define TZX_MAX_EDGES_PER_SLICE 200000

/* ================ API pública =============================== */
tzx_player_t* tzx_create(void){
    tzx_player_t* tp = (tzx_player_t*)calloc(1, sizeof(*tp));
    tp->ear_level = 1;
    return tp;
}
void tzx_destroy(tzx_player_t* tp){
    if (!tp) return;
    free(tp->blk);
    free(tp->buf);
    free(tp);
}
const char* tzx_last_error(const tzx_player_t* tp){ return tp?tp->last_error:""; }

void tzx_set_ear_notify(tzx_player_t* tp, tzx_ear_notify_fn fn){
    if (tp) tp->notify_fn = fn;
}

void tzx_play(tzx_player_t* tp){ if (tp){ tp->playing=1; } }
void tzx_pause(tzx_player_t* tp, int pause_on){ if(tp){ tp->playing = pause_on?0:1; } }
void tzx_rewind(tzx_player_t* tp){
    if (!tp) return;
    tp->i_blk=0; tp->done=0;
    tp->sub_ofs=tp->sub_len=0;
    tp->frame_origin=tp->slice_origin=0;
    tp->next_edge_at=0; tp->pause_end_at=0;
    tp->ear_level=1; tp->pilot_left=0;
    tp->i_byte=0; tp->bit_mask=0x80; tp->subpulse=0;
    tp->dr_total_bits=0; tp->dr_abs_bit=0; tp->dr_ts_per_sample=0;
    /* 0x19 */
    tp->gen_loaded=0; tp->gen_phase=0; tp->gen_cur_sym=NULL;
    tp->gen_sym_ip=0; tp->gen_sym_rem=0;
    tp->gen_pilot_pos=0; tp->gen_pilot_rep_left=0;
    tp->gen_data_pos=0; tp->gen_data_dsize=0; tp->gen_data_ofs=0;
    tp->gen_data_bits_left=0; tp->gen_bits=0;
}

/* ================ indexado de bloques ======================= */

static int tzx_build_block_index(tzx_player_t* tp)
{
    if (tp->len < 10){ tzx_set_error(tp,"TZX corto"); return -1; }
    if (memcmp(tp->buf, "ZXTape!", 7)!=0 || tp->buf[7]!=0x1A){
        tzx_set_error(tp,"Cabecera TZX invalida");
        return -1;
    }
    tp->ver_major = tp->buf[8];
    tp->ver_minor = tp->buf[9];

    size_t p=10;
    int cap=64, n=0;
    tzx_block_ref_t* v = (tzx_block_ref_t*)malloc(cap*sizeof(*v));
    if(!v){ tzx_set_error(tp,"mem"); return -1; }

    while (p < tp->len){
        uint8_t id = tp->buf[p];
        size_t start = p; p+=1;
        size_t advance=0;

        switch(id){
        case 0x10:
            if (p+4 > tp->len) goto trunc;
            advance = 2 + 2 + rd_le16(&tp->buf[p+2]);
            break;
        case 0x11:
            if (p+2*6+1+2+3 > tp->len) goto trunc;
            advance = 2*6+1+2+3 + rd_le24(&tp->buf[p+2*6+1+2]);
            break;
        case 0x12: advance=4; break;
        case 0x13:
            if (p+1 > tp->len) goto trunc;
            advance = 1 + 2*tp->buf[p];
            break;
        case 0x14:
            if (p+2+2+1+2+3 > tp->len) goto trunc;
            advance = 2+2+1+2+3 + rd_le24(&tp->buf[p+2+2+1+2]);
            break;
        case 0x15:
            if (p+2+2+1+3 > tp->len) goto trunc;
            advance = 2+2+1+3 + rd_le24(&tp->buf[p+2+2+1]);
            break;
        case 0x18:
            if (p+4+2+3+1+4 > tp->len) goto trunc;
            advance = 4+2+3+1+4 + rd_le32(&tp->buf[p]);
            break;
        case 0x19:
            if (p+4+2 > tp->len) goto trunc;
            advance = 4 + rd_le32(&tp->buf[p]);
            break;
        case 0x20: advance = 2; break;
        case 0x21: if (p+1 > tp->len) goto trunc; advance=1+tp->buf[p]; break;
        case 0x22: advance = 0; break;
        case 0x23: advance = 2; break;
        case 0x24: advance = 2; break;
        case 0x25: advance = 0; break;
        case 0x26:
            if (p+2 > tp->len) goto trunc;
            advance = 2 + 2*rd_le16(&tp->buf[p]);
            break;
        case 0x27: advance=0; break;
        case 0x28:
            if (p+2 > tp->len) goto trunc;
            advance = 2 + rd_le16(&tp->buf[p]);
            break;
        case 0x2A: advance = 4; break;
        case 0x2B: advance = 5; break;
        case 0x30: if (p+1 > tp->len) goto trunc; advance=1+tp->buf[p]; break;
        case 0x31: if (p+2 > tp->len) goto trunc; advance=2+tp->buf[p+1]; break;
        case 0x32: if (p+2 > tp->len) goto trunc; advance=2+rd_le16(&tp->buf[p]); break;
        case 0x33: if (p+1 > tp->len) goto trunc; advance=1 + 3*tp->buf[p]; break;
        case 0x35: if (p+16+4 > tp->len) goto trunc; advance=16+4+rd_le32(&tp->buf[p+16]); break;
        case 0x5A: advance=9; break;
        default:
            free(v); tzx_set_error(tp,"Bloque TZX no soportado"); return -1;
        }
        if (p+advance > tp->len) goto trunc;
        if (n==cap){ cap*=2; tzx_block_ref_t* nv=(tzx_block_ref_t*)realloc(v,cap*sizeof(*nv)); if(!nv){free(v);tzx_set_error(tp,"mem");return -1;} v=nv; }
        v[n].id = id; v[n].ofs = (uint32_t)start; v[n].len = (uint32_t)(1 + advance); n++;
        p += advance;
    }
    tp->blk = v; tp->nblk = n;
    return 0;
trunc:
    free(v); tzx_set_error(tp,"TZX truncado"); return -1;
}

/* ================ TZX trace helpers (compiled only when TZX_TRACE) ======== */
#if TZX_TRACE

static const char* tzx_block_id_name(uint8_t id)
{
    switch(id){
    case 0x10: return "Standard speed data";
    case 0x11: return "Turbo speed data";
    case 0x12: return "Pure tone";
    case 0x13: return "Pulse sequence";
    case 0x14: return "Pure data";
    case 0x15: return "Direct recording";
    case 0x18: return "CSW recording";
    case 0x19: return "Generalized data";
    case 0x20: return "Pause/Stop";
    case 0x21: return "Group start";
    case 0x22: return "Group end";
    case 0x23: return "Jump to block";
    case 0x24: return "Loop start";
    case 0x25: return "Loop end";
    case 0x26: return "Call sequence";
    case 0x27: return "Return from sequence";
    case 0x28: return "Select block";
    case 0x2A: return "Stop tape if in 48K mode";
    case 0x2B: return "Set signal level";
    case 0x30: return "Text description";
    case 0x31: return "Message block";
    case 0x32: return "Archive info";
    case 0x33: return "Hardware type";
    case 0x35: return "Custom info";
    case 0x5A: return "Glue block";
    default:   return "Unknown";
    }
}

static void tzx_trace_block_details(const tzx_player_t* tp, int idx)
{
    const tzx_block_ref_t* ref = &tp->blk[idx];
    const uint8_t* p = &tp->buf[ref->ofs + 1]; /* payload */
    uint32_t plen = (ref->len > 1) ? (ref->len - 1) : 0;

    switch(ref->id){
    case 0x10:
        if (plen >= 4)
            TZX_TRACEF("    pause=%ums datalen=%u\n", rd_le16(p), rd_le16(p+2));
        break;
    case 0x11:
        if (plen >= 18)
            TZX_TRACEF("    pilot=%u sync1=%u sync2=%u p0=%u p1=%u pilotcnt=%u usedbits=%u pause=%ums datalen=%u\n",
                rd_le16(p), rd_le16(p+2), rd_le16(p+4), rd_le16(p+6), rd_le16(p+8),
                rd_le16(p+10), p[12], rd_le16(p+13), rd_le24(p+15));
        break;
    case 0x12:
        if (plen >= 4)
            TZX_TRACEF("    pulselen=%u pulsecount=%u\n", rd_le16(p), rd_le16(p+2));
        break;
    case 0x13:
        if (plen >= 1)
            TZX_TRACEF("    npulses=%u\n", p[0]);
        break;
    case 0x14:
        if (plen >= 10)
            TZX_TRACEF("    p0=%u p1=%u usedbits=%u pause=%ums datalen=%u\n",
                rd_le16(p), rd_le16(p+2), p[4], rd_le16(p+5), rd_le24(p+7));
        break;
    case 0x15:
        if (plen >= 8)
            TZX_TRACEF("    tsps=%u pause=%ums usedbits=%u datalen=%u\n",
                rd_le16(p), rd_le16(p+2), p[4], rd_le24(p+5));
        break;
    case 0x18:
        if (plen >= 14)
            TZX_TRACEF("    blen=%u pause=%ums rate=%u ctype=%u npulses=%u\n",
                rd_le32(p), rd_le16(p+4), rd_le24(p+6), p[9], rd_le32(p+10));
        break;
    case 0x19:
        if (plen >= 18)
            TZX_TRACEF("    blen=%u pause=%ums totp=%u npp=%u asp=%u totd=%u npd=%u asd=%u\n",
                rd_le32(p), rd_le16(p+4), rd_le32(p+6), p[10], p[11],
                rd_le32(p+12), p[16], p[17]);
        break;
    case 0x20:
        if (plen >= 2)
            TZX_TRACEF("    pause_ms=%u%s\n", rd_le16(p), rd_le16(p)==0?" (STOP)":"");
        break;
    case 0x21:
        if (plen >= 1){
            uint8_t slen = p[0];
            if (plen >= (uint32_t)(1 + slen))
                TZX_TRACEF("    name=\"%.*s\"\n", slen, p+1);
        }
        break;
    case 0x23:
        if (plen >= 2)
            TZX_TRACEF("    rel_jump=%d\n", (int)(int16_t)rd_le16(p));
        break;
    case 0x24:
        if (plen >= 2)
            TZX_TRACEF("    count=%u\n", rd_le16(p));
        break;
    case 0x26:
        if (plen >= 2)
            TZX_TRACEF("    ncalls=%u\n", rd_le16(p));
        break;
    case 0x28:
        if (plen >= 2)
            TZX_TRACEF("    nselections=%u\n", rd_le16(p));
        break;
    case 0x2B:
        if (plen >= 5)
            TZX_TRACEF("    level=%u\n", p[4]);
        break;
    case 0x30:
        if (plen >= 1){
            uint8_t slen = p[0];
            if (plen >= (uint32_t)(1 + slen))
                TZX_TRACEF("    text=\"%.*s\"\n", slen, p+1);
        }
        break;
    case 0x31:
        if (plen >= 2){
            uint8_t slen = p[1];
            if (plen >= (uint32_t)(2 + slen))
                TZX_TRACEF("    display_time=%us text=\"%.*s\"\n", p[0], slen, p+2);
        }
        break;
    default:
        break;
    }
}

static void tzx_trace_block_list(const tzx_player_t* tp, const char* path)
{
    TZX_TRACEF("[TZX] Loaded \"%s\" ver %u.%u  blocks=%d\n",
        path, tp->ver_major, tp->ver_minor, tp->nblk);
    TZX_TRACEF("[TZX] %-4s  %-6s  %-5s  %s  %s\n",
        "IDX", "OFFSET", "LEN", "ID  ", "Description");
    for (int i = 0; i < tp->nblk; i++){
        const tzx_block_ref_t* b = &tp->blk[i];
        TZX_TRACEF("[TZX] #%-3d  0x%04X  %-5u  0x%02X  %s\n",
            i, b->ofs, b->len, b->id, tzx_block_id_name(b->id));
        tzx_trace_block_details(tp, i);
    }
}

#endif /* TZX_TRACE */

int tzx_load_file(tzx_player_t* tp, const char* path)
{
    if (!tp) return -1;
    FILE* f = fopen(path, "rb");
    if (!f){ tzx_set_error(tp,"IO"); return -1; }
    fseek(f,0,SEEK_END); long sz=ftell(f); fseek(f,0,SEEK_SET);
    if (sz<=0){ fclose(f); tzx_set_error(tp,"IO"); return -1; }
    uint8_t* buf=(uint8_t*)malloc((size_t)sz);
    if(!buf){ fclose(f); tzx_set_error(tp,"mem"); return -1; }
    if (fread(buf,1,(size_t)sz,f)!=(size_t)sz){ free(buf); fclose(f); tzx_set_error(tp,"IO"); return -1; }
    fclose(f);

    free(tp->buf); tp->buf=buf; tp->len=(size_t)sz;
    free(tp->blk); tp->blk=NULL; tp->nblk=0;

    if (tzx_build_block_index(tp)!=0) return -1;

#if TZX_TRACE
    tzx_trace_block_list(tp, path);
#endif

    tzx_rewind(tp);
    tzx_play(tp);
    return 0;
}

/* ================ helpers de reproducción ===================== */

static inline const uint8_t* cur_payload(const tzx_player_t* tp, uint32_t* out_plen){
    uint32_t ofs = tp->blk[tp->i_blk].ofs + 1;
    *out_plen = (uint32_t)(tp->len - ofs);
    return &tp->buf[ofs];
}

/* Forward declaration needed by proc functions defined before tzx_next_block */
static void tzx_next_block(tzx_player_t* tp);

/* -------- 0x10 / 0x11 Standard / Turbo -------- */
static void tzx_init_std_or_turbo(tzx_player_t* tp, int turbo)
{
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    uint32_t q=0;
    if (!turbo){
        if (plen<4){ tp->done=1; return; }
        tp->p_pause_ms = rd_le16(p+0);
        tp->data_len   = rd_le16(p+2);
        tp->p_pilot_len=2168; tp->p_sync1=667; tp->p_sync2=735; tp->p_0=855; tp->p_1=1710;
        q=4;
        if (tp->data_len==0 || q+tp->data_len>plen){ tp->done=1; return; }
        uint8_t flag = p[q];
        tp->p_pilot_count = (flag<128)? 8063:3223;
    } else {
        if (plen<2*6+1+2+3){ tp->done=1; return; }
        tp->p_pilot_len  = rd_le16(p+0);
        tp->p_sync1      = rd_le16(p+2);
        tp->p_sync2      = rd_le16(p+4);
        tp->p_0          = rd_le16(p+6);
        tp->p_1          = rd_le16(p+8);
        tp->p_pilot_count= rd_le16(p+10);
        tp->p_used_bits  = p[12];
        tp->p_pause_ms   = rd_le16(p+13);
        tp->data_len     = rd_le24(p+15);
        q=18;
        if (q+tp->data_len>plen){ tp->done=1; return; }
    }
    tp->sub_ofs = q; tp->sub_len = tp->data_len;
    tp->i_byte=0; tp->bit_mask=0x80; tp->subpulse=0;

    tp->pilot_left = tp->p_pilot_count;  /* >0 => pilot */
    TZX_EAR_TOGGLE(tp, tp->slice_origin);  /* primer flanco */
    tp->next_edge_at = tp->slice_origin + tp->p_pilot_len;
}
static void tzx_proc_std_or_turbo(tzx_player_t* tp, uint64_t t_now)
{
    if (tp->pilot_left > 0){
        if (t_now < tp->next_edge_at) return;
        TZX_EAR_TOGGLE(tp, tp->next_edge_at);
        if (--tp->pilot_left > 0){
            tp->next_edge_at += tp->p_pilot_len;
        } else {
            /* sync1 */
            tp->next_edge_at += tp->p_sync1;
            tp->pilot_left = -1;
        }
        return;
    }
    if (tp->pilot_left == -1){
        if (t_now < tp->next_edge_at) return;
        TZX_EAR_TOGGLE(tp, tp->next_edge_at);
        tp->next_edge_at += tp->p_sync2;
        tp->pilot_left = -2;
        return;
    }
    if (tp->pilot_left == -2){
        if (t_now < tp->next_edge_at) return;
        TZX_EAR_TOGGLE(tp, tp->next_edge_at);
        /* entra a bits */
        tp->pilot_left = -3;
        uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
        uint8_t b = p[tp->sub_ofs + tp->i_byte];
        int bit = (b & tp->bit_mask)?1:0;
        tp->next_edge_at += (bit?tp->p_1:tp->p_0);
        return;
    }
    /* bits */
    if (t_now < tp->next_edge_at) return;
    TZX_EAR_TOGGLE(tp, tp->next_edge_at);
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    uint8_t b = p[tp->sub_ofs + tp->i_byte];
    int bit = (b & tp->bit_mask)?1:0;
    int tlen = bit?tp->p_1:tp->p_0;

    if (tp->subpulse==0){
        tp->subpulse=1;
        tp->next_edge_at += tlen;
    } else {
        tp->subpulse=0;
        if (tp->bit_mask==0x01){ tp->bit_mask=0x80; tp->i_byte++; }
        else tp->bit_mask >>= 1;

        /* última byte con used_bits (solo turbo/pure) */
        if ((tp->blk[tp->i_blk].id==0x11 || tp->blk[tp->i_blk].id==0x14) &&
            tp->i_byte == tp->sub_len-1 && tp->p_used_bits>=1 && tp->p_used_bits<=8){
            /* si máscara cae fuera de bits usados, terminamos */
            if (tp->bit_mask < (1u<<(8 - tp->p_used_bits))){
                if (tp->p_pause_ms){
                    tp->pause_end_at = tp->next_edge_at + ms_to_tstates(tp->p_pause_ms);
                    tp->next_edge_at = 0;
                    tzx_next_block(tp);  /* avanzar ya al siguiente bloque */
                } else {
                    tzx_next_block(tp);
                }
                return;
            }
        }

        if (tp->i_byte >= tp->sub_len){
            if (tp->p_pause_ms){
                tp->pause_end_at = tp->next_edge_at + ms_to_tstates(tp->p_pause_ms);
                tp->next_edge_at = 0;
                tzx_next_block(tp);  /* avanzar ya al siguiente bloque */
            } else {
                tzx_next_block(tp);
            }
            return;
        }
        b = p[tp->sub_ofs + tp->i_byte];
        bit = (b & tp->bit_mask)?1:0;
        tlen = bit?tp->p_1:tp->p_0;
        tp->next_edge_at += tlen;
    }
}

/* -------- 0x12 Pure tone -------- */
static void tzx_init_pure_tone(tzx_player_t* tp)
{
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    if (plen<4){ tp->done=1; return; }
    tp->p_pilot_len = rd_le16(p+0);
    tp->p_pilot_count = rd_le16(p+2);
    tp->pilot_left = (int)tp->p_pilot_count;
    TZX_EAR_TOGGLE(tp, tp->slice_origin);
    tp->next_edge_at = tp->slice_origin + tp->p_pilot_len;
}
static void tzx_proc_pure_tone(tzx_player_t* tp, uint64_t t_now)
{
    if (tp->pilot_left<=0){ tzx_next_block(tp); return; }
    if (t_now < tp->next_edge_at) return;
    TZX_EAR_TOGGLE(tp, tp->next_edge_at);
    if (--tp->pilot_left>0) tp->next_edge_at += tp->p_pilot_len;
    else tzx_next_block(tp);
}

/* -------- 0x13 Pulse sequence -------- */
static void tzx_init_pulse_seq(tzx_player_t* tp)
{
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    if (plen<1){ tp->done=1; return; }
    uint8_t np = p[0];
    if (1 + 2*np > plen){ tp->done=1; return; }
    tp->sub_ofs = 1; tp->sub_len = 1 + 2*np;
    tp->i_byte = 0;
    TZX_EAR_TOGGLE(tp, tp->slice_origin);
    uint16_t t = rd_le16(p+tp->sub_ofs);
    tp->next_edge_at = tp->slice_origin + t;
}
static void tzx_proc_pulse_seq(tzx_player_t* tp, uint64_t t_now)
{
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    uint8_t np = p[0];
    if (t_now < tp->next_edge_at) return;
    TZX_EAR_TOGGLE(tp, tp->next_edge_at);
    tp->i_byte++;
    if (tp->i_byte >= np){ tzx_next_block(tp); return; }
    uint16_t t = rd_le16(p+tp->sub_ofs + 2*tp->i_byte);
    tp->next_edge_at += t;
}

/* -------- 0x14 Pure data -------- */
static void tzx_init_pure_data(tzx_player_t* tp)
{
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    if (plen<2+2+1+2+3){ tp->done=1; return; }
    tp->p_0 = rd_le16(p+0);
    tp->p_1 = rd_le16(p+2);
    tp->p_used_bits = p[4];
    tp->p_pause_ms  = rd_le16(p+5);
    tp->data_len    = rd_le24(p+7);
    if (10 + tp->data_len > plen){ tp->done=1; return; }
    tp->sub_ofs = 10; tp->sub_len = tp->data_len;
    tp->i_byte=0; tp->bit_mask=0x80; tp->subpulse=0;

    TZX_EAR_TOGGLE(tp, tp->slice_origin);
    uint8_t b = p[tp->sub_ofs];
    int bit = (b & tp->bit_mask)?1:0;
    tp->next_edge_at = tp->slice_origin + (bit?tp->p_1:tp->p_0);
}
static void tzx_proc_pure_data(tzx_player_t* tp, uint64_t t_now)
{
    /* reutiliza lógica de turbo */
    tzx_proc_std_or_turbo(tp, t_now);
}

/* -------- 0x15 Direct recording -------- */

/* Lee el bit i-ésimo (MSB-first dentro de cada byte) desde Direct Recording */
static inline int dr_get_bit(const uint8_t* base, uint32_t sub_ofs, uint32_t i)
{
    uint32_t byte_i = i >> 3;          /* i / 8 */
    uint8_t  bit_i  = 7 - (i & 7);     /* MSB-first en el byte */
    uint8_t  b      = base[sub_ofs + byte_i];
    return (b >> bit_i) & 1;
}

/* Devuelve la primera posición >= start_bit donde el bit cambia, o total_bits si no cambia */
static inline uint32_t dr_scan_run(const uint8_t* base, uint32_t sub_ofs,
                                   uint32_t start_bit, uint32_t total_bits)
{
    int cur = dr_get_bit(base, sub_ofs, start_bit);
    uint32_t i = start_bit + 1;
    while (i < total_bits) {
        if (dr_get_bit(base, sub_ofs, i) != cur) break;
        ++i;
    }
    return i;
}

static void tzx_init_direct(tzx_player_t* tp)
{
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    if (plen<2+2+1+3){ tp->done=1; return; }
    uint16_t tsps = rd_le16(p+0);
    tp->p_pause_ms = rd_le16(p+2);
    tp->p_used_bits= p[4];
    tp->data_len   = rd_le24(p+5);
    if (8 + tp->data_len > plen){ tp->done=1; return; }
    tp->sub_ofs = 8; tp->sub_len = tp->data_len;
    tp->dr_ts_per_sample = tsps;
    /* total bits = (len-1)*8 + used_bits (si 0 => 8 en último) */
    uint8_t u = tp->p_used_bits ? tp->p_used_bits : 8;
    tp->dr_total_bits = (tp->sub_len? (tp->sub_len-1)*8 + u : 0);
    tp->dr_abs_bit = 0;

    /* nivel inicial según primer bit (si hay datos) */
    if (tp->dr_total_bits > 0){
        uint8_t b0 = p[tp->sub_ofs];
        TZX_EAR_SET(tp, tp->slice_origin, (b0 & 0x80) ? 1 : 0);
        tp->next_edge_at = tp->slice_origin + tp->dr_ts_per_sample;
    } else {
        /* sin datos: solo pausa */
        tp->next_edge_at = 0;
        if (tp->p_pause_ms){
            tp->pause_end_at = tp->slice_origin + ms_to_tstates(tp->p_pause_ms);
            tzx_next_block(tp);  /* avanzar ya al siguiente bloque */
        } else {
            tzx_next_block(tp);
        }
    }
}

static void tzx_proc_direct(tzx_player_t* tp, uint64_t t_now)
{
    if (tp->dr_abs_bit >= tp->dr_total_bits){
        if (tp->p_pause_ms){
            tp->pause_end_at = t_now + ms_to_tstates(tp->p_pause_ms);
            tp->next_edge_at = 0;
            tzx_next_block(tp);  /* avanzar ya al siguiente bloque */
        } else {
            tzx_next_block(tp);
        }
        return;
    }
    if (t_now < tp->next_edge_at) return;

    /* Avanza un "run" de muestras iguales y programa el siguiente toggle */
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);

    int cur = dr_get_bit(p, tp->sub_ofs, tp->dr_abs_bit);
    uint32_t i = dr_scan_run(p, tp->sub_ofs, tp->dr_abs_bit, tp->dr_total_bits);

    uint32_t run = i - tp->dr_abs_bit;
    tp->dr_abs_bit = i;

    /* Toggle de nivel al final del run */
    TZX_EAR_SET(tp, tp->next_edge_at, cur ? 0 : 1);
    tp->next_edge_at = t_now + (uint64_t)run * (uint64_t)tp->dr_ts_per_sample;
}

/* -------- 0x18 CSW v2 -------- */
static uint32_t csw_next_count(const uint8_t* d, uint32_t len, uint32_t* idx, uint8_t ctype)
{
    if (*idx >= len) return 0;
    if (ctype==1){ /* RLE: 0 => ext 16-bit LE */
        uint8_t b = d[*idx]; (*idx)++;
        if (b!=0) return b;
        if (*idx+2 > len) return 0;
        uint16_t ext = rd_le16(&d[*idx]); (*idx)+=2;
        return ext;
    } else if (ctype==2){ /* Zero-based RLE */
        uint8_t b = d[*idx]; (*idx)++;
        return (uint32_t)b + 1u;
    }
    return 0;
}
static void tzx_init_csw(tzx_player_t* tp)
{
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    if (plen < 4+2+3+1+4){ tp->done=1; return; }
    uint32_t blen = rd_le32(p+0);
    uint16_t pause = rd_le16(p+4);
    uint32_t rate  = rd_le24(p+6);
    uint8_t  ctype = p[9];
    /* uint32_t npulses = rd_le32(p+10); (informativo) */

    if (14 + blen > plen){ tp->done=1; return; }
    tp->p_pause_ms = pause;
    double tsps = TZX_CPU_TSTATES / (double)rate;
    tp->p_0 = (uint16_t)(tsps + 0.5); /* guardamos en p_0 */
    tp->sub_ofs = 14; tp->sub_len = blen;
    tp->i_byte = 0; tp->csw_ctype = ctype;

    /* Programación diferida (se hará en proc) */
    tp->next_edge_at = 0;
}
static void tzx_proc_csw(tzx_player_t* tp, uint64_t t_now)
{
    if (tp->next_edge_at && t_now < tp->next_edge_at) return;

    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    const uint8_t* d = &p[tp->sub_ofs];
    uint32_t idx = tp->i_byte;
    uint32_t cnt = csw_next_count(d, tp->sub_len, &idx, tp->csw_ctype);
    if (cnt==0){
        if (tp->p_pause_ms){
            tp->pause_end_at = t_now + ms_to_tstates(tp->p_pause_ms);
            tp->next_edge_at = 0;
            tzx_next_block(tp);  /* avanzar ya al siguiente bloque */
        } else {
            tzx_next_block(tp);
        }
        return;
    }
    TZX_EAR_TOGGLE(tp, tp->next_edge_at ? tp->next_edge_at : t_now);
    tp->next_edge_at = t_now + (uint64_t)cnt * (uint64_t)tp->p_0;
    tp->i_byte = idx;
}

/* -------- 0x19 Generalized data -------- */
static void tzx_parse_generalized_tables(tzx_player_t* tp)
{
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    uint32_t blen  = rd_le32(p+0);
    tp->gen_blen = blen;
    tp->gen_pause= rd_le16(p+4);
    tp->gen_totp = rd_le32(p+6);
    tp->gen_npp  = p[10];
    tp->gen_asp  = p[11]; if (tp->gen_totp>0 && tp->gen_asp==0) tp->gen_asp=(uint8_t)256;
    tp->gen_totd = rd_le32(p+12);
    tp->gen_npd  = p[16];
    tp->gen_asd  = p[17]; if (tp->gen_totd>0 && tp->gen_asd==0) tp->gen_asd=(uint8_t)256;

    uint32_t q = 18;

    /* alphap */
    tp->gen_ofs_alphaP = 0;
    if (tp->gen_totp>0){
        tp->gen_ofs_alphaP = q;
        for (int i=0;i<tp->gen_asp;i++){
            tzx_symbol_t* s = &tp->gen_symP[i];
            s->flags = p[q+0]; s->npulses = tp->gen_npp;
            int useN = (tp->gen_npp > TZX_MAX_PULSES)? TZX_MAX_PULSES : tp->gen_npp;
            for (int k=0;k<useN;k++) s->pulses[k]=rd_le16(p+q+1+2*k);
            for (int k=useN;k<tp->gen_npp;k++) { /* ignorados */ }
            q += (2*tp->gen_npp + 1);
        }
        tp->gen_ofs_pilotStream = q;
        q += 3 * tp->gen_totp; /* index+rep */
    }

    /* alphad */
    tp->gen_ofs_alphaD = q;
    if (tp->gen_totd>0){
        for (int i=0;i<tp->gen_asd;i++){
            tzx_symbol_t* s = &tp->gen_symD[i];
            s->flags = p[q+0]; s->npulses = tp->gen_npd;
            int useN = (tp->gen_npd > TZX_MAX_PULSES)? TZX_MAX_PULSES : tp->gen_npd;
            for (int k=0;k<useN;k++) s->pulses[k]=rd_le16(p+q+1+2*k);
            q += (2*tp->gen_npd + 1);
        }
        tp->gen_bits = 0; while ((1<<tp->gen_bits) < tp->gen_asd) tp->gen_bits++;
        tp->gen_data_dsize = (tp->gen_bits*tp->gen_totd + 7)/8;
        tp->gen_ofs_dataStream = q;
        q += tp->gen_data_dsize;
    }
    tp->gen_loaded = 1;

    /* inicializa estado */
    tp->gen_phase = (tp->gen_totp>0)? 0 : ((tp->gen_totd>0)?1:2);
    tp->gen_pilot_pos = 0; tp->gen_pilot_rep_left=0;
    tp->gen_cur_sym = NULL; tp->gen_sym_ip=0; tp->gen_sym_rem=0;
    tp->gen_data_pos = 0; tp->gen_data_ofs = tp->gen_ofs_dataStream;
    tp->gen_data_bits_left = 0; tp->gen_data_byte=0;
}

static void gen_start_symbol(tzx_player_t* tp, const tzx_symbol_t* s, uint64_t t_now)
{
    /* aplicar flags de nivel */
    switch (s->flags & 3){
        case 0: TZX_EAR_TOGGLE(tp, t_now); break;
        case 1: /* igual */ break;
        case 2: TZX_EAR_SET(tp, t_now, 0); break;
        case 3: TZX_EAR_SET(tp, t_now, 1); break;
    }
    tp->gen_cur_sym = s;
    tp->gen_sym_ip  = 0;
    tp->gen_sym_rem = s->npulses;

    /* programa primer pulso no-cero */
    while (tp->gen_sym_rem > 0){
        uint16_t dur = s->pulses[tp->gen_sym_ip++];
        tp->gen_sym_rem--;
        if (dur){
            tp->next_edge_at = t_now + dur;
            return;
        }
    }
    tp->gen_cur_sym = NULL; /* símbolo vacío */
    tp->next_edge_at = 0;
}

static int gen_advance_symbol(tzx_player_t* tp, uint64_t t_now)
{
    /* llamado cuando hemos alcanzado next_edge_at: hacemos toggle y buscamos siguiente pulso */
    if (!tp->gen_cur_sym) return 1; /* terminado */
    TZX_EAR_TOGGLE(tp, tp->next_edge_at);
    while (tp->gen_sym_rem > 0){
        uint16_t dur = tp->gen_cur_sym->pulses[tp->gen_sym_ip++];
        tp->gen_sym_rem--;
        if (dur){
            tp->next_edge_at = t_now + dur;
            return 0; /* sigue dentro del símbolo */
        }
    }
    tp->gen_cur_sym = NULL;
    tp->next_edge_at = 0;
    return 1; /* símbolo terminado */
}

static int gen_read_pilot_entry(tzx_player_t* tp, uint8_t* out_sym_idx, uint16_t* out_rep)
{
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    if (tp->gen_pilot_pos >= tp->gen_totp) return 0;
    uint32_t ofs = tp->gen_ofs_pilotStream + 3*tp->gen_pilot_pos;
    *out_sym_idx = p[ofs+0];
    *out_rep     = rd_le16(p+ofs+1);
    tp->gen_pilot_pos++;
    return 1;
}

static int gen_read_data_symbol_index(tzx_player_t* tp, uint32_t* io_ofs, int* io_bits_left, uint8_t* io_byte, uint32_t* out_idx)
{
    uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
    int bits = tp->gen_bits;
    uint32_t idx=0;
    for (int b=0;b<bits;b++){
        if (*io_bits_left==0){
            if (*io_ofs >= tp->gen_ofs_dataStream + tp->gen_data_dsize) return 0;
            *io_byte = p[*io_ofs]; (*io_ofs)++;
            *io_bits_left = 8;
        }
        idx = (idx<<1) | ((*io_byte & 0x80)?1:0); /* MSB-first */
        *io_byte <<=1; (*io_bits_left)--;
    }
    *out_idx = idx;
    return 1;
}

static void tzx_init_gen(tzx_player_t* tp)
{
    tzx_parse_generalized_tables(tp);
    tp->next_edge_at = 0;
    tp->pause_end_at = 0;
    /* arrancar primer símbolo si hay pilot */
    if (tp->gen_phase==0){
        /* leer primera entrada pilot (idx+rep) */
        uint8_t sidx; uint16_t rep;
        if (!gen_read_pilot_entry(tp,&sidx,&rep)){ tp->gen_phase = (tp->gen_totd?1:2); return; }
        tp->gen_pilot_sym_idx = sidx;
        tp->gen_pilot_rep_left = rep;
        gen_start_symbol(tp, &tp->gen_symP[tp->gen_pilot_sym_idx], tp->slice_origin);
    } else if (tp->gen_phase==1){
        /* empezar data stream */
        uint32_t sidx=0;
        if (!gen_read_data_symbol_index(tp, &tp->gen_data_ofs, &tp->gen_data_bits_left, &tp->gen_data_byte, &sidx)){
            tp->gen_phase = 2; return;
        }
        gen_start_symbol(tp, &tp->gen_symD[sidx], tp->slice_origin);
        tp->gen_data_pos = 1;
    } else {
        /* pausa/done */
    }
}

static void tzx_proc_gen(tzx_player_t* tp, uint64_t t_now)
{
    if (!tp->gen_loaded) tzx_parse_generalized_tables(tp);

    /* Si no hay edge programado, intentamos arrancar */
    if (!tp->next_edge_at && tp->gen_cur_sym==NULL){
        if (tp->gen_phase==0){
            if (tp->gen_pilot_rep_left==0){
                /* tomar siguiente entrada */
                uint8_t sidx; uint16_t rep;
                if (!gen_read_pilot_entry(tp,&sidx,&rep)){
                    tp->gen_phase = (tp->gen_totd?1:2);
                } else {
                    tp->gen_pilot_sym_idx = sidx;
                    tp->gen_pilot_rep_left = rep;
                    gen_start_symbol(tp, &tp->gen_symP[tp->gen_pilot_sym_idx], t_now);
                }
            } else {
                /* aún quedan repeticiones del mismo símbolo */
                gen_start_symbol(tp, &tp->gen_symP[tp->gen_pilot_sym_idx], t_now);
            }
            return;
        } else if (tp->gen_phase==1){
            if (tp->gen_data_pos >= tp->gen_totd){
                tp->gen_phase = 2;
            } else {
                uint32_t sidx=0;
                if (!gen_read_data_symbol_index(tp, &tp->gen_data_ofs, &tp->gen_data_bits_left, &tp->gen_data_byte, &sidx)){
                    tp->gen_phase = 2;
                } else {
                    gen_start_symbol(tp, &tp->gen_symD[sidx], t_now);
                    tp->gen_data_pos++;
                }
            }
            return;
        } else {
            /* pausa/done */
            if (tp->gen_pause){
                tp->pause_end_at = t_now + ms_to_tstates(tp->gen_pause);
                tp->next_edge_at = 0; tp->gen_pause = 0;
                tzx_next_block(tp);  /* avanzar ya al siguiente bloque */
                return;
            } else {
                tzx_next_block(tp); return;
            }
        }
    }

    /* Si hay un edge programado y aún no hemos llegado, esperamos */
    if (tp->next_edge_at && t_now < tp->next_edge_at) return;

    /* Consumimos un pulso del símbolo actual */
    if (tp->gen_cur_sym){
        int done = gen_advance_symbol(tp, t_now);
        if (!done) return; /* sigue en el mismo símbolo (siguiente pulso ya programado) */
        /* símbolo terminado: pasamos a repetir o al siguiente */
        if (tp->gen_phase==0){
            if (tp->gen_pilot_rep_left>0) tp->gen_pilot_rep_left--;
            if (tp->gen_pilot_rep_left>0){
                /* repetir el mismo símbolo */
                return;
            } else {
                /* siguiente entrada */
                return;
            }
        } else if (tp->gen_phase==1){
            /* siguiente símbolo de datos */
            return;
        }
    }

    /* Si no hay símbolo y no hay edge, avanzamos de fase en próxima llamada */
}

/* ================ control de flujo y avance global ============ */

static void tzx_next_block(tzx_player_t* tp){ tp->i_blk++; }

/* Aplica bloques de control que no generan señal inmediatamente */
static int tzx_apply_control_blocks(tzx_player_t* tp, int* jumped)
{
    *jumped = 0;
    while (tp->i_blk < tp->nblk){
        uint8_t id = tp->blk[tp->i_blk].id;
        uint32_t plen; const uint8_t* p = cur_payload(tp,&plen);
        switch(id){
        case 0x21:
            if (plen>=1 && plen>=(uint32_t)(1+p[0])){ TZX_TRACEF("[TZX] #%d/%-3d 0x21 Group start: \"%.*s\"\n", tp->i_blk, tp->nblk, p[0], p+1); }
            tzx_next_block(tp); break;
        case 0x22:
            TZX_TRACEF("[TZX] #%d/%-3d 0x22 Group end\n", tp->i_blk, tp->nblk);
            tzx_next_block(tp); break;
        case 0x30:
            if (plen>=1 && plen>=(uint32_t)(1+p[0])){ TZX_TRACEF("[TZX] #%d/%-3d 0x30 Text: \"%.*s\"\n", tp->i_blk, tp->nblk, p[0], p+1); }
            tzx_next_block(tp); break;
        case 0x31:
            if (plen>=2 && plen>=(uint32_t)(2+p[1])){ TZX_TRACEF("[TZX] #%d/%-3d 0x31 Message: \"%.*s\"\n", tp->i_blk, tp->nblk, p[1], p+2); }
            tzx_next_block(tp); break;
        case 0x32:
            TZX_TRACEF("[TZX] #%d/%-3d 0x32 Archive info\n", tp->i_blk, tp->nblk);
            tzx_next_block(tp); break;
        case 0x33:
            TZX_TRACEF("[TZX] #%d/%-3d 0x33 Hardware type\n", tp->i_blk, tp->nblk);
            tzx_next_block(tp); break;
        case 0x35:
            TZX_TRACEF("[TZX] #%d/%-3d 0x35 Custom info\n", tp->i_blk, tp->nblk);
            tzx_next_block(tp); break;
        case 0x5A:
            TZX_TRACEF("[TZX] #%d/%-3d 0x5A Glue block\n", tp->i_blk, tp->nblk);
            tzx_next_block(tp); break;
        case 0x23: {
            if (plen<2){ tp->done=1; return -1; }
            int16_t rel = (int16_t)rd_le16(p+0);
            if (rel==0){ tp->done=1; return 0; }
            int target = tp->i_blk + rel + 1;
            TZX_TRACEF("[TZX] #%d/%-3d 0x23 Jump rel=%d -> #%d\n", tp->i_blk, tp->nblk, rel, target);
            if (target < 0) target = 0;
            if (target >= tp->nblk){ tp->done=1; return 0; }
            tp->i_blk = target; *jumped=1; break;
        }
        case 0x24: { /* loop start (no pila completa aquí; se puede ampliar) */
            if (plen>=2){ TZX_TRACEF("[TZX] #%d/%-3d 0x24 Loop start count=%u\n", tp->i_blk, tp->nblk, rd_le16(p)); }
            tzx_next_block(tp); break;
        }
        case 0x25: { /* loop end */
            TZX_TRACEF("[TZX] #%d/%-3d 0x25 Loop end\n", tp->i_blk, tp->nblk);
            tzx_next_block(tp); break;
        }
        case 0x26: { /* call (simplificado: saltamos a la primera llamada) */
            if (plen<2){ tp->done=1; return -1; }
            if (plen>=4){
                int16_t rel = (int16_t)rd_le16(p+2);
                int target = tp->i_blk + rel + 1;
                TZX_TRACEF("[TZX] #%d/%-3d 0x26 Call rel=%d -> #%d\n", tp->i_blk, tp->nblk, rel, target);
                tp->i_blk = target; *jumped=1;
            } else tzx_next_block(tp);
            break;
        }
        case 0x27:
            TZX_TRACEF("[TZX] #%d/%-3d 0x27 Return from sequence\n", tp->i_blk, tp->nblk);
            tzx_next_block(tp); break; /* return */
        case 0x28:
            TZX_TRACEF("[TZX] #%d/%-3d 0x28 Select block\n", tp->i_blk, tp->nblk);
            tzx_next_block(tp); break; /* select */
        case 0x2A:
            TZX_TRACEF("[TZX] #%d/%-3d 0x2A Stop tape (48K)\n", tp->i_blk, tp->nblk);
            tzx_next_block(tp); break; /* stop 48K (ignorado aquí) */
        case 0x2B: { /* set signal */
            if (plen<5){ tp->done=1; return -1; }
            TZX_TRACEF("[TZX] #%d/%-3d 0x2B Set signal level=%u\n", tp->i_blk, tp->nblk, p[4]);
            TZX_EAR_SET(tp, tp->slice_origin, p[4]?1:0);
            tzx_next_block(tp); break;
        }
        case 0x20: { /* pause/stop */
            if (plen<2){ tp->done=1; return -1; }
            uint16_t ms = rd_le16(p+0);
            if (ms==0){
                TZX_TRACEF("[TZX] #%d/%-3d 0x20 Stop tape\n", tp->i_blk, tp->nblk);
                tp->done=1; return 0;
            }
            TZX_TRACEF("[TZX] #%d/%-3d 0x20 Pause %ums\n", tp->i_blk, tp->nblk, ms);
            tp->pause_end_at = tp->slice_origin + ms_to_tstates(ms);
            tp->next_edge_at = 0;
            tzx_next_block(tp);
            return 0;
        }
        default:
            return 0; /* productor de señal */
        }
    }
    tp->done=1;
    return 0;
}

static void tzx_advance_to(tzx_player_t* tp, uint64_t t_now)
{
    if (!tzx_active(tp)) return;

    /* Process ALL pending edges up to t_now in one call (like tape_advance_to).
     * prev_blk is local so sub-state is cleared whenever i_blk changes. */
    int prev_blk = tp->i_blk;
    int safety = TZX_MAX_EDGES_PER_SLICE;

    while (tzx_active(tp) && safety-- > 0) {

        /* 1. Clear sub-state whenever block index changed */
        if (tp->i_blk != prev_blk) {
            tp->sub_ofs = tp->sub_len = 0;
            tp->pilot_left = 0;
            tp->i_byte = 0; tp->bit_mask = 0x80; tp->subpulse = 0;
            tp->gen_cur_sym = NULL;
            tp->next_edge_at = 0;
            prev_blk = tp->i_blk;
        }

        /* 2. Handle active pause */
        if (tp->pause_end_at) {
            if (t_now >= tp->pause_end_at)
                tp->pause_end_at = 0;
            else
                break;  /* still waiting for pause to end */
        }

        if (tp->i_blk >= tp->nblk) { tp->done = 1; break; }

        /* 3. Skip control/info blocks; may set pause_end_at (block already advanced) */
        int jumped = 0;
        if (tzx_apply_control_blocks(tp, &jumped) != 0) break;
        if (tp->done) break;

        /* If control blocks changed i_blk, restart loop to clear sub-state */
        if (tp->i_blk != prev_blk) continue;

        uint8_t id = tp->blk[tp->i_blk].id;

        /* 4. Initialize new signal block (anchored to t_now for accurate timing) */
        if (tp->sub_ofs == 0 && tp->sub_len == 0 && tp->next_edge_at == 0 &&
            tp->pilot_left == 0 && tp->gen_cur_sym == NULL) {
            tp->slice_origin = t_now;  /* anchor first-edge to current time */
#if TZX_TRACE
            TZX_TRACEF("[TZX] #%d/%-3d 0x%02X %s  init\n",
                tp->i_blk, tp->nblk, id, tzx_block_id_name(id));
#endif
            switch (id) {
            case 0x10: tzx_init_std_or_turbo(tp, 0); break;
            case 0x11: tzx_init_std_or_turbo(tp, 1); break;
            case 0x12: tzx_init_pure_tone(tp); break;
            case 0x13: tzx_init_pulse_seq(tp); break;
            case 0x14: tzx_init_pure_data(tp); break;
            case 0x15: tzx_init_direct(tp); break;
            case 0x18: tzx_init_csw(tp); break;
            case 0x19: tzx_init_gen(tp); break;
            default:   tzx_next_block(tp); continue;
            }
            /* If init triggered a pause (e.g. no-data direct), restart loop */
            if (tp->pause_end_at) continue;
        }

        /* 5. If next event is in the future, done for now */
        if (tp->next_edge_at != 0 && t_now < tp->next_edge_at) break;

        /* 6. Stall guard: track state before proc */
        int      old_blk = tp->i_blk;
        uint64_t old_nea = tp->next_edge_at;
        uint64_t old_pea = tp->pause_end_at;

        /* 7. Process one edge / state transition */
        switch (id) {
        case 0x10: case 0x11: tzx_proc_std_or_turbo(tp, t_now); break;
        case 0x12: tzx_proc_pure_tone(tp, t_now); break;
        case 0x13: tzx_proc_pulse_seq(tp, t_now); break;
        case 0x14: tzx_proc_pure_data(tp, t_now); break;
        case 0x15: tzx_proc_direct(tp, t_now); break;
        case 0x18: tzx_proc_csw(tp, t_now); break;
        case 0x19: tzx_proc_gen(tp, t_now); break;
        default:   tzx_next_block(tp); break;
        }

        if (tp->i_blk >= tp->nblk) { tp->done = 1; break; }

        /* 8. Stall guard: nothing changed → stop to avoid infinite loop */
        if (tp->i_blk == old_blk &&
            tp->next_edge_at == old_nea &&
            tp->pause_end_at == old_pea)
            break;
    }
}

/* ================ Hooks de porción ============================ */
void tzx_begin_slice(tzx_player_t* tp, uint64_t global_frame_origin)
{
    if (!tp) return;
    tp->slice_origin = global_frame_origin;
    tp->frame_origin = global_frame_origin;
}
void tzx_end_slice(tzx_player_t* tp, const Z80Context* cpu, uint64_t* io_new_frame_origin)
{
    if (!tp || !cpu || !io_new_frame_origin) return;
    uint64_t t_now = tp->slice_origin + (uint64_t)cpu->tstates;
    tzx_advance_to(tp, t_now);
    tp->frame_origin = t_now;
    *io_new_frame_origin = tp->frame_origin;
}

/* ================ Consultas ================================ */
int tzx_active(const tzx_player_t* tp){ return tp && tp->playing && !tp->done; }
uint8_t tzx_ear_bit6(const tzx_player_t* tp){ return (tzx_active(tp) && tp->ear_level)?0x40:0x00; }

