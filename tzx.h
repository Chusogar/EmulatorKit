#ifndef TZX_H
#define TZX_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "libz80/z80.h"

/* Reproductor TZX por pulsos:
 *  - tzx_load_file(): carga .tzx en memoria
 *  - tzx_begin_slice()/tzx_end_slice(): sincronización por porciones (t-states)
 *  - tzx_ear_bit6(): bit 6 (EAR) para inyectar en la lectura del puerto FEh
 *  - tzx_play/pause/rewind: control de reproducción
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tzx_player tzx_player_t;

tzx_player_t* tzx_create(void);
void tzx_destroy(tzx_player_t* tp);

int  tzx_load_file(tzx_player_t* tp, const char* path);

void tzx_play(tzx_player_t* tp);
void tzx_pause(tzx_player_t* tp, int pause_on);
void tzx_rewind(tzx_player_t* tp);

void tzx_begin_slice(tzx_player_t* tp, uint64_t global_frame_origin);
void tzx_end_slice  (tzx_player_t* tp, const Z80Context* cpu, uint64_t* io_new_frame_origin);

uint8_t tzx_ear_bit6(const tzx_player_t* tp);
int     tzx_active  (const tzx_player_t* tp);

const char* tzx_last_error(const tzx_player_t* tp);

#ifdef __cplusplus
}
#endif

#endif /* TZX_H */