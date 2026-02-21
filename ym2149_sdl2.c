#include <SDL2/SDL.h>
#include "emu2149/emu2149.h"
#include "ym2149.h"

#define SAMPLE_SIZE 512
#define FREQUENCY 223721
#define CHANNELS 1
#define CPU_CLK 1789773

void play_buffer(void *, unsigned char *, int);

SDL_AudioSpec spec = {
	.freq = FREQUENCY,
	.format = AUDIO_S16SYS,
	.channels = CHANNELS,
	.samples = SAMPLE_SIZE,
	.callback = play_buffer,
	.userdata = NULL
};

struct ym2149 {
    SDL_AudioDeviceID dev;
    PSG *psg;
};

static PSG *psg;

// The sample rate should be 1/8th of the clock frequency
// See: https://github.com/visrealm/emu2149
struct ym2149 *ym2149_create(uint16_t clk)
{
    struct ym2149 *sn = malloc(sizeof(struct ym2149));
    if (sn == NULL)
    {
        fprintf(stderr, "Out of memory\n");
        exit(1);
    }

    psg = PSG_new(clk, clk / 8);
    PSG_set_quality(psg, 0);
    PSG_reset(psg);

    sn->psg = psg;

    /* TODO: We probably should't be doing this multiple times if
       we have multiple audio devices */
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0)
    {
        fprintf(stderr, "Could not init SDL audio subsystem. %s\n", SDL_GetError());
        return NULL;
    }
    SDL_AudioDeviceID dev = SDL_OpenAudioDevice(NULL, 0, &spec, NULL, 0);
    SDL_PauseAudioDevice(dev, 0);
    return sn;
}

void play_buffer(void *userdata, unsigned char *stream, int len)
{
    static size_t total_sample_count = 0;
    Sint16 *audio_buffer = (Sint16*)stream;
    int bytes_per_sample = CHANNELS * sizeof(Sint16);
    int samples_to_write = len / bytes_per_sample;
    for (int i=0; i<samples_to_write; i++)
    {
        *audio_buffer++ = PSG_calc(psg);
        total_sample_count ++;
    }
}

void ym2149_write(struct ym2149 *sn, uint8_t reg, uint8_t val)
{
    SDL_LockAudioDevice(sn->dev);
    PSG_writeReg(sn->psg, reg, val);
    SDL_UnlockAudioDevice(sn->dev);
}

void ym2149_destroy(struct ym2149 *sn)
{
    SDL_CloseAudioDevice(sn->dev);
    PSG_delete(sn->psg);
    free(sn);
}
