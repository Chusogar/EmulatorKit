#include "sound/namco_snd.h"

#include <algorithm>

namespace dsp {
namespace {

// Resample factors from namco_snd.pas: round(96_000_000 / 44100) and
// round(24_000_000 / 44100).
constexpr int kResample96 = 2177;
constexpr int kResample24 = 544;

}  // namespace

NamcoSnd::NamcoSnd(int voices) : voices_(std::clamp(voices, 1, kMaxVoices)) {}

void NamcoSnd::set_wave_prom(const std::vector<uint8_t>& prom) {
    wave_prom_.fill(0);
    const size_t count = std::min(prom.size(), wave_prom_.size());
    std::copy_n(prom.begin(), count, wave_prom_.begin());
    for (int offset = 0; offset < 0x100; offset++) update_waveform(offset, wave_prom_[size_t(offset)]);
}

void NamcoSnd::update_waveform(int offset, uint8_t data) {
    // PROM mode uses only the low nibble of each byte.
    decoded_wave_[size_t(offset)] = uint8_t(data & 0x0f);
}

void NamcoSnd::reset() {
    enabled_ = true;
    regs_.fill(0);
    for (Voice& voice : voice_) {
        voice = Voice{};
    }
    for (int offset = 0; offset < 0x100; offset++) update_waveform(offset, wave_prom_[size_t(offset)]);
}

void NamcoSnd::write_reg(int offset, uint8_t value) { regs_[size_t(offset & 0x3f)] = value; }

uint8_t NamcoSnd::read_reg(int offset) const { return regs_[size_t(offset & 0x3f)]; }

void NamcoSnd::refresh_voice_3(int index) {
    const int base = 5 * index;
    Voice& voice = voice_[size_t(index)];
    voice.wave = uint8_t(regs_[size_t(0x05 + base)] & 7);
    voice.volume = uint8_t((regs_[size_t(0x15 + base)] & 0x0f) >> 1);

    int frequency = (regs_[size_t(0x14 + base)] & 0x0f) << 16;
    frequency |= (regs_[size_t(0x13 + base)] & 0x0f) << 12;
    frequency |= (regs_[size_t(0x12 + base)] & 0x0f) << 8;
    frequency |= (regs_[size_t(0x11 + base)] & 0x0f) << 4;
    if (index == 0) frequency |= regs_[0x10] & 0x0f;

    voice.frequency = frequency * kResample96;
    if (voice.frequency == 0 || voice.volume == 0) {
        voice.active = false;
        voice.position = 0;
    } else {
        voice.active = true;
    }
}

void NamcoSnd::refresh_voice_8(int index) {
    const int base = (8 * index) + 0x03;
    Voice& voice = voice_[size_t(index)];
    voice.wave = uint8_t(regs_[size_t(0x03 + base)] >> 4);
    voice.volume = uint8_t((regs_[size_t(0x00 + base)] & 0x0f) >> 1);

    int frequency = regs_[size_t(0x01 + base)];
    frequency |= regs_[size_t(0x02 + base)] << 8;
    frequency |= (regs_[size_t(0x03 + base)] & 0x0f) << 16;

    voice.frequency = frequency * kResample24;
    if (voice.frequency == 0 && voice.volume == 0) {
        voice.active = false;
        voice.position = 0;
    } else {
        voice.active = true;
    }
}

int32_t NamcoSnd::update() {
    if (!enabled_) return 0;

    int sample = 0;
    for (int index = 0; index < voices_; index++) {
        if (voices_ == 3) {
            refresh_voice_3(index);
        } else {
            refresh_voice_8(index);
        }
        Voice& voice = voice_[size_t(index)];
        if (!voice.active) continue;
        const uint32_t offset = voice.position;
        const int wave_base = 32 * voice.wave;
        sample += (decoded_wave_[size_t(wave_base + int((offset >> 25) & 0x1f))] * voice.volume) << 6;
        voice.position = offset + uint32_t(voice.frequency);
    }

    sample = (sample / voices_) * 4;
    return std::clamp(sample, -32768, 32767);
}

}  // namespace dsp
