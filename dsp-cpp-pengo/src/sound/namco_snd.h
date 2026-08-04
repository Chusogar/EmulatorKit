#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace dsp {

// Namco waveform sound generator (3 or 8 voices), ported from namco_snd.pas.
// Pengo uses the 3-voice PROM-backed mode.
class NamcoSnd {
public:
    static constexpr int kSampleRate = 44100;
    static constexpr int kMaxVoices = 8;

    explicit NamcoSnd(int voices = 3);

    void reset();
    void set_enabled(bool enabled) { enabled_ = enabled; }
    bool enabled() const { return enabled_; }

    // 32-byte (3-voice) or 64-byte (8-voice) register file.
    void write_reg(int offset, uint8_t value);
    uint8_t read_reg(int offset) const;

    // Loads the 256-byte waveform PROM used by the 3/8-voice Pac-Man style chip.
    void set_wave_prom(const std::vector<uint8_t>& prom);

    // Generates the next mixed sample at kSampleRate.
    int32_t update();

private:
    struct Voice {
        uint8_t volume = 0;
        uint8_t wave = 0;
        int frequency = 0;
        bool active = false;
        uint32_t position = 0;
    };

    void update_waveform(int offset, uint8_t data);
    void refresh_voice_3(int index);
    void refresh_voice_8(int index);

    int voices_;
    bool enabled_ = true;
    std::array<uint8_t, 0x40> regs_{};
    std::array<uint8_t, 0x100> wave_prom_{};
    std::array<uint8_t, 0x200> decoded_wave_{};
    std::array<Voice, kMaxVoices> voice_{};
};

}  // namespace dsp
