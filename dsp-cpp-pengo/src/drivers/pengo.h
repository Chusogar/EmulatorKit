#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "core/machine.h"
#include "cpu/z80.h"
#include "sound/namco_snd.h"
#include "video/gfx.h"

namespace dsp {

// Pengo (Sega, 1982), ported from pengo_hw.pas.
class Pengo : public Machine {
public:
    static constexpr int kScreenWidth = 224;
    static constexpr int kScreenHeight = 288;
    static constexpr double kFramesPerSecond = 18432000.0 / 3.0 / 384.0 / 264.0;
    static constexpr int kScanlines = 264;
    static constexpr uint32_t kCpuClock = 18432000 / 6;

    Pengo();

    bool init(const std::string& rom_path, std::string* error) override;
    void reset() override;
    void run_frame() override;

    void set_inputs(const MachineInputs& inputs) override;
    void set_dip_switch(int bank, uint8_t value) override;

    const uint32_t* framebuffer() const override { return framebuffer_.data(); }
    int screen_width() const override { return kScreenWidth; }
    int screen_height() const override { return kScreenHeight; }
    double frames_per_second() const override { return kFramesPerSecond; }

    void drain_audio(std::vector<int16_t>& out) override;
    int sample_rate() const override { return NamcoSnd::kSampleRate; }

    const char* title() const override { return "Pengo"; }

private:
    uint8_t read_byte(uint16_t address);
    void write_byte(uint16_t address, uint8_t value);
    void on_cycles(int cycles);

    void decode_graphics(std::vector<uint8_t> sprite_rom);
    void build_palette(const std::vector<uint8_t>& prom);
    void update_video();
    void draw_tile(int offset);
    void draw_sprite(int index);

    Z80 cpu_;
    NamcoSnd namco_;

    std::array<uint8_t, 0x10000> memory_{};
    std::array<uint8_t, 0x8000> opcodes_{};
    std::array<bool, 0x400> dirty_{};
    std::array<uint32_t, 32> palette_{};
    std::array<uint8_t, 0x200> color_lut_{};

    GfxSet chars_;
    GfxSet sprites_;

    // Work surfaces match the 224x288 Pengo bitmap used in pengo_hw.pas.
    std::array<uint32_t, kScreenWidth * kScreenHeight> tilemap_{};
    std::array<uint32_t, kScreenWidth * kScreenHeight> composite_{};
    std::vector<uint32_t> framebuffer_;

    bool irq_enable_ = false;
    bool flip_screen_ = false;
    uint8_t gfx_bank_ = 0;
    uint8_t pal_bank_ = 0;
    uint8_t colortable_bank_ = 0;

    uint8_t in0_ = 0xff;
    uint8_t in1_ = 0xff;
    uint8_t dsw_a_ = 0xb0;
    uint8_t dsw_b_ = 0xcc;

    int64_t audio_accumulator_ = 0;
    std::vector<int16_t> audio_;
};

}  // namespace dsp
