#include "drivers/pengo.h"

#include <algorithm>
#include <cstring>

#include "core/rom_loader.h"
#include "machine/sega_decrypt.h"

namespace dsp {
namespace {

const std::vector<RomEntry> kMainRoms = {
    {"ep1689c.8|ep1689.8", 0x1000, 0x0000, 0xf37066a8},
    {"ep1690b.7|ep1690.7", 0x1000, 0x1000, 0xbaf48143},
    {"ep1691b.15|ep1691.15", 0x1000, 0x2000, 0xadf0eba0},
    {"ep1692b.14|ep1692.14", 0x1000, 0x3000, 0xa086d60f},
    {"ep1693b.21|ep1693.21", 0x1000, 0x4000, 0xb72084ec},
    {"ep1694b.20|ep1694.20", 0x1000, 0x5000, 0x94194a89},
    {"ep5118b.32|ep5118.32", 0x1000, 0x6000, 0xaf7b12c4},
    {"ep5119c.31|ep5119.31", 0x1000, 0x7000, 0x933950fe},
};

const std::vector<RomEntry> kPaletteRoms = {
    {"pr1633.78", 0x20, 0x00, 0x3a5844ec},
    {"pr1634.88", 0x400, 0x20, 0x766b139b},
};

const std::vector<RomEntry> kSoundRoms = {
    {"pr1635.51", 0x100, 0x00, 0xc29dea27},
};

const std::vector<RomEntry> kGfxRoms = {
    {"ep1640.92|ep1640.92.bin", 0x2000, 0x0000, 0xd7eec6cd},
    {"ep1695.105|ep1695.105.bin", 0x2000, 0x4000, 0x5bfd26e9},
};

GfxLayout char_layout() {
    GfxLayout layout;
    layout.width = 8;
    layout.height = 8;
    layout.total = 0x200;
    layout.planes = 2;
    layout.char_increment = 16 * 8;
    layout.rotate_cw = true;
    layout.plane_offsets = {0, 4};
    layout.x_offsets = {8 * 8 + 0, 8 * 8 + 1, 8 * 8 + 2, 8 * 8 + 3, 0, 1, 2, 3};
    layout.y_offsets = {0 * 8, 1 * 8, 2 * 8, 3 * 8, 4 * 8, 5 * 8, 6 * 8, 7 * 8};
    return layout;
}

GfxLayout sprite_layout() {
    GfxLayout layout;
    layout.width = 16;
    layout.height = 16;
    layout.total = 0x80;
    layout.planes = 2;
    layout.char_increment = 64 * 8;
    layout.rotate_cw = true;
    layout.plane_offsets = {0, 4};
    layout.x_offsets = {8 * 8,     8 * 8 + 1, 8 * 8 + 2,  8 * 8 + 3,  16 * 8 + 0, 16 * 8 + 1,
                        16 * 8 + 2, 16 * 8 + 3, 24 * 8 + 0, 24 * 8 + 1, 24 * 8 + 2, 24 * 8 + 3,
                        0,          1,          2,          3};
    layout.y_offsets = {0 * 8,  1 * 8,  2 * 8,  3 * 8,  4 * 8,  5 * 8,  6 * 8,  7 * 8,
                        32 * 8, 33 * 8, 34 * 8, 35 * 8, 36 * 8, 37 * 8, 38 * 8, 39 * 8};
    return layout;
}

// Classic Pac-Man / Pengo video RAM offset for a tile at (x, y) in the 28x36 map.
int tile_offset(int tile_x, int tile_y) {
    int sx = 29 - tile_x;
    int sy = tile_y - 2;
    if (sy & 0x20) return sx + ((sy & 0x1f) << 5);
    return sy + (sx << 5);
}

}  // namespace

Pengo::Pengo() : cpu_(kCpuClock), namco_(3) {
    framebuffer_.assign(size_t(kScreenWidth) * kScreenHeight, 0xff000000u);

    cpu_.set_memory_handlers([this](uint16_t address) { return read_byte(address); },
                             [this](uint16_t address, uint8_t value) { write_byte(address, value); });
    cpu_.set_cycle_handler([this](int cycles) { on_cycles(cycles); });
}

bool Pengo::init(const std::string& rom_path, std::string* error) {
    RomLoader loader;
    if (!loader.open(rom_path, error)) return false;

    std::vector<uint8_t> main_rom(0x8000, 0);
    if (!loader.load(kMainRoms, main_rom, error)) return false;
    std::copy(main_rom.begin(), main_rom.end(), memory_.begin());
    decrypt_sega(memory_.data(), opcodes_.data(), 2, 0x8000);

    std::vector<uint8_t> gfx_rom(0x6000, 0);
    if (!loader.load(kGfxRoms, gfx_rom, error)) return false;

    std::vector<uint8_t> prom(0x420, 0);
    if (!loader.load(kPaletteRoms, prom, error)) return false;

    std::vector<uint8_t> wave(0x100, 0);
    if (!loader.load(kSoundRoms, wave, error)) return false;
    namco_.set_wave_prom(wave);

    decode_graphics(std::move(gfx_rom));
    build_palette(prom);
    warnings_ = loader.warnings();

    reset();
    return true;
}

void Pengo::decode_graphics(std::vector<uint8_t> sprite_rom) {
    // Reorder the two 8K gfx ROMs the same way pengo_hw.pas does before convert_gfx.
    if (sprite_rom.size() < 0x6000) sprite_rom.resize(0x6000, 0);
    std::memcpy(sprite_rom.data() + 0x2000, sprite_rom.data() + 0x1000, 0x1000);
    std::memcpy(sprite_rom.data() + 0x1000, sprite_rom.data() + 0x4000, 0x1000);
    std::memcpy(sprite_rom.data() + 0x3000, sprite_rom.data() + 0x5000, 0x1000);

    chars_.decode(char_layout(), sprite_rom);
    std::vector<uint8_t> sprite_region(sprite_rom.begin() + 0x2000, sprite_rom.begin() + 0x4000);
    sprites_.decode(sprite_layout(), sprite_region);
}

void Pengo::build_palette(const std::vector<uint8_t>& prom) {
    const std::vector<int> resistances = {1000, 470, 220};
    auto weights = compute_resistor_weights(0, 255, -1.0,
                                            {{resistances, 0, 0},
                                             {resistances, 0, 0},
                                             {{470, 220}, 0, 0}});

    palette_.fill(0xff000000u);
    for (size_t index = 0; index < 0x20; index++) {
        uint8_t data = prom[index];
        int red = combine_weights(weights[0], {(data >> 0) & 1, (data >> 1) & 1, (data >> 2) & 1});
        int green = combine_weights(weights[1], {(data >> 3) & 1, (data >> 4) & 1, (data >> 5) & 1});
        int blue = combine_weights(weights[2], {(data >> 6) & 1, (data >> 7) & 1});
        palette_[index] = 0xff000000u | (uint32_t(red) << 16) | (uint32_t(green) << 8) | uint32_t(blue);
    }

    // Colour lookup PROM: bank 0 uses pens 0-15, bank 1 uses pens 16-31.
    color_lut_.fill(0);
    for (size_t index = 0; index < 256; index++) {
        uint8_t entry = uint8_t(prom[0x20 + index] & 0x0f);
        color_lut_[index] = entry;
        color_lut_[index + 0x100] = uint8_t(entry + 0x10);
    }
}

void Pengo::reset() {
    cpu_.reset();
    namco_.reset();
    irq_enable_ = false;
    flip_screen_ = false;
    gfx_bank_ = 0;
    pal_bank_ = 0;
    colortable_bank_ = 0;
    in0_ = 0xff;
    in1_ = 0xff;
    dirty_.fill(true);
    tilemap_.fill(0xff000000u);
    composite_.fill(0xff000000u);
    audio_accumulator_ = 0;
    audio_.clear();
}

uint8_t Pengo::read_byte(uint16_t address) {
    if (address <= 0x7fff) {
        return cpu_.opcode_fetch() ? opcodes_[address] : memory_[address];
    }
    if (address >= 0x8000 && address <= 0x8fff) return memory_[address];
    if (address >= 0x9000 && address <= 0x903f) return dsw_b_;
    if (address >= 0x9040 && address <= 0x907f) return dsw_a_;
    if (address >= 0x9080 && address <= 0x90bf) return in1_;
    if (address >= 0x90c0 && address <= 0x90ff) return in0_;
    return 0xff;
}

void Pengo::write_byte(uint16_t address, uint8_t value) {
    if (address <= 0x7fff) return;  // ROM

    if (address >= 0x8000 && address <= 0x87ff) {
        if (memory_[address] != value) {
            dirty_[address & 0x3ff] = true;
            memory_[address] = value;
        }
        return;
    }
    if ((address >= 0x8800 && address <= 0x8fff) || (address >= 0x9020 && address <= 0x902f)) {
        memory_[address] = value;
        return;
    }
    if (address >= 0x9000 && address <= 0x901f) {
        namco_.write_reg(int(address & 0x1f), value);
        return;
    }

    switch (address) {
        case 0x9040:
            irq_enable_ = value != 0;
            if (!irq_enable_) cpu_.set_irq(IrqLine::Clear);
            break;
        case 0x9041:
            namco_.set_enabled(value != 0);
            break;
        case 0x9042:
            if (pal_bank_ != (value & 1)) {
                pal_bank_ = uint8_t(value & 1);
                dirty_.fill(true);
            }
            break;
        case 0x9043:
            flip_screen_ = (value & 1) != 0;
            break;
        case 0x9046:
            if (colortable_bank_ != (value & 1)) {
                colortable_bank_ = uint8_t(value & 1);
                dirty_.fill(true);
            }
            break;
        case 0x9047:
            if (gfx_bank_ != (value & 1)) {
                gfx_bank_ = uint8_t(value & 1);
                dirty_.fill(true);
            }
            break;
        case 0x9070:
            break;  // watchdog
        default:
            break;
    }
}

void Pengo::on_cycles(int cycles) {
    audio_accumulator_ += int64_t(cycles) * NamcoSnd::kSampleRate;
    while (audio_accumulator_ >= kCpuClock) {
        audio_accumulator_ -= kCpuClock;
        int32_t sample = namco_.update();
        audio_.push_back(int16_t(std::clamp(sample, -32768, 32767)));
    }
}

void Pengo::draw_tile(int offset) {
    // `offset` is packed as ((tile_x & 0xff) << 8) | (tile_y & 0xff) by update_video().
    const int tile_x = (offset >> 8) & 0xff;
    const int tile_y = offset & 0xff;
    const int ram_offset = tile_offset(tile_x, tile_y);

    const int color_base =
        (((memory_[0x8400 + ram_offset] & 0x1f) | (colortable_bank_ << 5) | (pal_bank_ << 6)) << 2);
    const int code = memory_[0x8000 + ram_offset] + (gfx_bank_ << 8);
    const uint8_t* pixels = chars_.element(code);

    for (int y = 0; y < 8; y++) {
        uint32_t* target = &tilemap_[size_t((tile_y * 8 + y) * kScreenWidth + tile_x * 8)];
        for (int x = 0; x < 8; x++) {
            uint8_t pen = color_lut_[size_t(pixels[y * 8 + x] + color_base)];
            target[x] = palette_[pen];
        }
    }
}

void Pengo::draw_sprite(int index) {
    const uint8_t attrib = memory_[0x8ff0 + index * 2];
    const int code = (attrib >> 2) | (gfx_bank_ << 6);
    const int color_base =
        (((memory_[0x8ff1 + index * 2] & 0x1f) | (colortable_bank_ << 5) | (pal_bank_ << 6)) << 2);
    const int pos_x = (240 - memory_[0x9020 + index * 2] - 1) & 0xff;
    const int pos_y = 272 - memory_[0x9021 + index * 2];
    const bool flip_x = (attrib & 2) != 0;
    const bool flip_y = (attrib & 1) != 0;

    const uint8_t* pixels = sprites_.element(code);
    for (int y = 0; y < 16; y++) {
        const int screen_y = pos_y + y;
        if (screen_y < 0 || screen_y >= kScreenHeight) continue;
        const int source_y = flip_y ? (15 - y) : y;
        for (int x = 0; x < 16; x++) {
            const int source_x = flip_x ? (15 - x) : x;
            const uint8_t raw = pixels[source_y * 16 + source_x];
            const uint8_t pen = color_lut_[size_t(raw + color_base)];
            // put_gfx_sprite_mask(..., trans=0, mask=0xf): skip when low nibble is 0.
            if ((pen & 0x0f) == 0) continue;
            const int screen_x = (pos_x + x) & 0xff;
            if (screen_x >= kScreenWidth) continue;
            composite_[size_t(screen_y * kScreenWidth + screen_x)] = palette_[pen];
        }
    }
}

void Pengo::update_video() {
    for (int tile_x = 0; tile_x < 28; tile_x++) {
        for (int tile_y = 0; tile_y < 36; tile_y++) {
            const int ram_offset = tile_offset(tile_x, tile_y);
            if (!dirty_[size_t(ram_offset)]) continue;
            draw_tile((tile_x << 8) | tile_y);
            dirty_[size_t(ram_offset)] = false;
        }
    }

    composite_ = tilemap_;
    for (int index = 7; index >= 0; index--) draw_sprite(index);

    for (int y = 0; y < kScreenHeight; y++) {
        for (int x = 0; x < kScreenWidth; x++) {
            uint32_t pixel = composite_[size_t(y * kScreenWidth + x)];
            size_t target = flip_screen_ ? size_t((kScreenHeight - 1 - y) * kScreenWidth +
                                                 (kScreenWidth - 1 - x))
                                        : size_t(y * kScreenWidth + x);
            framebuffer_[target] = pixel;
        }
    }
}

void Pengo::run_frame() {
    const int cycles_per_line = int(kCpuClock / kFramesPerSecond / kScanlines);
    for (int line = 0; line < kScanlines; line++) {
        if (line == 224) {
            update_video();
            if (irq_enable_) cpu_.set_irq(IrqLine::Hold);
        }
        cpu_.run(cycles_per_line);
    }
}

void Pengo::set_inputs(const MachineInputs& inputs) {
    const InputState& player1 = inputs.player1;
    in0_ = 0xff;
    in1_ = 0xff;
    if (player1.up) in0_ &= 0xfe;
    if (player1.down) in0_ &= 0xfd;
    if (player1.left) in0_ &= 0xfb;
    if (player1.right) in0_ &= 0xf7;
    if (inputs.coin1) in0_ &= 0xef;
    if (inputs.coin2) in0_ &= 0xdf;
    if (player1.button1) in0_ &= 0x7f;

    if (player1.start) in1_ &= 0xdf;
    if (inputs.player2.start) in1_ &= 0xbf;
}

void Pengo::set_dip_switch(int bank, uint8_t value) {
    if (bank == 0) dsw_a_ = value;
    if (bank == 1) dsw_b_ = value;
}

void Pengo::drain_audio(std::vector<int16_t>& out) {
    out.insert(out.end(), audio_.begin(), audio_.end());
    audio_.clear();
}

}  // namespace dsp
