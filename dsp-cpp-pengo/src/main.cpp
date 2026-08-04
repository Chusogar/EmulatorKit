#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "core/machine.h"
#include "drivers/bagman.h"
#include "drivers/doubledragon.h"
#include "drivers/gauntlet.h"
#include "drivers/mikie.h"
#include "drivers/pengo.h"
#include "frontend/sdl_app.h"

namespace {

struct DipSetting {
    int bank;
    uint8_t value;
};

void print_usage(const char* program) {
    std::printf(
        "Usage: %s [options] <romset.zip | rom directory>\n"
        "\n"
        "Games: bagman (default), mikie, gauntlet, ddragon, ddragon2, pengo\n"
        "\n"
        "Options:\n"
        "  --game NAME        game to run: bagman, mikie, gauntlet, ddragon,\n"
        "                     ddragon2 or pengo\n"
        "  --scale N          window scale factor (default 3)\n"
        "  --dip [BANK:]VALUE DIP switch byte, decimal or 0x hex; bagman has one\n"
        "                     bank, mikie has three (0=A, 1=B, 2=C),\n"
        "                     pengo has two (0=A gameplay, 1=B coinage)\n"
        "  --mute             disable audio\n"
        "  --fullscreen       start in full screen\n"
        "  --screenshot FILE  headless mode: render frames and write FILE (BMP)\n"
        "  --frames N         frames to run in headless mode (default 300)\n"
        "  --help             show this help\n"
        "\n"
        "Controls: arrows move, Left Ctrl/Space button 1, Left Alt/Z button 2,\n"
        "          X button 3 (Double Dragon jump),\n"
        "          1/2 start, 5/6 insert coin, P pause, F3 reset, Esc quit.\n",
        program);
}

// Guesses the game from the ROM set name when --game is not given.
std::string guess_game(const std::string& rom_path) {
    std::string lowered;
    for (char character : rom_path) lowered += char(std::tolower(character));
    if (lowered.find("mikie") != std::string::npos) return "mikie";
    if (lowered.find("gauntlet") != std::string::npos) return "gauntlet";
    if (lowered.find("ddragon2") != std::string::npos) return "ddragon2";
    if (lowered.find("ddragon") != std::string::npos) return "ddragon";
    if (lowered.find("pengo") != std::string::npos) return "pengo";
    return "bagman";
}

std::unique_ptr<dsp::Machine> create_machine(const std::string& game) {
    if (game == "bagman") return std::make_unique<dsp::Bagman>();
    if (game == "mikie") return std::make_unique<dsp::Mikie>();
    if (game == "gauntlet") return std::make_unique<dsp::Gauntlet>();
    if (game == "ddragon") {
        return std::make_unique<dsp::DoubleDragon>(dsp::DoubleDragon::Variant::DDragon);
    }
    if (game == "ddragon2") {
        return std::make_unique<dsp::DoubleDragon>(dsp::DoubleDragon::Variant::DDragon2);
    }
    if (game == "pengo") return std::make_unique<dsp::Pengo>();
    return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
    dsp::AppOptions options;
    std::string game;
    std::vector<DipSetting> dips;

    for (int index = 1; index < argc; index++) {
        std::string argument = argv[index];
        auto next = [&](const char* name) -> const char* {
            if (index + 1 >= argc) {
                std::fprintf(stderr, "%s requires a value\n", name);
                std::exit(1);
            }
            return argv[++index];
        };
        if (argument == "--help" || argument == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (argument == "--game") {
            game = next("--game");
        } else if (argument == "--scale") {
            options.scale = std::atoi(next("--scale"));
            if (options.scale < 1) options.scale = 1;
        } else if (argument == "--dip") {
            std::string value = next("--dip");
            size_t separator = value.find(':');
            DipSetting setting{0, 0};
            if (separator == std::string::npos) {
                setting.value = uint8_t(std::strtoul(value.c_str(), nullptr, 0));
            } else {
                setting.bank = std::atoi(value.substr(0, separator).c_str());
                setting.value = uint8_t(std::strtoul(value.c_str() + separator + 1, nullptr, 0));
            }
            dips.push_back(setting);
        } else if (argument == "--mute") {
            options.mute = true;
        } else if (argument == "--fullscreen") {
            options.fullscreen = true;
        } else if (argument == "--screenshot") {
            options.screenshot = next("--screenshot");
            if (options.frames == 0) options.frames = 300;
        } else if (argument == "--frames") {
            options.frames = std::atoi(next("--frames"));
        } else if (!argument.empty() && argument[0] == '-') {
            std::fprintf(stderr, "unknown option: %s\n", argument.c_str());
            return 1;
        } else {
            options.rom_path = argument;
        }
    }

    if (options.rom_path.empty()) {
        print_usage(argv[0]);
        return 1;
    }
    if (game.empty()) game = guess_game(options.rom_path);

    std::unique_ptr<dsp::Machine> machine = create_machine(game);
    if (machine == nullptr) {
        std::fprintf(stderr, "unknown game: %s\n", game.c_str());
        return 1;
    }

    std::string error;
    if (!machine->init(options.rom_path, &error)) {
        std::fprintf(stderr, "cannot start %s: %s\n", game.c_str(), error.c_str());
        return 1;
    }
    for (const DipSetting& setting : dips) machine->set_dip_switch(setting.bank, setting.value);
    for (const std::string& warning : machine->warnings()) {
        std::fprintf(stderr, "warning: %s\n", warning.c_str());
    }

    dsp::SdlApp app(options);
    return app.run(*machine);
}
