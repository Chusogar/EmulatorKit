# dsp-cpp

C++17 + SDL2 port of [dsp-emulator](https://github.com/leniad/dsp-emulator) (Free Pascal).
Supported games: **Bagman** (Valadon Automation, 1982), **Mikie** (Konami, 1984),
**Gauntlet** (Atari, 1985), **Double Dragon** and **Double Dragon II** (Technos, 1987/1988),
**Pengo** (Sega, 1982).

## What is ported

| Component | Origin | Notes |
| --- | --- | --- |
| Z80 CPU | `src/cpu/z80/nz80.pas` | Passes the `zexdoc` instruction exerciser (67/67) |
| M6809 CPU | `src/cpu/m6809.pas` | Mikie main CPU |
| AY-3-8910 PSG | `src/snd/ay_8910.pas` | 44100 Hz mono output |
| SN76496 PSG | `src/snd/sn_76496.pas` | Two chips in Mikie |
| PAL16R6 protection | `src/arcade/misc/bagman_pal.pas` | Original fuse map |
| Graphics decoding, palette | `src/misc/gfx_engine.pas`, `pal_engine.pas` | Bit-level layouts and resistor weights |
| Bagman driver | `src/arcade/bagman_hw.pas` | Memory map, video, inputs, DIP switches |
| Mikie driver | `src/arcade/mikie_hw.pas` | M6809 + sound Z80, PROM colour lookup tables, sprites |
| M68000/68010 CPU | `src/cpu/m68000.pas` | Gauntlet main CPU |
| M6502 CPU | `src/cpu/m6502.pas` | Gauntlet sound CPU |
| YM2151 FM, POKEY | `src/snd/fm_2151.pas`, `src/snd/pokey.pas` | Gauntlet sound board |
| SLAPSTIC | `src/arcade/misc/slapstic.pas` | Types 101-107, bank switched protected ROM |
| Atari motion objects | `src/arcade/misc/atari_mo.pas` | SLIP based sprite lists |
| Gauntlet driver | `src/arcade/gauntlet_hw.pas` | Memory map, playfield/char/sprite video, EEPROM, sound communication |
| HD63701Y MCU | `src/cpu/m680x.pas` | Double Dragon sub CPU: internal RAM/ROM, I/O ports, output compare timer |
| MSM5205 ADPCM | `src/snd/msm5205.pas` | Two chips in Double Dragon |
| OKI MSM6295 | `src/snd/oki6295.pas` | Double Dragon II sample player |
| Double Dragon driver | `src/arcade/doubledragon_hw.pas` | Both variants: banked ROM, shared RAM, scroll, sprites, sound CPUs |
| Namco WSG | `src/snd/namco_snd.pas` | 3-voice PROM waveform chip used by Pengo |
| Sega Z80 decrypt | `src/arcade/misc/sega_decrypt.pas` | Opcode/data tables; Pengo uses table 2 |
| Pengo driver | `src/arcade/pengo_hw.pas` | Encrypted Z80, Pac-Man style video, Namco sound, DIPs |
| Front end | `src/misc/main_engine.pas` | SDL2 window, texture, audio queue, keyboard |

## Building

Requirements: CMake >= 3.16, a C++17 compiler, SDL2 and zlib.

```bash
sudo apt-get install cmake libsdl2-dev zlib1g-dev   # Debian/Ubuntu
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This produces `build/dsp` and the unit test binary `build/dsp_tests`.

## Running

ROMs are **not** included. Point the emulator to a `bagman.zip` set or to a directory
holding the individual files:

```bash
./build/dsp /path/to/bagman.zip
./build/dsp --scale 3 --dip 0xfe /path/to/roms/bagman/
./build/dsp --game mikie /path/to/mikie.zip
./build/dsp --game gauntlet /path/to/gauntlet.zip
./build/dsp --game ddragon /path/to/ddragon.zip
./build/dsp --game ddragon2 /path/to/ddragon2.zip
./build/dsp --game pengo /path/to/pengo.zip
```

The game is taken from `--game` (`bagman`, `mikie`, `gauntlet`, `ddragon`,
`ddragon2` or `pengo`); when omitted it is guessed from the ROM set name. Gauntlet accepts both the four player parent set
(SLAPSTIC 104) and the two player `136041-xxx` set (SLAPSTIC 107).

Required files: `e9_b05.bin`, `f9_b06.bin`, `f9_b07.bin`, `k9_b08.bin`, `m9_b09s.bin`,
`n9_b10.bin`, `c1_b01.bin`, `e1_b02.bin`, `f1_b03s.bin`, `j1_b04.bin`, `p3.bin`, `r3.bin`.

Options:

```
--game NAME        game to run: bagman, mikie, gauntlet, ddragon, ddragon2 or pengo
--scale N          window scale factor (default 3)
--dip [BANK:]VALUE DIP switch byte, decimal or 0x hex (bagman: one bank,
                   mikie: 0=A coinage, 1=B gameplay, 2=C flip screen;
                   gauntlet: service switch;
                   double dragon: 0=A coinage/cabinet, 1=B gameplay;
                   pengo: 0=A gameplay, 1=B coinage)
--mute             disable audio
--fullscreen       start in full screen
--screenshot FILE  headless mode: render frames and write FILE (BMP)
--frames N         frames to run in headless mode (default 300)
```

### Controls

| Key | Action |
| --- | --- |
| Arrows | Player 1 movement |
| Left Ctrl / Space | Player 1 button 1 |
| Left Alt / Z | Player 1 button 2 |
| X | Player 1 button 3 (Double Dragon jump) |
| D / G / R / F | Player 2 movement |
| A / S | Player 2 buttons |
| Q | Player 2 button 3 |
| 1, 2 | Start 1P / 2P |
| 5, 6 | Insert coin 1 / 2 |
| P | Pause |
| F3 | Reset |
| Esc | Quit |

### DIP switches (`--dip`)

| Bits | Meaning |
| --- | --- |
| 0-1 | Lives: 3=2, 2=3, 1=4, 0=5 |
| 2 | Coinage |
| 3-4 | Difficulty: 0x18 easy … 0x00 hardest |
| 5 | Language: set = English |
| 6 | Bonus life: set = 30K, clear = 40K |
| 7 | Cabinet: set = upright |

Mikie has three banks (`--dip 0:0xff --dip 1:0x7b --dip 2:0xfe` are the defaults):
bank A holds coin A (low nibble) and coin B (high nibble), bank B holds lives (bits 0-1),
cabinet (bit 2), bonus life (bits 3-4), difficulty (bits 5-6) and demo sounds (bit 7),
and bank C holds flip screen (bit 0) and upright controls (bit 1).

Mikie ROM set: `n14.11c`, `o13.12a`, `o17.12d`, `n10.6e`, `o11.8i`, `001.f1`, `003.f3`,
`005.h1`, `007.h3`, `d19.1i`, `d21.3i`, `d20.2i`, `d22.12h`, `d18.f9`.

Pengo defaults are `--dip 0:0xb0 --dip 1:0xcc`. ROM set: `ep1689c.8`, `ep1690b.7`,
`ep1691b.15`, `ep1692b.14`, `ep1693b.21`, `ep1694b.20`, `ep5118b.32`, `ep5119c.31`,
`ep1640.92`, `ep1695.105`, `pr1633.78`, `pr1634.88`, `pr1635.51`.

## Tests

```bash
ctest --test-dir build          # unit tests (CPU, PAL, graphics, palette, PSG)
```

The Z80 core can additionally be validated with the standard instruction exerciser,
which is not shipped here:

```bash
cmake --build build --target dsp_zexdoc
./build/dsp_zexdoc /path/to/zexdoc.com
```

## Layout

```
src/cpu/        Z80 and M6809 cores
src/sound/      AY-3-8910 and SN76496
src/video/      graphics decoding and resistor based palette helpers
src/machine/    Bagman PAL16R6
src/drivers/    Bagman and Mikie machines (memory map, video, inputs)
src/frontend/   SDL2 front end, driven through the core/machine.h interface
src/core/       ROM loader (directory or zip) and the Machine interface
```
