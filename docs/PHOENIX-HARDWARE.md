# Phoenix Arcade Hardware Reference

## Overview

Phoenix is a fixed-shooter arcade game released by Amstar Electronics (distributed
by Centuri) in 1980.  The goal of this document is to capture enough detail about the
original hardware to support a high-accuracy emulator.

---

## CPU

| Property | Value |
|----------|-------|
| Chip     | Intel 8080A |
| Clock    | 2.000 MHz |
| Addr bus | 16-bit (64 KB space) |
| Data bus | 8-bit |

The 8080A generates a single maskable interrupt (INT) that the hardware pulses once
per video frame (VBLANK).  There is no NMI line on the stock 8080A; the VBLANK line
is controlled by a latch that the game writes via an I/O port.

---

## Memory Map

```
0x0000 – 0x07FF   ROM bank 0  (phoenix.45, 2 KB)
0x0800 – 0x0FFF   ROM bank 1  (phoenix.46, 2 KB)
0x1000 – 0x17FF   ROM bank 2  (phoenix.47, 2 KB)
0x1800 – 0x1FFF   ROM bank 3  (phoenix.48, 2 KB)
0x2000 – 0x27FF   ROM bank 4  (phoenix.49, 2 KB)  – not present in all sets
0x2800 – 0x2FFF   ROM bank 5  (phoenix.h5-ic50,   2 KB) – not present in all sets
0x4000 – 0x43FF   Video RAM – foreground tile map (32 × 32, 1 byte/tile)
0x4400 – 0x47FF   Video RAM – background tile map (32 × 32, 1 byte/tile)
0x4800 – 0x4BFF   Color / attribute RAM
0x4C00 – 0x4FFF   Work RAM (1 KB)
```

Reads outside mapped areas return 0xFF.  Writes to ROM are ignored.

---

## I/O Port Map

| Port | Dir | Function |
|------|-----|----------|
| 0x00 | IN  | Player 1 controls |
| 0x01 | IN  | Player 2 controls |
| 0x02 | IN  | DIP switch bank |
| 0x03 | IN  | Watchdog / coin inputs |
| 0x04 | OUT | Palette select, coin counter |
| 0x05 | OUT | Sound control – melody section (TMS36XX latch) |
| 0x06 | OUT | Sound control – noise / effect triggers |
| 0x07 | OUT | Bit 0 = IRQ enable; bits 1-4 = background scroll offset |

### Player 1 control byte (port 0x00)

```
Bit 7  – coin insert
Bit 4  – start 2P
Bit 3  – start 1P
Bit 2  – fire
Bit 1  – move right
Bit 0  – move left
```

(Active-low: 0 = pressed.)

---

## Video Hardware

The video system draws two independent 32 × 32 tile planes onto a 256 × 208 pixel
raster.  The monitor is rotated 90°, so the effective playfield is 208 × 256 pixels
in portrait orientation.

* **Tile size**: 8 × 8 pixels.
* **Foreground plane** (sprites / player ship): tile map at 0x4000; character ROM
  `phoenix.b2-4k`.
* **Background plane** (scrolling starfield / alien formations): tile map at 0x4400;
  character ROM `phoenix.b1-4k`.
* **Color RAM** at 0x4800: one byte per 8 × 32-pixel column strip, selecting one of
  eight hardware-defined 2-color palettes.
* Background plane has a horizontal scroll register set via port 0x07 bits 1-4
  (pixel-accurate scroll in 16-pixel steps).

### Palette

Phoenix uses a resistor-ladder DAC to produce 8 colors:

```
Index  R  G  B   Name
  0    0  0  0   Black
  1    0  0  1   Blue
  2    0  1  0   Green
  3    0  1  1   Cyan
  4    1  0  0   Red
  5    1  0  1   Magenta
  6    1  1  0   Yellow
  7    1  1  1   White
```

---

## Sound Hardware

Phoenix has two independent sound sections.

### Melody section

A **TMS36XX** (Texas Instruments) melody generator IC produces the iconic bird-song
sounds.  It has 12 note inputs and is programmed by writing a tone-select byte to
port 0x05.

### Noise / effects section

A discrete transistor circuit generates rocket engine rumble and explosion noise.
The hardware is triggered by bits in port 0x06.  Full discrete emulation is out of
scope for the skeleton; the subsystem is scaffolded for future implementation.

---

## Interrupt Handling

The VBLANK pulse fires at the display refresh rate (~60.6 Hz for NTSC).  The game
enables the INT line by writing bit 0 = 1 to port 0x07.  The 8080A responds by
executing a `RST 7` instruction (vector 0x0038), which jumps to the VBLANK handler.

---

## ROM Files

A standard `phoenix.zip` (Centuri / Amstar set) contains:

| File name        | Size  | Load address | Contents                  |
|------------------|-------|--------------|---------------------------|
| `phoenix.45`     | 2 KB  | 0x0000       | Program ROM 0             |
| `phoenix.46`     | 2 KB  | 0x0800       | Program ROM 1             |
| `phoenix.47`     | 2 KB  | 0x1000       | Program ROM 2             |
| `phoenix.48`     | 2 KB  | 0x1800       | Program ROM 3             |
| `phoenix.b1-4k`  | 4 KB  | (gfx)        | Background character data |
| `phoenix.b2-4k`  | 4 KB  | (gfx)        | Foreground character data |

Some dumps use the alternate names `h1-ic45.1a` … `h4-ic48.4a`.  The loader accepts
both name variants.

---

## References

* **MAME source**: `src/mame/amstar/phoenix.cpp` and `src/mame/amstar/phoenix.h`
  (video, memory map, I/O), `src/mame/audio/phoenix.cpp` (sound hardware).
* **MAME ROM catalog**: `hash/phoenix.xml`.
* **Intel 8080A Datasheet**: Intel order number MCS-80/85 Family User's Manual
  (October 1979).
* **TMS36XX Application Note**: Texas Instruments, "The TMS3600/TMS3601 Complex
  Sound Generator", SPRA012.
* **Arcade history**: https://www.arcade-history.com/?n=phoenix&page=detail&id=2006

---

## Building and Running Locally

### Prerequisites

* GCC (or Clang) with C11 support
* SDL2 development libraries (`libsdl2-dev` on Debian/Ubuntu)
* A copy of `phoenix.zip` (ROM archive)

### Extracting the ROMs

```sh
unzip phoenix.zip -d /path/to/roms
```

The directory must contain the files listed in the ROM table above.

### Compiling

From the repository root:

```sh
make phoenix
```

This produces a `phoenix` binary in the repository root directory.

### Running

```sh
./phoenix -roms /path/to/roms
```

**Optional flags**:

| Flag              | Description                                         |
|-------------------|-----------------------------------------------------|
| `-roms <dir>`     | Directory containing the unpacked ROM files         |
| `-d <level>`      | Debug trace bitmask (see TRACE_* constants in code) |
| `-f`              | Fast mode – disable 60 Hz throttle                  |

### Example

```sh
unzip ~/Downloads/phoenix.zip -d /tmp/phoenix_roms
make phoenix
./phoenix -roms /tmp/phoenix_roms
```

---

*This document was compiled from MAME source analysis, arcade PCB schematics, and
published datasheets.  Corrections and additions are welcome via pull request.*
