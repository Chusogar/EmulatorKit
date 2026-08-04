# Pengo driver port for [Chusogar/dsp-cpp](https://github.com/Chusogar/dsp-cpp)

This package ports the **Pengo** arcade driver from
[leniad/dsp-emulator](https://github.com/leniad/dsp-emulator)
(`src/arcade/pengo_hw.pas`) into the C++17 + SDL2 emulator **dsp-cpp**.

> **Note:** This cloud agent was started on `Chusogar/EmulatorKit` and does not
> have write access to `Chusogar/dsp-cpp`. Apply the patch (or copy the files)
> into that repository and open the PR there.

## What was ported

| Component | Origin | New files in dsp-cpp |
| --- | --- | --- |
| Pengo driver | `pengo_hw.pas` | `src/drivers/pengo.{h,cpp}` |
| Namco WSG (3 voices) | `namco_snd.pas` | `src/sound/namco_snd.{h,cpp}` |
| Sega Z80 decrypt | `sega_decrypt.pas` | `src/machine/sega_decrypt.{h,cpp}` |
| Z80 M1 / opcode fetch flag | `nz80.pas` `opcode` | `src/cpu/z80.{h,cpp}` |

Also wired into `src/main.cpp`, `CMakeLists.txt`, `README.md` and unit tests.

## Apply to dsp-cpp

Preferred — fetch the ready-made commit from the git bundle (requires
dsp-cpp `main` at `700a45e` or later that contains that commit as ancestor):

```bash
cd dsp-cpp
git fetch /path/to/dsp-cpp-pengo/pengo-driver.bundle cursor/pengo-driver-f6ea:cursor/pengo-driver-f6ea
git checkout cursor/pengo-driver-f6ea
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build
./build/dsp --game pengo /path/to/pengo.zip
git push -u origin cursor/pengo-driver-f6ea   # then open the PR on dsp-cpp
```

Alternatives:

```bash
git checkout -b cursor/pengo-driver-f6ea
git apply /path/to/dsp-cpp-pengo/0001-Add-Pengo-arcade-driver.patch
# or copy the files under src/, tests/, CMakeLists.txt and README.md over the tree
```

The commit SHA is recorded in `SOURCE_COMMIT.txt` (built against dsp-cpp `700a45e`).

## Run

```bash
./build/dsp --game pengo /path/to/pengo.zip
./build/dsp --game pengo --dip 0:0xb0 --dip 1:0xcc /path/to/pengo.zip
```

ROM set (parent `pengo`): `ep1689c.8`, `ep1690b.7`, `ep1691b.15`, `ep1692b.14`,
`ep1693b.21`, `ep1694b.20`, `ep5118b.32`, `ep5119c.31`, `ep1640.92`,
`ep1695.105`, `pr1633.78`, `pr1634.88`, `pr1635.51`.

## Contents of this package

- `pengo-driver.bundle` — preferred: fetchable git commit for dsp-cpp
- `0001-Add-Pengo-arcade-driver.patch` — single commit patch against dsp-cpp `main`
- Full copies of every new/modified file for review without applying the patch
- Updated `CMakeLists.txt`, `README.dsp-cpp.md` (project README) and `tests/tests.cpp`
