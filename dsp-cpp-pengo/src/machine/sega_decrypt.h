#pragma once

#include <cstddef>
#include <cstdint>

namespace dsp {

// Sega Z80 opcode/data decryptor, ported from sega_decrypt.pas.
// `game` selects the conversion table; Pengo uses index 2.
// On entry `data` holds the encrypted ROM. On return `data` holds the
// decrypted data bytes and `opcodes` holds the decrypted opcode bytes.
void decrypt_sega(uint8_t* data, uint8_t* opcodes, int game, size_t length = 0x8000);

}  // namespace dsp
