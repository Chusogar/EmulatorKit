#include "cpu/z80.h"

#include <array>

namespace dsp {
namespace {

// T state tables, taken from the DSP emulator Z80 core (nz80.pas).
constexpr uint8_t kMain[256] = {
    4, 10, 7, 6, 4, 4, 7, 4, 4, 11, 7, 6, 4, 4, 7, 4,
    8, 10, 7, 6, 4, 4, 7, 4, 12, 11, 7, 6, 4, 4, 7, 4,
    7, 10, 16, 6, 4, 4, 7, 4, 7, 11, 16, 6, 4, 4, 7, 4,
    7, 10, 13, 6, 11, 11, 10, 4, 7, 11, 13, 6, 4, 4, 7, 4,
    4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
    4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
    4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
    7, 7, 7, 7, 7, 7, 4, 7, 4, 4, 4, 4, 4, 4, 7, 4,
    4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
    4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
    4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
    4, 4, 4, 4, 4, 4, 7, 4, 4, 4, 4, 4, 4, 4, 7, 4,
    5, 10, 10, 10, 10, 11, 7, 11, 5, 10, 10, 4, 10, 17, 7, 11,
    5, 10, 10, 11, 10, 11, 7, 11, 5, 4, 10, 11, 10, 4, 7, 11,
    5, 10, 10, 19, 10, 11, 7, 11, 5, 4, 10, 4, 10, 4, 7, 11,
    5, 10, 10, 4, 10, 11, 7, 11, 5, 6, 10, 4, 10, 4, 7, 11};

constexpr uint8_t kCb[256] = {
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4,
    4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, 4};

constexpr uint8_t kIndex[256] = {
    4, 10, 7, 6, 4, 4, 7, 4, 4, 11, 7, 6, 4, 4, 7, 4,
    8, 10, 7, 6, 4, 4, 7, 4, 12, 11, 7, 6, 4, 4, 7, 4,
    7, 10, 16, 6, 4, 4, 7, 4, 7, 11, 16, 6, 4, 4, 7, 4,
    7, 10, 13, 6, 19, 19, 15, 4, 7, 11, 13, 6, 4, 4, 7, 4,
    4, 4, 4, 4, 4, 4, 15, 4, 4, 4, 4, 4, 4, 4, 15, 4,
    4, 4, 4, 4, 4, 4, 15, 4, 4, 4, 4, 4, 4, 4, 15, 4,
    4, 4, 4, 4, 4, 4, 15, 4, 4, 4, 4, 4, 4, 4, 15, 4,
    15, 15, 15, 15, 15, 15, 4, 15, 4, 4, 4, 4, 4, 4, 15, 4,
    4, 4, 4, 4, 4, 4, 15, 4, 4, 4, 4, 4, 4, 4, 15, 4,
    4, 4, 4, 4, 4, 4, 15, 4, 4, 4, 4, 4, 4, 4, 15, 4,
    4, 4, 4, 4, 4, 4, 15, 4, 4, 4, 4, 4, 4, 4, 15, 4,
    4, 4, 4, 4, 4, 4, 15, 4, 4, 4, 4, 4, 4, 4, 15, 4,
    5, 10, 10, 10, 10, 11, 7, 11, 5, 10, 10, 7, 10, 17, 7, 11,
    5, 10, 10, 11, 10, 11, 7, 11, 5, 4, 10, 11, 10, 4, 7, 11,
    5, 10, 10, 19, 10, 11, 7, 11, 5, 4, 10, 4, 10, 4, 7, 11,
    5, 10, 10, 4, 10, 11, 7, 11, 5, 6, 10, 4, 10, 4, 7, 11};

constexpr uint8_t kIndexCb[256] = {
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12};

constexpr uint8_t kEd[256] = {
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    8, 8, 11, 16, 4, 10, 4, 5, 8, 8, 11, 16, 4, 10, 4, 5,
    8, 8, 11, 16, 4, 10, 4, 5, 8, 8, 11, 16, 4, 10, 4, 5,
    8, 8, 11, 16, 4, 10, 4, 14, 8, 8, 11, 16, 4, 10, 4, 14,
    8, 8, 11, 16, 4, 10, 4, 4, 8, 8, 11, 16, 4, 10, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    12, 12, 12, 12, 4, 4, 4, 4, 12, 12, 12, 12, 4, 4, 4, 4,
    12, 12, 12, 12, 4, 4, 4, 4, 12, 12, 12, 12, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};

// Extra T states for taken conditional branches / repeated block instructions.
constexpr uint8_t kExtra[256] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    5, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0,
    5, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    5, 5, 5, 5, 0, 0, 0, 0, 5, 5, 5, 5, 0, 0, 0, 0,
    6, 0, 0, 0, 7, 0, 0, 0, 6, 0, 0, 0, 7, 0, 0, 0,
    6, 0, 0, 0, 7, 0, 0, 0, 6, 0, 0, 0, 7, 0, 0, 0,
    6, 0, 0, 0, 7, 0, 0, 0, 6, 0, 0, 0, 7, 0, 0, 0,
    6, 0, 0, 0, 7, 0, 0, 0, 6, 0, 0, 0, 7, 0, 0, 0};

std::array<uint8_t, 256> build_parity() {
    std::array<uint8_t, 256> table{};
    for (int v = 0; v < 256; v++) {
        int bits = 0;
        for (int b = 0; b < 8; b++) bits += (v >> b) & 1;
        table[size_t(v)] = (bits & 1) ? 0 : Z80::PF;
    }
    return table;
}

const std::array<uint8_t, 256> kParity = build_parity();

inline uint8_t sz53(uint8_t value) {
    return uint8_t((value & (Z80::SF | Z80::YF | Z80::XF)) | (value == 0 ? Z80::ZF : 0));
}

inline uint8_t sz53p(uint8_t value) { return uint8_t(sz53(value) | kParity[value]); }

}  // namespace

Z80::Z80(uint32_t clock) : clock_(clock) {
    read_ = [](uint16_t) { return uint8_t(0xff); };
    write_ = [](uint16_t, uint8_t) {};
    in_ = [](uint16_t) { return uint8_t(0xff); };
    out_ = [](uint16_t, uint8_t) {};
    reset();
}

void Z80::set_memory_handlers(ReadHandler read, WriteHandler write) {
    read_ = std::move(read);
    write_ = std::move(write);
}

void Z80::set_io_handlers(InHandler in, OutHandler out) {
    in_ = std::move(in);
    out_ = std::move(out);
}

void Z80::reset() {
    a = f = b = c = d = e = h = l = 0;
    a2 = f2 = b2 = c2 = d2 = e2 = h2 = l2 = 0;
    ix = iy = wz = 0;
    sp = 0xffff;
    pc_ = 0;
    i = r = im = 0;
    iff1 = iff2 = false;
    halted = false;
    after_ei_ = false;
    irq_state_ = IrqLine::Clear;
    nmi_state_ = IrqLine::Clear;
    nmi_latched_ = false;
    irq_vector_ = 0xff;
}

void Z80::set_irq(IrqLine state, uint8_t vector) {
    irq_state_ = state;
    irq_vector_ = vector;
}

void Z80::set_nmi(IrqLine state) {
    nmi_state_ = state;
    if (state == IrqLine::Clear) nmi_latched_ = false;
}

uint8_t Z80::fetch() { return rd(pc_++); }

uint16_t Z80::fetch16() {
    uint16_t value = rd(pc_);
    value = uint16_t(value | (rd(uint16_t(pc_ + 1)) << 8));
    pc_ = uint16_t(pc_ + 2);
    return value;
}

void Z80::push(uint16_t value) {
    sp = uint16_t(sp - 1);
    wr(sp, uint8_t(value >> 8));
    sp = uint16_t(sp - 1);
    wr(sp, uint8_t(value));
}

uint16_t Z80::pop() {
    uint16_t value = rd(sp);
    sp = uint16_t(sp + 1);
    value = uint16_t(value | (rd(sp) << 8));
    sp = uint16_t(sp + 1);
    return value;
}

void Z80::add_a(uint8_t value) {
    uint16_t result = uint16_t(a + value);
    f = uint8_t(sz53(uint8_t(result)) | ((result > 0xff) ? CF : 0) |
                (((a ^ value ^ uint8_t(result)) & HF) ? HF : 0) |
                ((((a ^ value ^ 0x80) & (a ^ uint8_t(result))) & 0x80) ? PF : 0));
    a = uint8_t(result);
}

void Z80::adc_a(uint8_t value) {
    uint8_t carry = f & CF;
    uint16_t result = uint16_t(a + value + carry);
    f = uint8_t(sz53(uint8_t(result)) | ((result > 0xff) ? CF : 0) |
                (((a ^ value ^ uint8_t(result)) & HF) ? HF : 0) |
                ((((a ^ value ^ 0x80) & (a ^ uint8_t(result))) & 0x80) ? PF : 0));
    a = uint8_t(result);
}

void Z80::sub_a(uint8_t value) {
    int result = a - value;
    f = uint8_t(sz53(uint8_t(result)) | NF | ((result < 0) ? CF : 0) |
                (((a ^ value ^ uint8_t(result)) & HF) ? HF : 0) |
                ((((a ^ value) & (a ^ uint8_t(result))) & 0x80) ? PF : 0));
    a = uint8_t(result);
}

void Z80::sbc_a(uint8_t value) {
    uint8_t carry = f & CF;
    int result = a - value - carry;
    f = uint8_t(sz53(uint8_t(result)) | NF | ((result < 0) ? CF : 0) |
                (((a ^ value ^ uint8_t(result)) & HF) ? HF : 0) |
                ((((a ^ value) & (a ^ uint8_t(result))) & 0x80) ? PF : 0));
    a = uint8_t(result);
}

void Z80::and_a(uint8_t value) {
    a &= value;
    f = uint8_t(sz53p(a) | HF);
}

void Z80::xor_a(uint8_t value) {
    a ^= value;
    f = sz53p(a);
}

void Z80::or_a(uint8_t value) {
    a |= value;
    f = sz53p(a);
}

void Z80::cp_a(uint8_t value) {
    int result = a - value;
    f = uint8_t((uint8_t(result) & SF) | ((uint8_t(result) == 0) ? ZF : 0) | NF |
                (value & (YF | XF)) | ((result < 0) ? CF : 0) |
                (((a ^ value ^ uint8_t(result)) & HF) ? HF : 0) |
                ((((a ^ value) & (a ^ uint8_t(result))) & 0x80) ? PF : 0));
}

uint8_t Z80::inc8(uint8_t value) {
    uint8_t result = uint8_t(value + 1);
    f = uint8_t((f & CF) | sz53(result) | ((result == 0x80) ? PF : 0) |
                (((result & 0x0f) == 0) ? HF : 0));
    return result;
}

uint8_t Z80::dec8(uint8_t value) {
    uint8_t result = uint8_t(value - 1);
    f = uint8_t((f & CF) | NF | sz53(result) | ((result == 0x7f) ? PF : 0) |
                (((result & 0x0f) == 0x0f) ? HF : 0));
    return result;
}

uint16_t Z80::add16(uint16_t x, uint16_t y) {
    uint32_t result = uint32_t(x) + y;
    wz = uint16_t(x + 1);
    f = uint8_t((f & (SF | ZF | PF)) | (((x ^ y ^ uint16_t(result)) >> 8) & HF) |
                ((result > 0xffff) ? CF : 0) | ((result >> 8) & (YF | XF)));
    return uint16_t(result);
}

void Z80::adc_hl(uint16_t value) {
    uint16_t x = hl();
    uint32_t result = uint32_t(x) + value + (f & CF);
    wz = uint16_t(x + 1);
    uint16_t res16 = uint16_t(result);
    f = uint8_t(((res16 >> 8) & (SF | YF | XF)) | ((res16 == 0) ? ZF : 0) |
                ((result > 0xffff) ? CF : 0) | (((x ^ value ^ res16) >> 8) & HF) |
                ((((x ^ value ^ 0x8000) & (x ^ res16)) & 0x8000) ? PF : 0));
    set_hl(res16);
}

void Z80::sbc_hl(uint16_t value) {
    uint16_t x = hl();
    int32_t result = int32_t(x) - value - (f & CF);
    wz = uint16_t(x + 1);
    uint16_t res16 = uint16_t(result);
    f = uint8_t(((res16 >> 8) & (SF | YF | XF)) | ((res16 == 0) ? ZF : 0) | NF |
                ((result < 0) ? CF : 0) | (((x ^ value ^ res16) >> 8) & HF) |
                ((((x ^ value) & (x ^ res16)) & 0x8000) ? PF : 0));
    set_hl(res16);
}

void Z80::daa() {
    uint8_t correction = 0;
    uint8_t carry = f & CF;
    if ((f & HF) || (a & 0x0f) > 9) correction |= 0x06;
    if (carry || a > 0x99) {
        correction |= 0x60;
        carry = CF;
    }
    uint8_t before = a;
    if (f & NF) {
        a = uint8_t(a - correction);
    } else {
        a = uint8_t(a + correction);
    }
    f = uint8_t(sz53p(a) | carry | (f & NF) | (((before ^ a) & HF) ? HF : 0));
}

void Z80::rrd() {
    uint8_t value = rd(hl());
    wr(hl(), uint8_t((value >> 4) | (a << 4)));
    a = uint8_t((a & 0xf0) | (value & 0x0f));
    wz = uint16_t(hl() + 1);
    f = uint8_t((f & CF) | sz53p(a));
}

void Z80::rld() {
    uint8_t value = rd(hl());
    wr(hl(), uint8_t((value << 4) | (a & 0x0f)));
    a = uint8_t((a & 0xf0) | (value >> 4));
    wz = uint16_t(hl() + 1);
    f = uint8_t((f & CF) | sz53p(a));
}

uint8_t Z80::rlc(uint8_t value) {
    uint8_t carry = uint8_t(value >> 7);
    uint8_t result = uint8_t((value << 1) | carry);
    f = uint8_t(sz53p(result) | carry);
    return result;
}

uint8_t Z80::rrc(uint8_t value) {
    uint8_t carry = uint8_t(value & 1);
    uint8_t result = uint8_t((value >> 1) | (carry << 7));
    f = uint8_t(sz53p(result) | carry);
    return result;
}

uint8_t Z80::rl(uint8_t value) {
    uint8_t carry = uint8_t(value >> 7);
    uint8_t result = uint8_t((value << 1) | (f & CF));
    f = uint8_t(sz53p(result) | carry);
    return result;
}

uint8_t Z80::rr(uint8_t value) {
    uint8_t carry = uint8_t(value & 1);
    uint8_t result = uint8_t((value >> 1) | ((f & CF) << 7));
    f = uint8_t(sz53p(result) | carry);
    return result;
}

uint8_t Z80::sla(uint8_t value) {
    uint8_t carry = uint8_t(value >> 7);
    uint8_t result = uint8_t(value << 1);
    f = uint8_t(sz53p(result) | carry);
    return result;
}

uint8_t Z80::sra(uint8_t value) {
    uint8_t carry = uint8_t(value & 1);
    uint8_t result = uint8_t((value >> 1) | (value & 0x80));
    f = uint8_t(sz53p(result) | carry);
    return result;
}

uint8_t Z80::sll(uint8_t value) {
    uint8_t carry = uint8_t(value >> 7);
    uint8_t result = uint8_t((value << 1) | 1);
    f = uint8_t(sz53p(result) | carry);
    return result;
}

uint8_t Z80::srl(uint8_t value) {
    uint8_t carry = uint8_t(value & 1);
    uint8_t result = uint8_t(value >> 1);
    f = uint8_t(sz53p(result) | carry);
    return result;
}

void Z80::bit(uint8_t index, uint8_t value, uint8_t xy_source) {
    uint8_t masked = uint8_t(value & (1 << index));
    f = uint8_t((f & CF) | HF | (masked ? (masked & SF) : (ZF | PF)) | (xy_source & (YF | XF)));
}

void Z80::block_ld(int delta, bool repeat) {
    uint8_t value = rd(hl());
    wr(de(), value);
    set_hl(uint16_t(hl() + delta));
    set_de(uint16_t(de() + delta));
    set_bc(uint16_t(bc() - 1));
    uint8_t n = uint8_t(a + value);
    f = uint8_t((f & (SF | ZF | CF)) | ((n & 0x02) ? YF : 0) | ((n & 0x08) ? XF : 0) |
                (bc() != 0 ? PF : 0));
    if (repeat && bc() != 0) {
        pc_ = uint16_t(pc_ - 2);
        wz = uint16_t(pc_ + 1);
        cycles_ += 5;
    }
}

void Z80::block_cp(int delta, bool repeat) {
    uint8_t value = rd(hl());
    uint8_t result = uint8_t(a - value);
    uint8_t half = uint8_t(((a ^ value ^ result) & HF) ? HF : 0);
    set_hl(uint16_t(hl() + delta));
    set_bc(uint16_t(bc() - 1));
    uint8_t n = uint8_t(result - (half ? 1 : 0));
    f = uint8_t((f & CF) | NF | (result & SF) | ((result == 0) ? ZF : 0) | half |
                ((n & 0x02) ? YF : 0) | ((n & 0x08) ? XF : 0) | (bc() != 0 ? PF : 0));
    wz = uint16_t(wz + delta);
    if (repeat && bc() != 0 && result != 0) {
        pc_ = uint16_t(pc_ - 2);
        wz = uint16_t(pc_ + 1);
        cycles_ += 5;
    }
}

void Z80::block_in(int delta, bool repeat) {
    uint8_t value = in_(bc());
    wr(hl(), value);
    wz = uint16_t(bc() + delta);
    b = uint8_t(b - 1);
    set_hl(uint16_t(hl() + delta));
    unsigned sum = unsigned(value) + ((c + delta) & 0xff);
    f = uint8_t(sz53(b) | ((value & 0x80) ? NF : 0) | ((sum > 0xff) ? (HF | CF) : 0) |
                kParity[(sum & 7) ^ b]);
    if (repeat && b != 0) {
        pc_ = uint16_t(pc_ - 2);
        cycles_ += 5;
    }
}

void Z80::block_out(int delta, bool repeat) {
    uint8_t value = rd(hl());
    b = uint8_t(b - 1);
    wz = uint16_t(bc() + delta);
    out_(bc(), value);
    set_hl(uint16_t(hl() + delta));
    unsigned sum = unsigned(value) + l;
    f = uint8_t(sz53(b) | ((value & 0x80) ? NF : 0) | ((sum > 0xff) ? (HF | CF) : 0) |
                kParity[(sum & 7) ^ b]);
    if (repeat && b != 0) {
        pc_ = uint16_t(pc_ - 2);
        cycles_ += 5;
    }
}

int Z80::take_nmi() {
    if (nmi_latched_) return 0;
    halted = false;
    iff2 = iff1;
    iff1 = false;
    push(pc_);
    pc_ = 0x0066;
    wz = pc_;
    r = uint8_t(((r + 1) & 0x7f) | (r & 0x80));
    if (nmi_state_ == IrqLine::Pulse || nmi_state_ == IrqLine::Hold) nmi_state_ = IrqLine::Clear;
    else nmi_latched_ = true;
    return 11;
}

int Z80::take_irq() {
    if (!iff1) return 0;
    halted = false;
    iff1 = iff2 = false;
    if (irq_state_ == IrqLine::Hold || irq_state_ == IrqLine::Pulse) irq_state_ = IrqLine::Clear;
    r = uint8_t(((r + 1) & 0x7f) | (r & 0x80));
    switch (im) {
        case 0: {
            // Only the RST n form used by the supported drivers is decoded here.
            push(pc_);
            pc_ = uint16_t(irq_vector_ & 0x38);
            wz = pc_;
            return 13;
        }
        case 1:
            push(pc_);
            pc_ = 0x0038;
            wz = pc_;
            return 13;
        default: {
            uint16_t addr = uint16_t((i << 8) | irq_vector_);
            push(pc_);
            pc_ = uint16_t(rd(addr) | (rd(uint16_t(addr + 1)) << 8));
            wz = pc_;
            return 19;
        }
    }
}

int Z80::run(int cycles) {
    executed_ = 0;
    while (executed_ < cycles) {
        cycles_ = 0;
        if (!after_ei_) {
            if (nmi_state_ != IrqLine::Clear) {
                cycles_ += take_nmi();
            } else if (irq_state_ != IrqLine::Clear) {
                cycles_ += take_irq();
            }
        }
        after_ei_ = false;

        if (halted) {
            cycles_ += 4;
            r = uint8_t(((r + 1) & 0x7f) | (r & 0x80));
            executed_ += cycles_;
            if (cycle_handler_) cycle_handler_(cycles_);
            continue;
        }

        opcode_fetch_ = true;
        uint8_t opcode = fetch();
        opcode_fetch_ = false;
        r = uint8_t(((r + 1) & 0x7f) | (r & 0x80));
        cycles_ += kMain[opcode];

        switch (opcode) {
            case 0x00: break;  // nop
            case 0x01: set_bc(fetch16()); break;
            case 0x02: wr(bc(), a); wz = uint16_t(((bc() + 1) & 0xff) | (a << 8)); break;
            case 0x03: set_bc(uint16_t(bc() + 1)); break;
            case 0x04: b = inc8(b); break;
            case 0x05: b = dec8(b); break;
            case 0x06: b = fetch(); break;
            case 0x07:  // rlca
                f = uint8_t((f & (SF | ZF | PF)) | (a >> 7));
                a = uint8_t((a << 1) | (a >> 7));
                f = uint8_t(f | (a & (YF | XF)));
                break;
            case 0x08:
                std::swap(a, a2);
                std::swap(f, f2);
                break;
            case 0x09: set_hl(add16(hl(), bc())); break;
            case 0x0a: a = rd(bc()); wz = uint16_t(bc() + 1); break;
            case 0x0b: set_bc(uint16_t(bc() - 1)); break;
            case 0x0c: c = inc8(c); break;
            case 0x0d: c = dec8(c); break;
            case 0x0e: c = fetch(); break;
            case 0x0f:  // rrca
                f = uint8_t((f & (SF | ZF | PF)) | (a & CF));
                a = uint8_t((a >> 1) | (a << 7));
                f = uint8_t(f | (a & (YF | XF)));
                break;
            case 0x10: {  // djnz
                int8_t offset = int8_t(fetch());
                b = uint8_t(b - 1);
                if (b != 0) {
                    pc_ = uint16_t(pc_ + offset);
                    wz = pc_;
                    cycles_ += kExtra[opcode];
                }
                break;
            }
            case 0x11: set_de(fetch16()); break;
            case 0x12: wr(de(), a); wz = uint16_t(((de() + 1) & 0xff) | (a << 8)); break;
            case 0x13: set_de(uint16_t(de() + 1)); break;
            case 0x14: d = inc8(d); break;
            case 0x15: d = dec8(d); break;
            case 0x16: d = fetch(); break;
            case 0x17: {  // rla
                uint8_t carry = uint8_t(a >> 7);
                a = uint8_t((a << 1) | (f & CF));
                f = uint8_t((f & (SF | ZF | PF)) | carry | (a & (YF | XF)));
                break;
            }
            case 0x18: {  // jr
                int8_t offset = int8_t(fetch());
                pc_ = uint16_t(pc_ + offset);
                wz = pc_;
                break;
            }
            case 0x19: set_hl(add16(hl(), de())); break;
            case 0x1a: a = rd(de()); wz = uint16_t(de() + 1); break;
            case 0x1b: set_de(uint16_t(de() - 1)); break;
            case 0x1c: e = inc8(e); break;
            case 0x1d: e = dec8(e); break;
            case 0x1e: e = fetch(); break;
            case 0x1f: {  // rra
                uint8_t carry = uint8_t(a & CF);
                a = uint8_t((a >> 1) | ((f & CF) << 7));
                f = uint8_t((f & (SF | ZF | PF)) | carry | (a & (YF | XF)));
                break;
            }
            case 0x20:
            case 0x28:
            case 0x30:
            case 0x38: {  // jr cc
                int8_t offset = int8_t(fetch());
                bool taken = false;
                switch (opcode) {
                    case 0x20: taken = !(f & ZF); break;
                    case 0x28: taken = (f & ZF) != 0; break;
                    case 0x30: taken = !(f & CF); break;
                    default: taken = (f & CF) != 0; break;
                }
                if (taken) {
                    pc_ = uint16_t(pc_ + offset);
                    wz = pc_;
                    cycles_ += kExtra[opcode];
                }
                break;
            }
            case 0x21: set_hl(fetch16()); break;
            case 0x22: {
                uint16_t addr = fetch16();
                wr(addr, l);
                wr(uint16_t(addr + 1), h);
                wz = uint16_t(addr + 1);
                break;
            }
            case 0x23: set_hl(uint16_t(hl() + 1)); break;
            case 0x24: h = inc8(h); break;
            case 0x25: h = dec8(h); break;
            case 0x26: h = fetch(); break;
            case 0x27: daa(); break;
            case 0x29: set_hl(add16(hl(), hl())); break;
            case 0x2a: {
                uint16_t addr = fetch16();
                l = rd(addr);
                h = rd(uint16_t(addr + 1));
                wz = uint16_t(addr + 1);
                break;
            }
            case 0x2b: set_hl(uint16_t(hl() - 1)); break;
            case 0x2c: l = inc8(l); break;
            case 0x2d: l = dec8(l); break;
            case 0x2e: l = fetch(); break;
            case 0x2f:  // cpl
                a = uint8_t(~a);
                f = uint8_t((f & (SF | ZF | PF | CF)) | HF | NF | (a & (YF | XF)));
                break;
            case 0x31: sp = fetch16(); break;
            case 0x32: {
                uint16_t addr = fetch16();
                wr(addr, a);
                wz = uint16_t(((addr + 1) & 0xff) | (a << 8));
                break;
            }
            case 0x33: sp = uint16_t(sp + 1); break;
            case 0x34: wr(hl(), inc8(rd(hl()))); break;
            case 0x35: wr(hl(), dec8(rd(hl()))); break;
            case 0x36: wr(hl(), fetch()); break;
            case 0x37:  // scf
                f = uint8_t((f & (SF | ZF | PF)) | CF | (a & (YF | XF)));
                break;
            case 0x39: set_hl(add16(hl(), sp)); break;
            case 0x3a: {
                uint16_t addr = fetch16();
                a = rd(addr);
                wz = uint16_t(addr + 1);
                break;
            }
            case 0x3b: sp = uint16_t(sp - 1); break;
            case 0x3c: a = inc8(a); break;
            case 0x3d: a = dec8(a); break;
            case 0x3e: a = fetch(); break;
            case 0x3f:  // ccf
                f = uint8_t((f & (SF | ZF | PF)) | ((f & CF) ? HF : CF) | (a & (YF | XF)));
                break;
            case 0x76: halted = true; break;
            default:
                if (opcode >= 0x40 && opcode <= 0x7f) {  // ld r,r'
                    static uint8_t Z80::*const regs[8] = {&Z80::b, &Z80::c, &Z80::d, &Z80::e,
                                                          &Z80::h, &Z80::l, nullptr, &Z80::a};
                    uint8_t dst = uint8_t((opcode >> 3) & 7);
                    uint8_t src = uint8_t(opcode & 7);
                    uint8_t value = (src == 6) ? rd(hl()) : this->*regs[src];
                    if (dst == 6) {
                        wr(hl(), value);
                    } else {
                        this->*regs[dst] = value;
                    }
                } else if (opcode >= 0x80 && opcode <= 0xbf) {  // alu a,r
                    static uint8_t Z80::*const regs[8] = {&Z80::b, &Z80::c, &Z80::d, &Z80::e,
                                                          &Z80::h, &Z80::l, nullptr, &Z80::a};
                    uint8_t src = uint8_t(opcode & 7);
                    uint8_t value = (src == 6) ? rd(hl()) : this->*regs[src];
                    switch ((opcode >> 3) & 7) {
                        case 0: add_a(value); break;
                        case 1: adc_a(value); break;
                        case 2: sub_a(value); break;
                        case 3: sbc_a(value); break;
                        case 4: and_a(value); break;
                        case 5: xor_a(value); break;
                        case 6: or_a(value); break;
                        default: cp_a(value); break;
                    }
                } else {
                    switch (opcode) {
                        case 0xc0:
                        case 0xc8:
                        case 0xd0:
                        case 0xd8:
                        case 0xe0:
                        case 0xe8:
                        case 0xf0:
                        case 0xf8: {  // ret cc
                            bool taken = false;
                            switch ((opcode >> 3) & 7) {
                                case 0: taken = !(f & ZF); break;
                                case 1: taken = (f & ZF) != 0; break;
                                case 2: taken = !(f & CF); break;
                                case 3: taken = (f & CF) != 0; break;
                                case 4: taken = !(f & PF); break;
                                case 5: taken = (f & PF) != 0; break;
                                case 6: taken = !(f & SF); break;
                                default: taken = (f & SF) != 0; break;
                            }
                            if (taken) {
                                pc_ = pop();
                                wz = pc_;
                                cycles_ += kExtra[opcode];
                            }
                            break;
                        }
                        case 0xc1: set_bc(pop()); break;
                        case 0xd1: set_de(pop()); break;
                        case 0xe1: set_hl(pop()); break;
                        case 0xf1: set_af(pop()); break;
                        case 0xc5: push(bc()); break;
                        case 0xd5: push(de()); break;
                        case 0xe5: push(hl()); break;
                        case 0xf5: push(af()); break;
                        case 0xc2:
                        case 0xca:
                        case 0xd2:
                        case 0xda:
                        case 0xe2:
                        case 0xea:
                        case 0xf2:
                        case 0xfa: {  // jp cc,nn
                            uint16_t addr = fetch16();
                            wz = addr;
                            bool taken = false;
                            switch ((opcode >> 3) & 7) {
                                case 0: taken = !(f & ZF); break;
                                case 1: taken = (f & ZF) != 0; break;
                                case 2: taken = !(f & CF); break;
                                case 3: taken = (f & CF) != 0; break;
                                case 4: taken = !(f & PF); break;
                                case 5: taken = (f & PF) != 0; break;
                                case 6: taken = !(f & SF); break;
                                default: taken = (f & SF) != 0; break;
                            }
                            if (taken) pc_ = addr;
                            break;
                        }
                        case 0xc3: pc_ = fetch16(); wz = pc_; break;
                        case 0xc4:
                        case 0xcc:
                        case 0xd4:
                        case 0xdc:
                        case 0xe4:
                        case 0xec:
                        case 0xf4:
                        case 0xfc: {  // call cc,nn
                            uint16_t addr = fetch16();
                            wz = addr;
                            bool taken = false;
                            switch ((opcode >> 3) & 7) {
                                case 0: taken = !(f & ZF); break;
                                case 1: taken = (f & ZF) != 0; break;
                                case 2: taken = !(f & CF); break;
                                case 3: taken = (f & CF) != 0; break;
                                case 4: taken = !(f & PF); break;
                                case 5: taken = (f & PF) != 0; break;
                                case 6: taken = !(f & SF); break;
                                default: taken = (f & SF) != 0; break;
                            }
                            if (taken) {
                                push(pc_);
                                pc_ = addr;
                                cycles_ += kExtra[opcode];
                            }
                            break;
                        }
                        case 0xc6: add_a(fetch()); break;
                        case 0xce: adc_a(fetch()); break;
                        case 0xd6: sub_a(fetch()); break;
                        case 0xde: sbc_a(fetch()); break;
                        case 0xe6: and_a(fetch()); break;
                        case 0xee: xor_a(fetch()); break;
                        case 0xf6: or_a(fetch()); break;
                        case 0xfe: cp_a(fetch()); break;
                        case 0xc7:
                        case 0xcf:
                        case 0xd7:
                        case 0xdf:
                        case 0xe7:
                        case 0xef:
                        case 0xf7:
                        case 0xff:  // rst
                            push(pc_);
                            pc_ = uint16_t(opcode & 0x38);
                            wz = pc_;
                            break;
                        case 0xc9: pc_ = pop(); wz = pc_; break;
                        case 0xcb: exec_cb(); break;
                        case 0xcd: {
                            uint16_t addr = fetch16();
                            push(pc_);
                            pc_ = addr;
                            wz = pc_;
                            break;
                        }
                        case 0xd3: {  // out (n),a
                            uint8_t port = fetch();
                            out_(uint16_t((a << 8) | port), a);
                            wz = uint16_t((a << 8) | ((port + 1) & 0xff));
                            break;
                        }
                        case 0xd9:
                            std::swap(b, b2);
                            std::swap(c, c2);
                            std::swap(d, d2);
                            std::swap(e, e2);
                            std::swap(h, h2);
                            std::swap(l, l2);
                            break;
                        case 0xdb: {  // in a,(n)
                            uint8_t port = fetch();
                            uint16_t addr = uint16_t((a << 8) | port);
                            a = in_(addr);
                            wz = uint16_t(addr + 1);
                            break;
                        }
                        case 0xdd: exec_index(&ix); break;
                        case 0xe3: {  // ex (sp),hl
                            uint16_t value = rd(sp) | uint16_t(rd(uint16_t(sp + 1)) << 8);
                            wr(sp, l);
                            wr(uint16_t(sp + 1), h);
                            set_hl(value);
                            wz = value;
                            break;
                        }
                        case 0xe9: pc_ = hl(); break;
                        case 0xeb:
                            std::swap(d, h);
                            std::swap(e, l);
                            break;
                        case 0xed: exec_ed(); break;
                        case 0xf3: iff1 = iff2 = false; break;
                        case 0xf9: sp = hl(); break;
                        case 0xfb:
                            iff1 = iff2 = true;
                            after_ei_ = true;
                            break;
                        case 0xfd: exec_index(&iy); break;
                        default: break;
                    }
                }
                break;
        }

        executed_ += cycles_;
        if (cycle_handler_) cycle_handler_(cycles_);
    }
    return executed_;
}

void Z80::exec_cb() {
    opcode_fetch_ = true;
    uint8_t opcode = fetch();
    opcode_fetch_ = false;
    r = uint8_t(((r + 1) & 0x7f) | (r & 0x80));
    cycles_ += kCb[opcode];

    static uint8_t Z80::*const regs[8] = {&Z80::b, &Z80::c, &Z80::d, &Z80::e,
                                          &Z80::h, &Z80::l, nullptr, &Z80::a};
    uint8_t index = uint8_t(opcode & 7);
    uint8_t value = (index == 6) ? rd(hl()) : this->*regs[index];
    uint8_t op = uint8_t(opcode >> 6);
    uint8_t bit_index = uint8_t((opcode >> 3) & 7);

    if (op == 1) {  // bit
        bit(bit_index, value, (index == 6) ? uint8_t(wz >> 8) : value);
        return;
    }

    uint8_t result;
    if (op == 0) {
        switch (bit_index) {
            case 0: result = rlc(value); break;
            case 1: result = rrc(value); break;
            case 2: result = rl(value); break;
            case 3: result = rr(value); break;
            case 4: result = sla(value); break;
            case 5: result = sra(value); break;
            case 6: result = sll(value); break;
            default: result = srl(value); break;
        }
    } else if (op == 2) {
        result = uint8_t(value & ~(1 << bit_index));
    } else {
        result = uint8_t(value | (1 << bit_index));
    }

    if (index == 6) {
        wr(hl(), result);
    } else {
        this->*regs[index] = result;
    }
}

void Z80::exec_ed() {
    opcode_fetch_ = true;
    uint8_t opcode = fetch();
    opcode_fetch_ = false;
    r = uint8_t(((r + 1) & 0x7f) | (r & 0x80));
    cycles_ += kEd[opcode];

    static uint8_t Z80::*const regs[8] = {&Z80::b, &Z80::c, &Z80::d, &Z80::e,
                                          &Z80::h, &Z80::l, nullptr, &Z80::a};

    switch (opcode) {
        case 0x40: case 0x48: case 0x50: case 0x58:
        case 0x60: case 0x68: case 0x70: case 0x78: {  // in r,(c)
            uint8_t value = in_(bc());
            wz = uint16_t(bc() + 1);
            f = uint8_t((f & CF) | sz53p(value));
            uint8_t dst = uint8_t((opcode >> 3) & 7);
            if (dst != 6) this->*regs[dst] = value;
            break;
        }
        case 0x41: case 0x49: case 0x51: case 0x59:
        case 0x61: case 0x69: case 0x71: case 0x79: {  // out (c),r
            uint8_t src = uint8_t((opcode >> 3) & 7);
            uint8_t value = (src == 6) ? 0 : this->*regs[src];
            out_(bc(), value);
            wz = uint16_t(bc() + 1);
            break;
        }
        case 0x42: case 0x52: case 0x62: case 0x72: {  // sbc hl,rr
            uint16_t value = 0;
            switch ((opcode >> 4) & 3) {
                case 0: value = bc(); break;
                case 1: value = de(); break;
                case 2: value = hl(); break;
                default: value = sp; break;
            }
            sbc_hl(value);
            break;
        }
        case 0x4a: case 0x5a: case 0x6a: case 0x7a: {  // adc hl,rr
            uint16_t value = 0;
            switch ((opcode >> 4) & 3) {
                case 0: value = bc(); break;
                case 1: value = de(); break;
                case 2: value = hl(); break;
                default: value = sp; break;
            }
            adc_hl(value);
            break;
        }
        case 0x43: case 0x53: case 0x63: case 0x73: {  // ld (nn),rr
            uint16_t addr = fetch16();
            uint16_t value = 0;
            switch ((opcode >> 4) & 3) {
                case 0: value = bc(); break;
                case 1: value = de(); break;
                case 2: value = hl(); break;
                default: value = sp; break;
            }
            wr(addr, uint8_t(value));
            wr(uint16_t(addr + 1), uint8_t(value >> 8));
            wz = uint16_t(addr + 1);
            break;
        }
        case 0x4b: case 0x5b: case 0x6b: case 0x7b: {  // ld rr,(nn)
            uint16_t addr = fetch16();
            uint16_t value = uint16_t(rd(addr) | (rd(uint16_t(addr + 1)) << 8));
            wz = uint16_t(addr + 1);
            switch ((opcode >> 4) & 3) {
                case 0: set_bc(value); break;
                case 1: set_de(value); break;
                case 2: set_hl(value); break;
                default: sp = value; break;
            }
            break;
        }
        case 0x44: case 0x4c: case 0x54: case 0x5c:
        case 0x64: case 0x6c: case 0x74: case 0x7c: {  // neg
            uint8_t value = a;
            a = 0;
            sub_a(value);
            break;
        }
        case 0x45: case 0x55: case 0x65: case 0x75:
        case 0x5d: case 0x6d: case 0x7d:  // retn
        case 0x4d:                        // reti
            iff1 = iff2;
            pc_ = pop();
            wz = pc_;
            break;
        case 0x46: case 0x4e: case 0x66: case 0x6e: im = 0; break;
        case 0x56: case 0x76: im = 1; break;
        case 0x5e: case 0x7e: im = 2; break;
        case 0x47: i = a; break;
        case 0x4f: r = a; break;
        case 0x57:
            a = i;
            f = uint8_t((f & CF) | sz53(a) | (iff2 ? PF : 0));
            break;
        case 0x5f:
            a = r;
            f = uint8_t((f & CF) | sz53(a) | (iff2 ? PF : 0));
            break;
        case 0x67: rrd(); break;
        case 0x6f: rld(); break;
        case 0xa0: block_ld(1, false); break;
        case 0xa8: block_ld(-1, false); break;
        case 0xb0: block_ld(1, true); break;
        case 0xb8: block_ld(-1, true); break;
        case 0xa1: block_cp(1, false); break;
        case 0xa9: block_cp(-1, false); break;
        case 0xb1: block_cp(1, true); break;
        case 0xb9: block_cp(-1, true); break;
        case 0xa2: block_in(1, false); break;
        case 0xaa: block_in(-1, false); break;
        case 0xb2: block_in(1, true); break;
        case 0xba: block_in(-1, true); break;
        case 0xa3: block_out(1, false); break;
        case 0xab: block_out(-1, false); break;
        case 0xb3: block_out(1, true); break;
        case 0xbb: block_out(-1, true); break;
        default: break;  // nop
    }
}

void Z80::exec_index(uint16_t* index_reg) {
    opcode_fetch_ = true;
    uint8_t opcode = fetch();
    opcode_fetch_ = false;
    r = uint8_t(((r + 1) & 0x7f) | (r & 0x80));
    cycles_ += kIndex[opcode];

    uint8_t index_high = uint8_t(*index_reg >> 8);
    uint8_t index_low = uint8_t(*index_reg);
    auto set_index = [&](uint8_t hi, uint8_t lo) { *index_reg = uint16_t((hi << 8) | lo); };
    auto ea = [&]() {
        int8_t offset = int8_t(fetch());
        wz = uint16_t(*index_reg + offset);
        return wz;
    };

    static uint8_t Z80::*const regs[8] = {&Z80::b, &Z80::c, &Z80::d, &Z80::e,
                                          &Z80::h, &Z80::l, nullptr, &Z80::a};

    switch (opcode) {
        case 0x09: *index_reg = add16(*index_reg, bc()); break;
        case 0x19: *index_reg = add16(*index_reg, de()); break;
        case 0x21: *index_reg = fetch16(); break;
        case 0x22: {
            uint16_t addr = fetch16();
            wr(addr, index_low);
            wr(uint16_t(addr + 1), index_high);
            wz = uint16_t(addr + 1);
            break;
        }
        case 0x23: *index_reg = uint16_t(*index_reg + 1); break;
        case 0x24: set_index(inc8(index_high), index_low); break;
        case 0x25: set_index(dec8(index_high), index_low); break;
        case 0x26: set_index(fetch(), index_low); break;
        case 0x29: *index_reg = add16(*index_reg, *index_reg); break;
        case 0x2a: {
            uint16_t addr = fetch16();
            *index_reg = uint16_t(rd(addr) | (rd(uint16_t(addr + 1)) << 8));
            wz = uint16_t(addr + 1);
            break;
        }
        case 0x2b: *index_reg = uint16_t(*index_reg - 1); break;
        case 0x2c: set_index(index_high, inc8(index_low)); break;
        case 0x2d: set_index(index_high, dec8(index_low)); break;
        case 0x2e: set_index(index_high, fetch()); break;
        case 0x34: {
            uint16_t addr = ea();
            wr(addr, inc8(rd(addr)));
            break;
        }
        case 0x35: {
            uint16_t addr = ea();
            wr(addr, dec8(rd(addr)));
            break;
        }
        case 0x36: {
            uint16_t addr = ea();
            wr(addr, fetch());
            break;
        }
        case 0x39: *index_reg = add16(*index_reg, sp); break;
        case 0xcb: exec_index_cb(index_reg); break;
        case 0xe1: *index_reg = pop(); break;
        case 0xe3: {
            uint16_t value = rd(sp) | uint16_t(rd(uint16_t(sp + 1)) << 8);
            wr(sp, index_low);
            wr(uint16_t(sp + 1), index_high);
            *index_reg = value;
            wz = value;
            break;
        }
        case 0xe5: push(*index_reg); break;
        case 0xe9: pc_ = *index_reg; break;
        case 0xf9: sp = *index_reg; break;
        default:
            if (opcode >= 0x40 && opcode <= 0x7f && opcode != 0x76) {  // ld with index regs
                uint8_t dst = uint8_t((opcode >> 3) & 7);
                uint8_t src = uint8_t(opcode & 7);
                // With a displacement the H and L operands keep their normal
                // meaning, only the register to register forms use IXH/IXL.
                if (src == 6) {
                    uint8_t value = rd(ea());
                    this->*regs[dst] = value;
                } else if (dst == 6) {
                    wr(ea(), this->*regs[src]);
                } else {
                    uint8_t value = (src == 4)   ? index_high
                                    : (src == 5) ? index_low
                                                 : this->*regs[src];
                    if (dst == 4) {
                        set_index(value, uint8_t(*index_reg));
                    } else if (dst == 5) {
                        set_index(uint8_t(*index_reg >> 8), value);
                    } else {
                        this->*regs[dst] = value;
                    }
                }
            } else if (opcode >= 0x80 && opcode <= 0xbf) {  // alu with index regs
                uint8_t src = uint8_t(opcode & 7);
                uint8_t value;
                if (src == 6) {
                    value = rd(ea());
                } else if (src == 4) {
                    value = index_high;
                } else if (src == 5) {
                    value = index_low;
                } else {
                    value = this->*regs[src];
                }
                switch ((opcode >> 3) & 7) {
                    case 0: add_a(value); break;
                    case 1: adc_a(value); break;
                    case 2: sub_a(value); break;
                    case 3: sbc_a(value); break;
                    case 4: and_a(value); break;
                    case 5: xor_a(value); break;
                    case 6: or_a(value); break;
                    default: cp_a(value); break;
                }
            } else {
                // Any other opcode behaves as if the prefix was not there.
                pc_ = uint16_t(pc_ - 1);
            }
            break;
    }
}

void Z80::exec_index_cb(uint16_t* index_reg) {
    int8_t offset = int8_t(fetch());
    uint8_t opcode = fetch();
    cycles_ += kIndexCb[opcode];

    uint16_t addr = uint16_t(*index_reg + offset);
    wz = addr;
    uint8_t value = rd(addr);
    uint8_t op = uint8_t(opcode >> 6);
    uint8_t bit_index = uint8_t((opcode >> 3) & 7);

    if (op == 1) {
        bit(bit_index, value, uint8_t(addr >> 8));
        return;
    }

    uint8_t result;
    if (op == 0) {
        switch (bit_index) {
            case 0: result = rlc(value); break;
            case 1: result = rrc(value); break;
            case 2: result = rl(value); break;
            case 3: result = rr(value); break;
            case 4: result = sla(value); break;
            case 5: result = sra(value); break;
            case 6: result = sll(value); break;
            default: result = srl(value); break;
        }
    } else if (op == 2) {
        result = uint8_t(value & ~(1 << bit_index));
    } else {
        result = uint8_t(value | (1 << bit_index));
    }
    wr(addr, result);

    // Undocumented: the result is also copied into the encoded register.
    static uint8_t Z80::*const regs[8] = {&Z80::b, &Z80::c, &Z80::d, &Z80::e,
                                          &Z80::h, &Z80::l, nullptr, &Z80::a};
    uint8_t index = uint8_t(opcode & 7);
    if (index != 6) this->*regs[index] = result;
}

}  // namespace dsp
