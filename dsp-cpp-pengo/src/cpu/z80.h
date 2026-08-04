#pragma once

#include <cstdint>
#include <functional>
#include <utility>

#include "cpu/irq_line.h"

namespace dsp {

class Z80 {
public:
    using ReadHandler = std::function<uint8_t(uint16_t)>;
    using WriteHandler = std::function<void(uint16_t, uint8_t)>;
    using InHandler = std::function<uint8_t(uint16_t)>;
    using OutHandler = std::function<void(uint16_t, uint8_t)>;
    using CycleHandler = std::function<void(int)>;

    explicit Z80(uint32_t clock);

    void set_memory_handlers(ReadHandler read, WriteHandler write);
    void set_io_handlers(InHandler in, OutHandler out);
    // Called after every instruction with the number of elapsed T states.
    void set_cycle_handler(CycleHandler handler) { cycle_handler_ = std::move(handler); }

    void reset();
    // Runs until at least `cycles` T states have elapsed, returns the amount executed.
    int run(int cycles);

    void set_irq(IrqLine state, uint8_t vector = 0xff);
    void set_nmi(IrqLine state);

    uint32_t clock() const { return clock_; }
    uint16_t pc() const { return pc_; }
    void set_pc(uint16_t value) { pc_ = value; }

    // True while the core is fetching an opcode/prefix byte (M1), mirroring
    // nz80.pas `opcode`. Encrypted games such as Pengo use this to pick the
    // opcode ROM bank versus the data ROM bank.
    bool opcode_fetch() const { return opcode_fetch_; }

    // Registers, public to keep debugging and driver hooks simple.
    uint8_t a = 0, f = 0, b = 0, c = 0, d = 0, e = 0, h = 0, l = 0;
    uint8_t a2 = 0, f2 = 0, b2 = 0, c2 = 0, d2 = 0, e2 = 0, h2 = 0, l2 = 0;
    uint16_t ix = 0, iy = 0, sp = 0, wz = 0;
    uint8_t i = 0, r = 0, im = 0;
    bool iff1 = false, iff2 = false, halted = false;

    static constexpr uint8_t CF = 0x01;
    static constexpr uint8_t NF = 0x02;
    static constexpr uint8_t PF = 0x04;
    static constexpr uint8_t XF = 0x08;
    static constexpr uint8_t HF = 0x10;
    static constexpr uint8_t YF = 0x20;
    static constexpr uint8_t ZF = 0x40;
    static constexpr uint8_t SF = 0x80;

private:
    uint8_t rd(uint16_t addr) const { return read_(addr); }
    void wr(uint16_t addr, uint8_t value) { write_(addr, value); }
    uint8_t fetch();
    uint16_t fetch16();
    void push(uint16_t value);
    uint16_t pop();

    uint16_t bc() const { return uint16_t((b << 8) | c); }
    uint16_t de() const { return uint16_t((d << 8) | e); }
    uint16_t hl() const { return uint16_t((h << 8) | l); }
    void set_bc(uint16_t v) { b = uint8_t(v >> 8); c = uint8_t(v); }
    void set_de(uint16_t v) { d = uint8_t(v >> 8); e = uint8_t(v); }
    void set_hl(uint16_t v) { h = uint8_t(v >> 8); l = uint8_t(v); }
    uint16_t af() const { return uint16_t((a << 8) | f); }
    void set_af(uint16_t v) { a = uint8_t(v >> 8); f = uint8_t(v); }

    // ALU helpers.
    void add_a(uint8_t value);
    void adc_a(uint8_t value);
    void sub_a(uint8_t value);
    void sbc_a(uint8_t value);
    void and_a(uint8_t value);
    void xor_a(uint8_t value);
    void or_a(uint8_t value);
    void cp_a(uint8_t value);
    uint8_t inc8(uint8_t value);
    uint8_t dec8(uint8_t value);
    uint16_t add16(uint16_t x, uint16_t y);
    void adc_hl(uint16_t value);
    void sbc_hl(uint16_t value);
    void daa();
    void rrd();
    void rld();

    uint8_t rlc(uint8_t value);
    uint8_t rrc(uint8_t value);
    uint8_t rl(uint8_t value);
    uint8_t rr(uint8_t value);
    uint8_t sla(uint8_t value);
    uint8_t sra(uint8_t value);
    uint8_t sll(uint8_t value);
    uint8_t srl(uint8_t value);
    void bit(uint8_t index, uint8_t value, uint8_t xy_source);

    void block_ld(int delta, bool repeat);
    void block_cp(int delta, bool repeat);
    void block_in(int delta, bool repeat);
    void block_out(int delta, bool repeat);

    void exec_cb();
    void exec_ed();
    void exec_index(uint16_t* index_reg);
    void exec_index_cb(uint16_t* index_reg);

    int take_nmi();
    int take_irq();

    uint32_t clock_;
    ReadHandler read_;
    WriteHandler write_;
    InHandler in_;
    OutHandler out_;
    CycleHandler cycle_handler_;

    int cycles_ = 0;      // T states consumed by the instruction being executed
    int executed_ = 0;    // T states consumed in the current run() call
    bool after_ei_ = false;
    bool opcode_fetch_ = false;
    IrqLine irq_state_ = IrqLine::Clear;
    IrqLine nmi_state_ = IrqLine::Clear;
    // A level asserted NMI is edge triggered: it fires once and stays latched
    // until the line is released.
    bool nmi_latched_ = false;
    uint8_t irq_vector_ = 0xff;
    uint16_t pc_ = 0;
};

}  // namespace dsp
