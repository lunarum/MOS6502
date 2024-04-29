const memory = @import("memory.zig");

const byte = memory.byte;
const word = memory.word;

pub const MEM_STACK_PAGE: byte = 0x01;
pub const MEM_ZERO_PAGE: byte = 0x00;
pub const MEM_NMI: word = 0xFFFA;
pub const MEM_RESET: word = 0xFFFC;
pub const MEM_IRQ_BREAK: word = 0xFFFE;
pub const COUNTER_VIC: byte = 0;
pub const COUNTER_VIA1: byte = 1;
pub const COUNTER_VIA2: byte = 2;
pub const MAX_COUNTERS: byte = 3;

pub fn getAddress(lowByte: byte, highByte: byte) word {
    return @as(word, highByte) << 8 | @as(word, lowByte);
}

pub const CpuResult = enum {
    RESULT_STEP,
    RESULT_NMI,
    RESULT_RESET,
    RESULT_IRQ,
    RESULT_ILLEGAL_INSTUCTION,
};

pub const CpuFlags = packed struct(byte) {
    PS_C: bool = false, // Carry
    PS_Z: bool = false, // Zero
    PS_I: bool = false, // IRQ disable
    PS_D: bool = false, // Decimal mode
    PS_B: bool = false, // (in) Break command
    PS_0: bool = false,
    PS_V: bool = false, // oVerflow
    PS_N: bool = false, // Negative
};

pub export const CPU = struct {
    A: byte = 0, // Accumulator
    X: byte = 0, // X index register
    Y: byte = 0, // Y idex register
    SP: byte = 0, // Stack Pointer
    PC: word = 0, // Program Counter
    PS: CpuFlags, // Processor Status

    instruction_cycles: byte = 0, // #cycles of last executed instruction
    cycles: i128 = 0, // instruction cycles spent after start (overflows)
    single_step: bool = false, // single step modus

    memoryMap: memory.Memory = memory.Memory{},

    /// Immediate Addressing
    /// --------------------
    /// The value (8-bits) given is a number to be used immediately by the instruction.
    /// For example, LDA #$99 loads the value $99 into the accumulator.
    pub fn memoryReadImmediate(self: *CPU) byte {
        const value = self.memoryMap.read(self.PC);
        self.PC +%= 1;
        return value;
    }

    /// Absolute Addressing
    /// -------------------
    /// The address (16-bits) given is on the absolute memory location after the instruction.
    /// For example, JMP $3E32 loads the address on memory location $3E32 into the PC (jumps to).
    fn memoryReadAbsoluteAddress(self: *CPU) word {
        return getAddress(self.memoryReadImmediate(), self.memoryReadImmediate());
    }

    /// Absolute Addressing
    /// -------------------
    /// The value given is the address (8-bits) on the absolute memory location after the instruction.
    /// For example, STA $3E32 stores the present value of the accumulator in memory location $3E32.
    pub fn memoryReadAbsolute(self: *CPU) byte {
        return self.memoryMap.read(getAddress(memoryReadImmediate(), self.memoryReadImmediate()));
    }

    /// Absolute Addressing
    /// -------------------
    /// The given value is stored on the absolute memory location after the instruction.
    /// For example, STA $3E32 stores the present value of the accumulator in memory location $3E32.
    pub fn memoryWriteAbsolute(self: *CPU, value: byte) void {
        self.memoryMap.write(getAddress(memoryReadImmediate(), self.memoryReadImmediate()), value);
    }

    pub fn run(self: *CPU) CpuResult {
        var tmp_PS_C: bool = undefined;
        var opcode: byte = undefined;
        var value: byte = undefined;
        var value_w: word = undefined;
        var value_w2: word = undefined;

        while (true) {
            opcode = self.memoryReadImmediate;
            switch (opcode) {
                0x00 => { // BRK
                    self.instruction_cycles = 7;
                    memoryStackPushAddress(&self.SP, self.PC +% 1);
                    self.PS_B = true;
                    memoryStackPush(&self.SP, self.PS);
                    self.PS_I = true;
                    self.PC = memoryGetVector(MEM_IRQ_BREAK);
                    break;
                },
                0x01 => { // ORA (aa,X)
                    self.instruction_cycles = 6;
                    self.A |= memoryGetIndexedIndirectX();
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x02: KIL
                // Illegal opcode 0x03: SLO (aa,X)
                // Illegal opcode 0x04: NOP aa
                0x05 => { // ORA aa
                    self.instruction_cycles = 3;
                    value = memoryGetZeroPage();
                    self.A |= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x06 => { // ASL aa
                    self.instruction_cycles = 5;
                    value = memoryGetZeroPage();
                    self.PS_C = ((value & 0x80) != 0);
                    value <<= 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memorySetLast(value);
                    break;
                },
                // Illegal opcode 0x07: SLO aa
                0x08 => { // PHP
                    self.instruction_cycles = 3;
                    memoryStackPush(&self.SP, self.PS);
                    break;
                },
                0x09 => { // ORA #aa
                    self.instruction_cycles = 2;
                    value = self.memoryReadImmediate;
                    self.A |= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x0A => { // ASL
                    self.instruction_cycles = 2;
                    self.PS_C = ((self.A & 0x80) != 0);
                    self.A <<= 1;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x0B: ANC #aa
                // Illegal opcode 0x0C: NOP aaaa
                0x0D => { // ORA aaaa
                    self.instruction_cycles = 4;
                    value = self.memoryReadAbsolute();
                    self.A |= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x0E => { // ASL aaaa
                    self.instruction_cycles = 6;
                    value = self.memoryReadAbsolute();
                    self.PS_C = ((value & 0x80) != 0);
                    value <<= 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x0F: SLO aaaa
                0x10 => { // BPL aaaa
                    self.instruction_cycles = 2;
                    value_w = memoryGetRelativeAddress();
                    if (!self.PS_N) {
                        self.instruction_cycles +%= 1;
                        self.PC = value_w;
                    }
                    break;
                },
                0x11 => { // ORA (aa),Y
                    self.instruction_cycles = 5;
                    value = memoryGetIndirectIndexedY();
                    self.A |= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x12: KIL
                // Illegal opcode 0x13: SLO (aa),Y
                // Illegal opcode 0x14: NOP aa,X
                0x15 => { // ORA aa,X
                    self.instruction_cycles = 4;
                    value = memoryGetZeroPageIndexedX();
                    self.A |= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x16 => { // ASL aa,X
                    self.instruction_cycles = 6;
                    value = memoryGetZeroPageIndexedX();
                    self.PS_C = ((value & 0x80) != 0);
                    value <<= 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x17: SLO aa,X
                0x18 => { // CLC
                    self.instruction_cycles = 2;
                    self.PS_C = false;
                    break;
                },
                0x19 => { // ORA aaaa,Y
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedY();
                    self.A |= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x1A: NOP
                // Illegal opcode 0x1B: SLO aaaa,Y
                // Illegal opcode 0x1C: NOP aaaa,X
                0x1D => { // ORA aaaa,X
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedX();
                    self.A |= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x1E => { // ASL aaaa,X
                    self.instruction_cycles = 7;
                    value = memoryGetAbsoluteIndexedX();
                    self.PS_C = ((value & 0x80) != 0);
                    value <<= 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x1F: SLO aaaa,X
                0x20 => { // JSR aaaa
                    self.instruction_cycles = 6;
                    value_w = self.memoryReadAbsoluteAddress();
                    memoryStackPushAddress(&self.SP, self.PC -% 1);
                    self.PC = value_w;
                    break;
                },
                0x21 => { // AND (aa,X)
                    self.instruction_cycles = 6;
                    value = memoryGetIndexedIndirectX();
                    self.A &= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x22: KIL
                // Illegal opcode 0x23: RLA (aa,X)
                0x24 => { // BIT aa
                    self.instruction_cycles = 3;
                    value = memoryGetZeroPage();
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_V = ((value & 0x40) != 0);
                    self.PS_Z = ((value & self.A) != 0);
                    break;
                },
                0x25 => { // AND aa
                    self.instruction_cycles = 3;
                    value = memoryGetZeroPage();
                    self.A &= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x26 => { // ROL aa
                    self.instruction_cycles = 5;
                    value = memoryGetZeroPage();
                    tmp_PS_C = self.PS_C;
                    self.PS_C = ((value & 0x80) != 0);
                    value = (value << 1) | tmp_PS_C;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x27: RLA aa
                0x28 => { // PLP
                    self.instruction_cycles = 4;
                    self.PS = memoryStackPull(&self.SP);
                    break;
                },
                0x29 => { // AND #aa
                    self.instruction_cycles = 2;
                    value = self.memoryReadImmediate;
                    self.A &= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x2A => { // ROL
                    self.instruction_cycles = 2;
                    tmp_PS_C = self.PS_C;
                    self.PS_C = ((self.A & 0x80) != 0);
                    self.A = (value << 1) | tmp_PS_C;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x2B: ANC #aa
                0x2C => { // BIT aaaa
                    self.instruction_cycles = 4;
                    value = self.memoryReadAbsolute();
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_V = ((value & 0x40) != 0);
                    self.PS_Z = ((value & self.A) != 0);
                    break;
                },
                0x2D => { // AND aaaa
                    self.instruction_cycles = 4;
                    value = self.memoryReadAbsolute();
                    self.A &= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x2E => { // ROL aaaa
                    self.instruction_cycles = 6;
                    value = self.memoryReadAbsolute();
                    tmp_PS_C = self.PS_C;
                    self.PS_C = ((value & 0x80) != 0);
                    value = (value << 1) | tmp_PS_C;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x2F: RLA aaaa
                0x30 => { // BMI aaaa
                    self.instruction_cycles = 2;
                    value_w = memoryGetRelativeAddress();
                    if (self.PS_N) {
                        self.instruction_cycles +%= 1;
                        self.PC = value_w;
                    }
                    break;
                },
                0x31 => { // AND (aa),Y
                    self.instruction_cycles = 5;
                    value = memoryGetIndirectIndexedY();
                    self.A &= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x32: KIL
                // Illegal opcode 0x33: RLA (aa),Y
                // Illegal opcode 0x34: NOP aa,X
                0x35 => { // AND aa,X
                    self.instruction_cycles = 4;
                    value = memoryGetZeroPageIndexedX();
                    self.A &= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x36 => { // ROL aa,X
                    self.instruction_cycles = 6;
                    value = memoryGetZeroPageIndexedX();
                    tmp_PS_C = self.PS_C;
                    self.PS_C = ((value & 0x80) != 0);
                    value = (value << 1) | tmp_PS_C;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x37: RLA aa,X
                0x38 => { // SEC
                    self.instruction_cycles = 2;
                    self.PS_C = true;
                    break;
                },
                0x39 => { // AND aaaa,Y
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedY();
                    self.A &= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x3A: NOP
                // Illegal opcode 0x3B: RLA aaaa,Y
                // Illegal opcode 0x3C: NOP aaaa,X
                0x3D => { // AND aaaa,X
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedX();
                    self.A &= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x3E => { // ROL aaaa,X
                    self.instruction_cycles = 7;
                    value = memoryGetAbsoluteIndexedX();
                    tmp_PS_C = self.PS_C;
                    self.PS_C = ((value & 0x80) != 0);
                    value = (value << 1) | tmp_PS_C;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x3F: RLA aaaa,X
                0x40 => { // RTI
                    self.instruction_cycles = 6;
                    self.PS = memoryStackPull(&self.SP);
                    self.PC = memoryStackPullAddress(&self.SP);
                    break;
                },
                0x41 => { // EOR (aa,X)
                    self.instruction_cycles = 6;
                    value = memoryGetIndexedIndirectX();
                    self.A ^= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x42: KIL
                // Illegal opcode 0x43: SRE (aa,X)
                // Illegal opcode 0x44: NOP aa
                0x45 => { // EOR aa
                    self.instruction_cycles = 3;
                    value = memoryGetZeroPage();
                    self.A ^= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x46 => { // LSR aa
                    self.instruction_cycles = 5;
                    value = memoryGetZeroPage();
                    self.PS_C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x47: SRE aa
                0x48 => { // PHA
                    self.instruction_cycles = 3;
                    memoryStackPush(&self.SP, self.A);
                    break;
                },
                0x49 => { // EOR #aa
                    self.instruction_cycles = 2;
                    value = self.memoryReadImmediate;
                    self.A ^= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x4A => { // LSR
                    self.instruction_cycles = 2;
                    self.PS_C = ((self.A & 0x01) != 0);
                    self.A >>= 1;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x4B: ALR #aa
                0x4C => { // JMP aaaa
                    self.instruction_cycles = 3;
                    self.PC = self.memoryReadAbsoluteAddress();
                    break;
                },
                0x4D => { // EOR aaaa
                    self.instruction_cycles = 4;
                    value = self.memoryReadAbsolute();
                    self.A ^= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x4E => { // LSR aaaa
                    self.instruction_cycles = 6;
                    value = self.memoryReadAbsolute();
                    self.PS_C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x4F: SRE aaaa
                0x50 => { // BVC aaaa
                    self.instruction_cycles = 2;
                    value_w = memoryGetRelativeAddress();
                    if (!self.PS_V) {
                        self.instruction_cycles +%= 1;
                        self.PC = value_w;
                    }
                    break;
                },
                0x51 => { // EOR (aa),Y
                    self.instruction_cycles = 5;
                    value = memoryGetIndirectIndexedY();
                    self.A ^= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x52: KIL
                // Illegal opcode 0x53: SRE (aa),Y
                // Illegal opcode 0x54: NOP aa,X
                0x55 => { // EOR aa,X
                    self.instruction_cycles = 4;
                    value = memoryGetZeroPageIndexedX();
                    self.A ^= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x56 => { // LSR aa,X
                    self.instruction_cycles = 6;
                    value = memoryGetZeroPageIndexedX();
                    self.PS_C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x57: SRE aa,X
                0x58 => { // CLI
                    self.instruction_cycles = 2;
                    self.PS_I = false;
                    break;
                },
                0x59 => { // EOR aaaa,Y
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedY();
                    self.A ^= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x5A: NOP
                // Illegal opcode 0x5B: SRE aaaa,Y
                // Illegal opcode 0x5C: NOP aaaa,X
                0x5D => { // EOR aaaa,X
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedX();
                    self.A ^= value;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x5E => { // LSR aaaa,X
                    self.instruction_cycles = 7;
                    value = memoryGetAbsoluteIndexedX();
                    self.PS_C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x5F: SRE aaaa,X
                0x60 => { // RTS
                    self.instruction_cycles = 6;
                    self.PC = memoryStackPullAddress(&self.SP) +% 1;
                    break;
                },
                0x61 => { // ADC (aa,X)
                    self.instruction_cycles = 6;
                    value = memoryGetIndexedIndirectX();
                    if (self.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% self.PS_C;
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% self.PS_C;
                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_Z = (self.A == 0);
                        self.PS_C = (value_w >= 0x100);
                    }

                    break;
                },
                // Illegal opcode 0x62: KIL
                // Illegal opcode 0x63: RRA (aa,X)
                // Illegal opcode 0x64: NOP aa
                0x65 => { // ADC aa
                    self.instruction_cycles = 3;
                    value = memoryGetZeroPage();
                    if (self.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% self.PS_C;
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% self.PS_C;
                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_Z = (self.A == 0);
                        self.PS_C = (value_w >= 0x100);
                    }

                    break;
                },
                0x66 => { // ROR aa
                    self.instruction_cycles = 5;
                    value = memoryGetZeroPage();
                    tmp_PS_C = self.PS_C;
                    self.PS_C = ((value & 0x01) != 0);
                    value = (value >> 1) | (tmp_PS_C << 7);
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x67: RRA aa
                0x68 => { // PLA
                    self.instruction_cycles = 4;
                    self.A = memoryStackPull(&self.SP);
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x69 => { // ADC #aa
                    self.instruction_cycles = 2;
                    value = self.memoryReadImmediate;
                    if (self.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% self.PS_C;
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% self.PS_C;
                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_Z = (self.A == 0);
                        self.PS_C = (value_w >= 0x100);
                    }

                    break;
                },
                0x6A => { // ROR
                    self.instruction_cycles = 2;
                    tmp_PS_C = self.PS_C;
                    self.PS_C = ((self.A & 0x01) != 0);
                    self.A = (value >> 1) | (tmp_PS_C << 7);
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x6B: ARR #aa
                0x6C => { // JMP (aaaa)
                    self.instruction_cycles = 5;
                    self.PC = memoryGetIndirectAbsoluteAddress();
                    break;
                },
                0x6D => { // ADC aaaa
                    self.instruction_cycles = 4;
                    value = self.memoryReadAbsolute();
                    if (self.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% self.PS_C;
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% self.PS_C;
                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_Z = (self.A == 0);
                        self.PS_C = (value_w >= 0x100);
                    }

                    break;
                },
                0x6E => { // ROR aaaa
                    self.instruction_cycles = 6;
                    value = self.memoryReadAbsolute();
                    tmp_PS_C = self.PS_C;
                    self.PS_C = ((value & 0x01) != 0);
                    value = (value >> 1) | (tmp_PS_C << 7);
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x6F: RRA aaaa
                0x70 => { // BVS aaaa
                    self.instruction_cycles = 2;
                    value_w = memoryGetRelativeAddress();
                    if (self.PS_V) {
                        self.instruction_cycles +%= 1;
                        self.PC = value_w;
                    }
                    break;
                },
                0x71 => { // ADC (aa),Y
                    self.instruction_cycles = 5;
                    value = memoryGetIndirectIndexedY();
                    if (self.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% self.PS_C;
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% self.PS_C;
                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_Z = (self.A == 0);
                        self.PS_C = (value_w >= 0x100);
                    }

                    break;
                },
                // Illegal opcode 0x72: KIL
                // Illegal opcode 0x73: RRA (aa),Y
                // Illegal opcode 0x74: NOP aa,X
                0x75 => { // ADC aa,X
                    self.instruction_cycles = 4;
                    value = memoryGetZeroPageIndexedX();
                    if (self.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% self.PS_C;
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% self.PS_C;
                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_Z = (self.A == 0);
                        self.PS_C = (value_w >= 0x100);
                    }

                    break;
                },
                0x76 => { // ROR aa,X
                    self.instruction_cycles = 6;
                    value = memoryGetZeroPageIndexedX();
                    tmp_PS_C = self.PS_C;
                    self.PS_C = ((value & 0x01) != 0);
                    value = (value >> 1) | (tmp_PS_C << 7);
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x77: RRA aa,X
                0x78 => { // SEI
                    self.instruction_cycles = 2;
                    self.PS_I = true;
                    break;
                },
                0x79 => { // ADC aaaa,Y
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedY();
                    if (self.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% self.PS_C;
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% self.PS_C;
                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_Z = (self.A == 0);
                        self.PS_C = (value_w >= 0x100);
                    }

                    break;
                },
                // Illegal opcode 0x7A: NOP
                // Illegal opcode 0x7B: RRA aaaa,Y
                // Illegal opcode 0x7C: NOP aaaa,X
                0x7D => { // ADC aaaa,X
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedX();
                    if (self.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% self.PS_C;
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% self.PS_C;
                        self.PS_N = ((value_w & 0x80) != 0);
                        self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                        self.A = (byte)(value_w & 0xFF);
                        self.PS_Z = (self.A == 0);
                        self.PS_C = (value_w >= 0x100);
                    }

                    break;
                },
                0x7E => { // ROR aaaa,X
                    self.instruction_cycles = 7;
                    value = memoryGetAbsoluteIndexedX();
                    tmp_PS_C = self.PS_C;
                    self.PS_C = ((value & 0x01) != 0);
                    value = (value >> 1) | (tmp_PS_C << 7);
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0x7F: RRA aaaa,X
                // Illegal opcode 0x80: NOP #aa
                0x81 => { // STA (aa,X)
                    self.instruction_cycles = 6;
                    memory_setIndexedIndirectX(self.A);
                    break;
                },
                // Illegal opcode 0x82: NOP #aa
                // Illegal opcode 0x83: SAX (aa,X)
                0x84 => { // STY aa
                    self.instruction_cycles = 3;
                    memory_setZeroPage(self.Y);
                    break;
                },
                0x85 => { // STA aa
                    self.instruction_cycles = 3;
                    memory_setZeroPage(self.A);
                    break;
                },
                0x86 => { // STX aa
                    self.instruction_cycles = 3;
                    memory_setZeroPage(self.X);
                    break;
                },
                // Illegal opcode 0x87: SAX aa
                0x88 => { // DEY
                    self.instruction_cycles = 2;
                    -%-%self.Y;
                    self.PS_N = ((self.Y & 0x80) != 0);
                    self.PS_Z = (self.Y == 0);
                    break;
                },
                // Illegal opcode 0x89: NOP #aa
                0x8A => { // TXA
                    self.instruction_cycles = 2;
                    self.A = self.X;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0x8B: XAA #aa
                0x8C => { // STY aaaa
                    self.instruction_cycles = 4;
                    self.memoryWriteAbsolute(self.Y);
                    break;
                },
                0x8D => { // STA aaaa
                    self.instruction_cycles = 4;
                    self.memoryWriteAbsolute(self.A);
                    break;
                },
                0x8E => { // STX aaaa
                    self.instruction_cycles = 4;
                    self.memoryWriteAbsolute(self.X);
                    break;
                },
                // Illegal opcode 0x8F: SAX aaaa
                0x90 => { // BCC aaaa
                    self.instruction_cycles = 2;
                    value_w = memoryGetRelativeAddress();
                    if (!self.PS_C) {
                        self.instruction_cycles +%= 1;
                        self.PC = value_w;
                    }
                    break;
                },
                0x91 => { // STA (aa),Y
                    self.instruction_cycles = 6;
                    memory_setIndirectIndexedY(self.A);
                    break;
                },
                // Illegal opcode 0x92: KIL
                // Illegal opcode 0x93: AHX (aa),Y
                0x94 => { // STY aa,X
                    self.instruction_cycles = 4;
                    memory_setZeroPageIndexedX(self.Y);
                    break;
                },
                0x95 => { // STA aa,X
                    self.instruction_cycles = 4;
                    memory_setZeroPageIndexedX(self.A);
                    break;
                },
                0x96 => { // STX aa,Y
                    self.instruction_cycles = 4;
                    memory_setZeroPageIndexedY(self.X);
                    break;
                },
                // Illegal opcode 0x97: SAX aa,Y
                0x98 => { // TYA
                    self.instruction_cycles = 2;
                    self.A = self.Y;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0x99 => { // STA aaaa,Y
                    self.instruction_cycles = 5;
                    memory_setAbsoluteIndexedY(self.A);
                    break;
                },
                0x9A => { // TXS
                    self.instruction_cycles = 2;
                    self.SP = self.X;
                    break;
                },
                // Illegal opcode 0x9B: TAS aaaa,Y
                // Illegal opcode 0x9C: SHY aaaa,X
                0x9D => { // STA aaaa,X
                    self.instruction_cycles = 5;
                    memory_setAbsoluteIndexedX(self.A);
                    break;
                },
                // Illegal opcode 0x9E: SHX aaaa,Y
                // Illegal opcode 0x9F: AHX aaaa,Y
                0xA0 => { // LDY #aa
                    self.instruction_cycles = 2;
                    self.Y = self.memoryReadImmediate;
                    self.PS_N = ((self.Y & 0x80) != 0);
                    self.PS_Z = (self.Y == 0);
                    break;
                },
                0xA1 => { // LDA (aa,X)
                    self.instruction_cycles = 6;
                    self.A = memoryGetIndexedIndirectX();
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0xA2 => { // LDX #aa
                    self.instruction_cycles = 2;
                    self.X = self.memoryReadImmediate;
                    self.PS_N = ((self.X & 0x80) != 0);
                    self.PS_Z = (self.X == 0);
                    break;
                },
                // Illegal opcode 0xA3: LAX (aa,X)
                0xA4 => { // LDY aa
                    self.instruction_cycles = 3;
                    self.Y = memoryGetZeroPage();
                    self.PS_N = ((self.Y & 0x80) != 0);
                    self.PS_Z = (self.Y == 0);
                    break;
                },
                0xA5 => { // LDA aa
                    self.instruction_cycles = 3;
                    self.A = memoryGetZeroPage();
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0xA6 => { // LDX aa
                    self.instruction_cycles = 3;
                    self.X = memoryGetZeroPage();
                    self.PS_N = ((self.X & 0x80) != 0);
                    self.PS_Z = (self.X == 0);
                    break;
                },
                // Illegal opcode 0xA7: LAX aa
                0xA8 => { // TAY
                    self.instruction_cycles = 2;
                    self.Y = self.A;
                    self.PS_N = ((self.Y & 0x80) != 0);
                    self.PS_Z = (self.Y == 0);
                    break;
                },
                0xA9 => { // LDA #aa
                    self.instruction_cycles = 2;
                    self.A = self.memoryReadImmediate;
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0xAA => { // TAX
                    self.instruction_cycles = 2;
                    self.X = self.A;
                    self.PS_N = ((self.X & 0x80) != 0);
                    self.PS_Z = (self.X == 0);
                    break;
                },
                // Illegal opcode 0xAB: LAX #aa
                0xAC => { // LDY aaaa
                    self.instruction_cycles = 4;
                    self.Y = self.memoryReadAbsolute();
                    self.PS_N = ((self.Y & 0x80) != 0);
                    self.PS_Z = (self.Y == 0);
                    break;
                },
                0xAD => { // LDA aaaa
                    self.instruction_cycles = 4;
                    self.A = self.memoryReadAbsolute();
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0xAE => { // LDX aaaa
                    self.instruction_cycles = 4;
                    self.X = self.memoryReadAbsolute();
                    self.PS_N = ((self.X & 0x80) != 0);
                    self.PS_Z = (self.X == 0);
                    break;
                },
                // Illegal opcode 0xAF: LAX aaaa
                0xB0 => { // BCS aaaa
                    self.instruction_cycles = 2;
                    value_w = memoryGetRelativeAddress();
                    if (self.PS_C) {
                        self.instruction_cycles +%= 1;
                        self.PC = value_w;
                    }
                    break;
                },
                0xB1 => { // LDA (aa),Y
                    self.instruction_cycles = 5;
                    self.A = memoryGetIndirectIndexedY();
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                // Illegal opcode 0xB2: KIL
                // Illegal opcode 0xB3: LAX (aa),Y
                0xB4 => { // LDY aa,X
                    self.instruction_cycles = 4;
                    self.Y = memoryGetZeroPageIndexedX();
                    self.PS_N = ((self.Y & 0x80) != 0);
                    self.PS_Z = (self.Y == 0);
                    break;
                },
                0xB5 => { // LDA aa,X
                    self.instruction_cycles = 4;
                    self.A = memoryGetZeroPageIndexedX();
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0xB6 => { // LDX aa,Y
                    self.instruction_cycles = 4;
                    self.X = memoryGetZeroPageIndexedY();
                    self.PS_N = ((self.X & 0x80) != 0);
                    self.PS_Z = (self.X == 0);
                    break;
                },
                // Illegal opcode 0xB7: LAX aa,Y
                0xB8 => { // CLV
                    self.instruction_cycles = 2;
                    self.PS_V = false;
                    break;
                },
                0xB9 => { // LDA aaaa,Y
                    self.instruction_cycles = 4;
                    self.A = memoryGetAbsoluteIndexedY();
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0xBA => { // TSX
                    self.instruction_cycles = 2;
                    self.X = self.SP;
                    self.PS_N = ((self.X & 0x80) != 0);
                    self.PS_Z = (self.X == 0);
                    break;
                },
                // Illegal opcode 0xBB: LAS aaaa,Y
                0xBC => { // LDY aaaa,X
                    self.instruction_cycles = 4;
                    self.Y = memoryGetAbsoluteIndexedX();
                    self.PS_N = ((self.Y & 0x80) != 0);
                    self.PS_Z = (self.Y == 0);
                    break;
                },
                0xBD => { // LDA aaaa,X
                    self.instruction_cycles = 4;
                    self.A = memoryGetAbsoluteIndexedX();
                    self.PS_N = ((self.A & 0x80) != 0);
                    self.PS_Z = (self.A == 0);
                    break;
                },
                0xBE => { // LDX aaaa,Y
                    self.instruction_cycles = 4;
                    self.X = memoryGetAbsoluteIndexedY();
                    self.PS_N = ((self.X & 0x80) != 0);
                    self.PS_Z = (self.X == 0);
                    break;
                },
                // Illegal opcode 0xBF: LAX aaaa,Y
                0xC0 => { // CPY #aa
                    self.instruction_cycles = 2;
                    value = self.memoryReadImmediate;
                    value = self.Y -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xC1 => { // CMP (aa,X)
                    self.instruction_cycles = 6;
                    value = memoryGetIndexedIndirectX();
                    value = self.A -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                // Illegal opcode 0xC2: NOP #aa
                // Illegal opcode 0xC3: DCP (aa,X)
                0xC4 => { // CPY aa
                    self.instruction_cycles = 3;
                    value = memoryGetZeroPage();
                    value = self.Y -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xC5 => { // CMP aa
                    self.instruction_cycles = 3;
                    value = memoryGetZeroPage();
                    value = self.A -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xC6 => { // DEC aa
                    self.instruction_cycles = 5;
                    value = memoryGetZeroPage();
                    -%-%value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0xC7: DCP aa
                0xC8 => { // INY
                    self.instruction_cycles = 2;
                    self.Y +%= 1;
                    self.PS_N = ((self.Y & 0x80) != 0);
                    self.PS_Z = (self.Y == 0);
                    break;
                },
                0xC9 => { // CMP #aa
                    self.instruction_cycles = 2;
                    value = self.memoryReadImmediate;
                    value = self.A -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xCA => { // DEX
                    self.instruction_cycles = 2;
                    -%-%self.X;
                    self.PS_N = ((self.X & 0x80) != 0);
                    self.PS_Z = (self.X == 0);
                    break;
                },
                // Illegal opcode 0xCB: AXS #aa
                0xCC => { // CPY aaaa
                    self.instruction_cycles = 4;
                    value = self.memoryReadAbsolute();
                    value = self.Y -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xCD => { // CMP aaaa
                    self.instruction_cycles = 4;
                    value = self.memoryReadAbsolute();
                    value = self.A -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xCE => { // DEC aaaa
                    self.instruction_cycles = 6;
                    value = self.memoryReadAbsolute();
                    -%-%value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0xCF: DCP aaaa
                0xD0 => { // BNE aaaa
                    self.instruction_cycles = 2;
                    value_w = memoryGetRelativeAddress();
                    if (!self.PS_Z) {
                        self.instruction_cycles +%= 1;
                        self.PC = value_w;
                    }
                    break;
                },
                0xD1 => { // CMP (aa),Y
                    self.instruction_cycles = 5;
                    value = memoryGetIndirectIndexedY();
                    value = self.A -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                // Illegal opcode 0xD2: KIL
                // Illegal opcode 0xD3: DCP (aa),Y
                // Illegal opcode 0xD4: NOP aa,X
                0xD5 => { // CMP aa,X
                    self.instruction_cycles = 4;
                    value = memoryGetZeroPageIndexedX();
                    value = self.A -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xD6 => { // DEC aa,X
                    self.instruction_cycles = 6;
                    value = memoryGetZeroPageIndexedX();
                    -%-%value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0xD7: DCP aa,X
                0xD8 => { // CLD
                    self.instruction_cycles = 2;
                    self.PS_D = false;
                    break;
                },
                0xD9 => { // CMP aaaa,Y
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedY();
                    value = self.A -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                // Illegal opcode 0xDA: NOP
                // Illegal opcode 0xDB: DCP aaaa,Y
                // Illegal opcode 0xDC: NOP aaaa,X
                0xDD => { // CMP aaaa,X
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedX();
                    value = self.A -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xDE => { // DEC aaaa,X
                    self.instruction_cycles = 7;
                    value = memoryGetAbsoluteIndexedX();
                    -%-%value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0xDF: DCP aaaa,X
                0xE0 => { // CPX #aa
                    self.instruction_cycles = 2;
                    value = self.memoryReadImmediate;
                    value = self.X -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xE1 => { // SBC (aa,X)
                    self.instruction_cycles = 6;
                    value = memoryGetIndexedIndirectX();
                    value_w = self.A -% value -% self.PS_C;
                    self.PS_N = ((value_w & 0x80) != 0);
                    self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                    self.PS_Z = (value_w == 0);
                    self.PS_C = (value_w < 0x100);

                    if (self.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% self.PS_C;
                        if (value_w2 & 0x8000)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000)
                            value_w -%= 0x60;
                        self.A = (byte)(value_w2 & 0xFF);
                    } else self.A = (byte)(value_w & 0xFF);

                    break;
                },
                // Illegal opcode 0xE2: NOP #aa
                // Illegal opcode 0xE3: ISC (aa,X)
                0xE4 => { // CPX aa
                    self.instruction_cycles = 3;
                    value = memoryGetZeroPage();
                    value = self.X -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xE5 => { // SBC aa
                    self.instruction_cycles = 3;
                    value = memoryGetZeroPage();
                    value_w = self.A -% value -% self.PS_C;
                    self.PS_N = ((value_w & 0x80) != 0);
                    self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                    self.PS_Z = (value_w == 0);
                    self.PS_C = (value_w < 0x100);

                    if (self.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% self.PS_C;
                        if (value_w2 & 0x8000)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000)
                            value_w -%= 0x60;
                        self.A = (byte)(value_w2 & 0xFF);
                    } else self.A = (byte)(value_w & 0xFF);

                    break;
                },
                0xE6 => { // INC aa
                    self.instruction_cycles = 5;
                    value = memoryGetZeroPage() +% 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0xE7: ISC aa
                0xE8 => { // INX
                    self.instruction_cycles = 2;
                    self.X +%= 1;
                    self.PS_N = ((self.X & 0x80) != 0);
                    self.PS_Z = (self.X == 0);
                    break;
                },
                0xE9 => { // SBC #aa
                    self.instruction_cycles = 2;
                    value = self.memoryReadImmediate;
                    value_w = self.A -% value -% self.PS_C;
                    self.PS_N = ((value_w & 0x80) != 0);
                    self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                    self.PS_Z = (value_w == 0);
                    self.PS_C = (value_w < 0x100);

                    if (self.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% self.PS_C;
                        if (value_w2 & 0x8000)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000)
                            value_w -%= 0x60;
                        self.A = (byte)(value_w2 & 0xFF);
                    } else self.A = (byte)(value_w & 0xFF);

                    break;
                },
                0xEA => { // NOP
                    self.instruction_cycles = 2;
                    // do nothing
                    break;
                },
                // Illegal opcode 0xEB: SBC #aa
                0xEC => { // CPX aaaa
                    self.instruction_cycles = 4;
                    value = self.memoryReadAbsolute();
                    value = self.X -% value;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    self.PS_C = !self.PS_N;
                    break;
                },
                0xED => { // SBC aaaa
                    self.instruction_cycles = 4;
                    value = self.memoryReadAbsolute();
                    value_w = self.A -% value -% self.PS_C;
                    self.PS_N = ((value_w & 0x80) != 0);
                    self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                    self.PS_Z = (value_w == 0);
                    self.PS_C = (value_w < 0x100);

                    if (self.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% self.PS_C;
                        if (value_w2 & 0x8000)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000)
                            value_w -%= 0x60;
                        self.A = (byte)(value_w2 & 0xFF);
                    } else self.A = (byte)(value_w & 0xFF);

                    break;
                },
                0xEE => { // INC aaaa
                    self.instruction_cycles = 6;
                    value = self.memoryReadAbsolute() +% 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0xEF: ISC aaaa
                0xF0 => { // BEQ aaaa
                    self.instruction_cycles = 2;
                    value_w = memoryGetRelativeAddress();
                    if (self.PS_Z) {
                        self.instruction_cycles +%= 1;
                        self.PC = value_w;
                    }
                    break;
                },
                0xF1 => { // SBC (aa),Y
                    self.instruction_cycles = 5;
                    value = memoryGetIndirectIndexedY();
                    value_w = self.A -% value -% self.PS_C;
                    self.PS_N = ((value_w & 0x80) != 0);
                    self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                    self.PS_Z = (value_w == 0);
                    self.PS_C = (value_w < 0x100);

                    if (self.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% self.PS_C;
                        if (value_w2 & 0x8000)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000)
                            value_w -%= 0x60;
                        self.A = (byte)(value_w2 & 0xFF);
                    } else self.A = (byte)(value_w & 0xFF);

                    break;
                },
                // Illegal opcode 0xF2: KIL
                // Illegal opcode 0xF3: ISC (aa),Y
                // Illegal opcode 0xF4: NOP aa,X
                0xF5 => { // SBC aa,X
                    self.instruction_cycles = 4;
                    value = memoryGetZeroPageIndexedX();
                    value_w = self.A -% value -% self.PS_C;
                    self.PS_N = ((value_w & 0x80) != 0);
                    self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                    self.PS_Z = (value_w == 0);
                    self.PS_C = (value_w < 0x100);

                    if (self.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% self.PS_C;
                        if (value_w2 & 0x8000)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000)
                            value_w -%= 0x60;
                        self.A = (byte)(value_w2 & 0xFF);
                    } else self.A = (byte)(value_w & 0xFF);

                    break;
                },
                0xF6 => { // INC aa,X
                    self.instruction_cycles = 6;
                    value = memoryGetZeroPageIndexedX() +% 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0xF7: ISC aa,X
                0xF8 => { // SED
                    self.instruction_cycles = 2;
                    self.PS_D = true;
                    break;
                },
                0xF9 => { // SBC aaaa,Y
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedY();
                    value_w = self.A -% value -% self.PS_C;
                    self.PS_N = ((value_w & 0x80) != 0);
                    self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                    self.PS_Z = (value_w == 0);
                    self.PS_C = (value_w < 0x100);

                    if (self.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% self.PS_C;
                        if (value_w2 & 0x8000)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000)
                            value_w -%= 0x60;
                        self.A = (byte)(value_w2 & 0xFF);
                    } else self.A = (byte)(value_w & 0xFF);

                    break;
                },
                // Illegal opcode 0xFA: NOP
                // Illegal opcode 0xFB: ISC aaaa,Y
                // Illegal opcode 0xFC: NOP aaaa,X
                0xFD => { // SBC aaaa,X
                    self.instruction_cycles = 4;
                    value = memoryGetAbsoluteIndexedX();
                    value_w = self.A -% value -% self.PS_C;
                    self.PS_N = ((value_w & 0x80) != 0);
                    self.PS_V = ((self.A ^ value_w) & 0x80) > 0 and !((self.A ^ value) & 0x80) > 0;
                    self.PS_Z = (value_w == 0);
                    self.PS_C = (value_w < 0x100);

                    if (self.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% self.PS_C;
                        if (value_w2 & 0x8000)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000)
                            value_w -%= 0x60;
                        self.A = (byte)(value_w2 & 0xFF);
                    } else self.A = (byte)(value_w & 0xFF);

                    break;
                },
                0xFE => { // INC aaaa,X
                    self.instruction_cycles = 7;
                    value = memoryGetAbsoluteIndexedX() +% 1;
                    self.PS_N = ((value & 0x80) != 0);
                    self.PS_Z = (value == 0);
                    memory_setLast(value);
                    break;
                },
                // Illegal opcode 0xFF: ISC aaaa,X
                else => {
                    self.instruction_cycles = 1;
                    return @as(c_uint, @bitCast(RESULT_ILLEGAL_INSTUCTION));
                },
            }

            // if(via_count_NMI(self.instruction_cycles)) {
            //     memoryStackPushAddress(&self.SPself.PC+%1);
            //     memoryStackPush(&self.SP, cpu_get_state());
            //     self.PS_I = true;
            //     self.PC = memoryGetVector(MEM_NMI);
            //     self.instruction_cycles +%= 7;
            //     via_count_NMI(7);
            //     break;
            // }
            // if(via_count_IRQ(self.instruction_cycles)) {
            //     memoryStackPushAddress(&self.SPself.PC+%1);
            //     self.PS_B = false;
            //     memoryStackPush(&self.SP, cpu_get_state());
            //     self.PS_I = true;
            //     self.PC = memoryGetVector(MEM_IRQ_BREAK);
            //     self.instruction_cycles +%= 7;
            //     via_count_NMI(7);
            //     via_count_IRQ(7);
            // }

            // value_w = 0;
            // while (value_w < 3) : (value_w +%= 1) {
            //     signal_func = counter[value_w].signal;
            //     if (signal_func != null) {
            //         count_tmp = counter[value_w].counter;
            //         counter[value_w].counter -%= @as(c_uint, @bitCast(@as(c_uint, self.instruction_cycles)));
            //         if (counter[value_w].counter > count_tmp) {
            //             signal_func.?();
            //         }
            //     }
            // }

            if (single_step != 0) return RESULT_STEP;
        }
        return RESULT_ILLEGAL_INSTUCTION;
    }
};
