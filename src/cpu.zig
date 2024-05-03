const std = @import("std");
const Memory = @import("memory.zig");

const byte = Memory.byte;
const word = Memory.word;

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

pub const RunResult = enum {
    RESULT_STEP,
    RESULT_NMI,
    RESULT_RESET,
    RESULT_IRQ,
    RESULT_ILLEGAL_INSTUCTION,
};

/// Processor Status flags: Carry, Zero, Irq disable, Decimal mode, Break mode, oVerflow and Negative
pub const Flags = packed struct(byte) {
    PS_C: bool = false, // Carry
    PS_Z: bool = false, // Zero
    PS_I: bool = false, // IRQ disable
    PS_D: bool = false, // Decimal mode
    PS_B: bool = false, // (in) Break command
    PS_0: bool = false,
    PS_V: bool = false, // oVerflow
    PS_N: bool = false, // Negative
};

/// The 6502 processor with registers Accumulator, X and Y index registers, Stack Pointer, Program Counter and Processor Status register
/// Also contains status like the last instruction cycles (last_cycles), total executed cycles (cycles) and if in single step modus (single_step).
/// Finally the memory Manager (memoryManager) is part of this cpu structure, which includes the full memory map.
///
/// All access modes are implemented in separate (private) functions, as is the function to run / continue execution.
/// When newly initialised, the cpu starts from the normal address which is stored at MEM_RESET, else it continues from current PC address.
pub const CPU6502 = struct {
    A: byte = 0, // Accumulator
    X: byte = 0, // X index register
    Y: byte = 0, // Y idex register
    SP: byte = 0, // Stack Pointer
    PC: word = MEM_RESET, // Program Counter
    PS: Flags = Flags{}, // Processor Status

    last_cycles: byte = 0, // #cycles of last executed instruction
    cycles: i128 = 0, // instruction cycles spent after start (overflows)
    single_step: bool = false, // single step modus

    memoryManager: *Memory.Memory = undefined,

    pub fn init(self: *CPU6502, memoryManager: *Memory.Memory) void {
        self.memoryManager = memoryManager;
        self.reset();
    }

    pub fn reset(self: *CPU6502) void {
        self.PS.PS_I = true;
        self.PS.PS_D = false;
        self.SP = 0xFF;
        self.PC = self.memoryReadAddress(MEM_RESET);
        self.last_cycles = 0;
        self.cycles = 0;
        self.single_step = false;
    }

    /// memoryReadAddress
    /// ----------------
    /// Read an address (word)) from given memory address, which is stored in little endian (LSB first) format.
    fn memoryReadAddress(self: *CPU6502, address: word) word {
        const lowByte: word = self.memoryManager.read(address);
        const highByte: word = self.memoryManager.read(address + 1);
        return highByte << 8 | lowByte;
    }

    /// stackPush
    /// ---------
    /// Push a value (byte) onto the stack (page 1) at location pointed to by SP, which grows downwards.
    fn stackPush(self: *CPU6502, value: byte) void {
        self.memoryManager.writeStack(self.SP, value);
        self.SP -%= 1;
    }

    /// stackPull
    /// ---------
    /// Pull a value (byte) from the stack (page 1) at location pointed to by SP, which shrinks upwards.
    fn stackPull(self: *CPU6502) byte {
        self.SP +%= 1;
        return self.memoryManager.readStack(self.SP);
    }

    /// stackPushAddress
    /// ----------------
    /// Push an address (word) onto the stack (page 1) at location pointed to by SP, MSB first, but because the stack grows downwards, resulting in a little endian stored address.
    fn stackPushAddress(self: *CPU6502, address: word) void {
        self.memoryManager.writeStack(self.SP, @truncate(address >> 8));
        self.SP -%= 1;
        self.memoryManager.writeStack(self.SP, @truncate(address));
        self.SP -%= 1;
    }

    /// stackPullAddress
    /// ----------------
    /// Pull an address (word) from the stack (page 1) at location pointed to by SP, LSB first, because the stack shrinks upwards.
    fn stackPullAddress(self: *CPU6502) word {
        self.SP +%= 1;
        const lowByte: word = self.memoryManager.readStack(self.SP);
        self.SP +%= 1;
        const highByte: word = self.memoryManager.readStack(self.SP);
        return highByte << 8 | lowByte;
    }

    /// memoryReadImmediate
    /// -------------------
    /// The byte value read is the byte at the memory location pointed to by the PC.
    ///
    /// For example, LDA #$1A loads the value $1A into the accumulator.
    pub fn memoryReadImmediate(self: *CPU6502) byte {
        const value = self.memoryManager.read(self.PC);
        self.PC +%= 1;
        return value;
    }

    /// memoryReadAbsolute
    /// ------------------
    /// The byte value read is the byte stored at the memory location pointed to by the PC.
    ///
    /// For example, LDA $1234 reads the value in memory location $1234 and stores it into the accumulator.
    pub fn memoryReadAbsolute(self: *CPU6502) byte {
        return self.memoryManager.read(getAddress(self.memoryReadImmediate(), self.memoryReadImmediate()));
    }

    /// memoryReadAbsoluteAddress
    /// -------------------------
    /// The address (word) read is the address stored at the memory location pointed to by the PC.
    ///
    /// For example, JMP $1A2B loads the address $1A2B into the PC (jumps to).
    fn memoryReadAbsoluteAddress(self: *CPU6502) word {
        return getAddress(self.memoryReadImmediate(), self.memoryReadImmediate());
    }

    /// memoryWriteAbsolute
    /// -------------------
    /// The byte value which is written at the memory location location pointed to by the PC.
    ///
    /// For example, STA $1A2B stores the present value of the accumulator in memory location $1A2B.
    pub fn memoryWriteAbsolute(self: *CPU6502, value: byte) void {
        self.memoryManager.write(getAddress(self.memoryReadImmediate(), self.memoryReadImmediate()), value);
    }

    /// memoryReadZeroPage
    /// ------------------
    /// The byte value read is the byte stored at the zero page memory location (the LSB byte value, because the MSB is assumed $00) pointed to by the PC.
    ///
    /// For example, LDA $34 reads the value in memory location $0034 and stores it into the accumulator.
    fn memoryReadZeroPage(self: *CPU6502) byte {
        return self.memoryManager.readZero(self.memoryReadImmediate());
    }

    /// memoryWriteZeroPage
    /// -------------------
    /// The byte value written is stored at the zero page memory location (the LSB byte value, because the MSB is assumed $00) pointed to by the PC.
    ///
    /// For example, STA $34 stores the present value of the accumulator in memory location $0034.
    fn memoryWriteZeroPage(self: *CPU6502, value: byte) void {
        self.memoryManager.writeZero(self.memoryReadImmediate(), value);
    }

    /// memoryReadAbsoluteIndexedX
    /// --------------------------
    /// The byte value read is the byte stored at the memory location pointed to by the PC with the X register added.
    ///
    /// For example, LDA $1234, X with the X register containing $06 reads the value in memory location $123A and stores it into the accumulator.
    fn memoryReadAbsoluteIndexedX(self: *CPU6502) byte {
        const lowByte = self.memoryReadImmediate();
        const highByte = self.memoryReadImmediate();
        const address = getAddress(lowByte, highByte) +% self.X;
        if (highByte != address >> 8) // crossing a page boundary adds an extra cycle
            self.last_cycles += 1;
        return self.memoryManager.read(address);
    }

    /// memoryReadAbsoluteIndexedY
    /// --------------------------
    /// The byte value read is the byte stored at the memory location pointed to by the PC with the Y register added.
    ///
    /// For example, LDA $1234, Y with the Y register containing $06 reads the value in memory location $123A and stores it into the accumulator.
    fn memoryReadAbsoluteIndexedY(self: *CPU6502) byte {
        const lowByte = self.memoryReadImmediate();
        const highByte = self.memoryReadImmediate();
        const address = getAddress(lowByte, highByte) +% self.Y;
        if (highByte != address >> 8) // crossing a page boundary adds an extra cycle
            self.last_cycles += 1;
        return self.memoryManager.read(address);
    }

    /// memoryWriteAbsoluteIndexedX
    /// --------------------------
    /// The byte value written is stored at the memory location pointed to by the PC with the X register added.
    ///
    /// For example, STA $1234, X with the X register containing $06 writes the accumulator to memory location $123A.
    fn memoryWriteAbsoluteIndexedX(self: *CPU6502, value: byte) void {
        const lowByte = self.memoryReadImmediate();
        const highByte = self.memoryReadImmediate();
        const address = getAddress(lowByte, highByte) +% self.X;
        if (highByte != address >> 8) // crossing a page boundary adds an extra cycle
            self.last_cycles += 1;
        self.memoryManager.write(address, value);
    }

    /// memoryWriteAbsoluteIndexedY
    /// --------------------------
    /// The byte value written is stored at the memory location pointed to by the PC with the Y register added.
    ///
    /// For example, STA $1234, Y with the Y register containing $06 writes the accumulator to memory location $123A.
    fn memoryWriteAbsoluteIndexedY(self: *CPU6502, value: byte) void {
        const lowByte = self.memoryReadImmediate();
        const highByte = self.memoryReadImmediate();
        const address = getAddress(lowByte, highByte) +% self.X;
        if (highByte != address >> 8) // crossing a page boundary adds an extra cycle
            self.last_cycles += 1;
        self.memoryManager.write(address, value);
    }

    /// memoryReadIndirectAbsoluteAddress
    /// ---------------------------------
    /// Reads the address (word) pointed to by the PC and then uses that as a pointer in memory to the returned address (again in little endian format).
    ///
    /// Due to a bug in the 6502, when the low (first) byte in memory of the address to be returned is 0xFF, the next (high) byte is read still is
    /// from adress 0x00, but on the same page e.g. when the address is $04FF is used (as the addres of the low address byte) the high byte is read from
    /// $0400 instead of the expected $0500!
    ///
    /// Only JMP uses this addressing mode, so e.g. JMP ($1234), with memory $1234 containing $CD and $1235 containing $AB, jumps to $ABCD.
    fn memoryReadIndirectAbsoluteAddress(self: *CPU6502) word {
        const address = getAddress(self.memoryReadImmediate(), self.memoryReadImmediate());
        const lowByte = self.memoryManager.read(address);
        // 6502 bug: fetching the full address doesn't cross a page boundary but wraps around on the same page
        const highByte = self.memoryManager.read(if (lowByte == 0xFF) address & 0xFF00 else address + 1);
        return getAddress(lowByte, highByte);
    }

    /// memoryReadZeroPageIndexedX
    /// --------------------------
    /// The byte value read is read from the zero page memory location (the LSB byte value, because the MSB is assumed $00) pointed to by the PC
    /// with index register X added with overflow (i.e. without leaving the zero page).
    ///
    /// For example, LDA $34, X, when X contains $F0 reads the byte value in memory location $0024 ($34 + $F0 = $0124 => $0024) into the accumulator.
    fn memoryReadZeroPageIndexedX(self: *CPU6502) byte {
        return self.memoryManager.readZero(self.memoryReadImmediate() +% self.X);
    }

    /// memoryReadZeroPageIndexedY
    /// --------------------------
    /// The byte value read is read from the zero page memory location (the LSB byte value, because the MSB is assumed $00) pointed to by the PC
    /// with index register Y added with overflow (i.e. without leaving the zero page).
    ///
    /// For example, LDA $34, Y, when Y contains $F0 reads the byte value in memory location $0024 ($34 + $F0 = $0124 => $0024) into the accumulator.
    fn memoryReadZeroPageIndexedY(self: *CPU6502) byte {
        return self.memoryManager.readZero(self.memoryReadImmediate() +% self.Y);
    }

    /// memoryWriteZeroPageIndexedX
    /// ---------------------------
    /// The byte value written is stored at the zero page memory location (the LSB byte value, because the MSB is assumed $00) pointed to by the PC
    /// with index register X added with overflow (i.e. without leaving the zero page).
    ///
    /// For example, STA $34, X, when X contains $F0 stores the present value of the accumulator in memory location $0024 ($34 + $F0 = $0124 => $0024).
    fn memoryWriteZeroPageIndexedX(self: *CPU6502, value: byte) void {
        self.memoryManager.writeZero(self.memoryReadImmediate() +% self.X, value);
    }

    /// memoryWriteZeroPageIndexedY
    /// ---------------------------
    /// The byte value written is stored at the zero page memory location (the LSB byte value, because the MSB is assumed $00) pointed to by the PC
    /// with index register Y added with overflow (i.e. without leaving the zero page).
    ///
    /// For example, STA $34, Y, when Y contains $F0 stores the present value of the accumulator in memory location $0024 ($34 + $F0 = $0124 => $0024).
    fn memoryWriteZeroPageIndexedY(self: *CPU6502, value: byte) void {
        self.memoryManager.writeZero(self.memoryReadImmediate() +% self.Y, value);
    }

    /// memoryReadIndexedIndirectX
    /// --------------------------
    /// Reads the address (word) on the zero page, (address byte) pointed to by the PC and add register X to that (wrapping around), in little endian format.
    ///
    /// For example, LDA ($34,X), where X contains 6 and memory on $3A containing $CD and on $3B containing $AB, results in loading the accumulator with the byte value at $ABCD.
    fn memoryReadIndexedIndirectX(self: *CPU6502) byte {
        const addressZ: byte = self.memoryReadImmediate() +% self.X;
        const address = getAddress(self.memoryManager.readZero(addressZ), self.memoryManager.readZero(addressZ +% 1));
        return self.memoryManager.read(address);
    }

    /// memoryWriteIndexedIndirectX
    /// --------------------------
    /// Writes the value (byte) to the address (word) found on the zero page at (address byte) pointed to by the PC and with register X added to that (wrapping around), in little endian format.
    ///
    /// Only used by STA, so with STA ($34,X), where X contains 6 and memory on $3A containing $CD and on $3B containing $AB, results in writing the accumulator byte value to $ABCD.
    fn memoryWriteIndexedIndirectX(self: *CPU6502, value: byte) void {
        const addressZ: byte = self.memoryReadImmediate() +% self.X;
        const address = getAddress(self.memoryManager.readZero(addressZ), self.memoryManager.readZero(addressZ +% 1));
        self.memoryManager.write(address, value);
    }

    /// memoryReadIndirectIndexedY
    /// --------------------------
    /// Reads the address (word) on the zero page, (address byte) pointed to by the PC in little endian format with the register Y added to that found address (wrapping around).
    ///
    /// For example, LDA ($34),Y, where Y contains 6 and memory on $34 containing $C7 and on $35 containing $AB, results in loading the accumulator with the byte value at $ABCD.
    fn memoryReadIndirectIndexedY(self: *CPU6502) byte {
        const addressZ: byte = self.memoryReadImmediate();
        const address = getAddress(self.memoryManager.readZero(addressZ), self.memoryManager.readZero(addressZ +% 1)) +% self.Y;
        return self.memoryManager.read(address);
    }

    /// memoryWriteIndirectIndexedY
    /// --------------------------
    /// Writes the value (byte) to the address (word) found on the zero page at (address byte) pointed to by the PC in little endian format with the register Y added to that found address (wrapping around).
    ///
    /// Only used by STA, so with STA ($34),Y, where Y contains 6 and memory on $34 containing $C7 and on $35 containing $AB, results in writing the accumulator byte value to $ABCD.
    fn memoryWriteIndirectIndexedY(self: *CPU6502, value: byte) void {
        const addressZ: byte = self.memoryReadImmediate();
        const address = getAddress(self.memoryManager.readZero(addressZ), self.memoryManager.readZero(addressZ +% 1)) +% self.Y;
        self.memoryManager.write(address, value);
    }

    /// memoryReadRelativeAddress
    /// -------------------------
    /// Branching instructions use relative addressing i.e. adding an offset (a byte), pointed to by the PC to that PC.
    /// The offset is interpreted as 2's complement, so if bit 7 is set it is used as a negative offset. To calculate that offset, negate all bits and then add 1.
    /// The resulting address is returned; PC is NOT set.
    ///
    /// For example, BEQ $0F will jump (branch) 15 bytes ahead if the Z-flag is set, while BEQ $F0 will jump 16 bytes back if the Z-flag is set.
    fn memoryReadRelativeAddress(self: *CPU6502) word {
        const offset = self.memoryReadImmediate();
        // check for a two's complement negative offset
        const address = if (offset & 0x80 == 0x80) self.PC -% (~offset + 1) else self.PC +% offset;
        if ((address & 0xFF00) != (self.PC & 0xFF00)) // crossing the page boundary adds an extra cycle
            self.cycles += 1;
        return address;
    }

    pub fn run(self: *CPU6502) RunResult {
        var tmp_PS_C: bool = undefined;
        var opcode: byte = undefined;
        var value: byte = undefined;
        var value_w: word = undefined;
        var value_w2: word = undefined;

        while (true) {
            opcode = self.memoryReadImmediate();
            switch (opcode) {
                0x00 => { // BRK
                    self.last_cycles = 7;
                    self.stackPushAddress(self.PC +% 1);
                    self.PS.PS_B = true;
                    self.stackPush(@bitCast(self.PS));
                    self.PS.PS_I = true;
                    self.PC = self.memoryReadAddress(MEM_IRQ_BREAK);
                },
                0x01 => { // ORA (aa,X)
                    self.last_cycles = 6;
                    self.A |= self.memoryReadIndexedIndirectX();
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x02: KIL
                // Illegal opcode 0x03: SLO (aa,X)
                // Illegal opcode 0x04: NOP aa
                0x05 => { // ORA aa
                    self.last_cycles = 3;
                    value = self.memoryReadZeroPage();
                    self.A |= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x06 => { // ASL aa
                    self.last_cycles = 5;
                    value = self.memoryReadZeroPage();
                    self.PS.PS_C = ((value & 0x80) != 0);
                    value <<= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x07: SLO aa
                0x08 => { // PHP
                    self.last_cycles = 3;
                    self.stackPush(@bitCast(self.PS));
                },
                0x09 => { // ORA #aa
                    self.last_cycles = 2;
                    value = self.memoryReadImmediate();
                    self.A |= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x0A => { // ASL
                    self.last_cycles = 2;
                    self.PS.PS_C = ((self.A & 0x80) != 0);
                    self.A <<= 1;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x0B: ANC #aa
                // Illegal opcode 0x0C: NOP aaaa
                0x0D => { // ORA aaaa
                    self.last_cycles = 4;
                    value = self.memoryReadAbsolute();
                    self.A |= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x0E => { // ASL aaaa
                    self.last_cycles = 6;
                    value = self.memoryReadAbsolute();
                    self.PS.PS_C = ((value & 0x80) != 0);
                    value <<= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x0F: SLO aaaa
                0x10 => { // BPL aaaa
                    self.last_cycles = 2;
                    value_w = self.memoryReadRelativeAddress();
                    if (!self.PS.PS_N) {
                        self.last_cycles +%= 1;
                        self.PC = value_w;
                    }
                },
                0x11 => { // ORA (aa),Y
                    self.last_cycles = 5;
                    value = self.memoryReadIndirectIndexedY();
                    self.A |= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x12: KIL
                // Illegal opcode 0x13: SLO (aa),Y
                // Illegal opcode 0x14: NOP aa,X
                0x15 => { // ORA aa,X
                    self.last_cycles = 4;
                    value = self.memoryReadZeroPageIndexedX();
                    self.A |= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x16 => { // ASL aa,X
                    self.last_cycles = 6;
                    value = self.memoryReadZeroPageIndexedX();
                    self.PS.PS_C = ((value & 0x80) != 0);
                    value <<= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x17: SLO aa,X
                0x18 => { // CLC
                    self.last_cycles = 2;
                    self.PS.PS_C = false;
                },
                0x19 => { // ORA aaaa,Y
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedY();
                    self.A |= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x1A: NOP
                // Illegal opcode 0x1B: SLO aaaa,Y
                // Illegal opcode 0x1C: NOP aaaa,X
                0x1D => { // ORA aaaa,X
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedX();
                    self.A |= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x1E => { // ASL aaaa,X
                    self.last_cycles = 7;
                    value = self.memoryReadAbsoluteIndexedX();
                    self.PS.PS_C = ((value & 0x80) != 0);
                    value <<= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x1F: SLO aaaa,X
                0x20 => { // JSR aaaa
                    self.last_cycles = 6;
                    value_w = self.memoryReadAbsoluteAddress();
                    self.stackPushAddress(self.PC -% 1);
                    self.PC = value_w;
                },
                0x21 => { // AND (aa,X)
                    self.last_cycles = 6;
                    value = self.memoryReadIndexedIndirectX();
                    self.A &= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x22: KIL
                // Illegal opcode 0x23: RLA (aa,X)
                0x24 => { // BIT aa
                    self.last_cycles = 3;
                    value = self.memoryReadZeroPage();
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_V = ((value & 0x40) != 0);
                    self.PS.PS_Z = ((value & self.A) == 0);
                },
                0x25 => { // AND aa
                    self.last_cycles = 3;
                    value = self.memoryReadZeroPage();
                    self.A &= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x26 => { // ROL aa
                    self.last_cycles = 5;
                    value = self.memoryReadZeroPage();
                    tmp_PS_C = self.PS.PS_C;
                    self.PS.PS_C = ((value & 0x80) != 0);
                    value = (value << 1) | @as(byte, @intFromBool(tmp_PS_C));
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x27: RLA aa
                0x28 => { // PLP
                    self.last_cycles = 4;
                    self.PS = @bitCast(self.stackPull());
                },
                0x29 => { // AND #aa
                    self.last_cycles = 2;
                    value = self.memoryReadImmediate();
                    self.A &= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x2A => { // ROL
                    self.last_cycles = 2;
                    tmp_PS_C = self.PS.PS_C;
                    self.PS.PS_C = ((self.A & 0x80) != 0);
                    self.A = (value << 1) | @as(byte, @intFromBool(tmp_PS_C));
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x2B: ANC #aa
                0x2C => { // BIT aaaa
                    self.last_cycles = 4;
                    value = self.memoryReadAbsolute();
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_V = ((value & 0x40) != 0);
                    self.PS.PS_Z = ((value & self.A) != 0);
                },
                0x2D => { // AND aaaa
                    self.last_cycles = 4;
                    value = self.memoryReadAbsolute();
                    self.A &= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x2E => { // ROL aaaa
                    self.last_cycles = 6;
                    value = self.memoryReadAbsolute();
                    tmp_PS_C = self.PS.PS_C;
                    self.PS.PS_C = ((value & 0x80) != 0);
                    value = (value << 1) | @as(byte, @intFromBool(tmp_PS_C));
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x2F: RLA aaaa
                0x30 => { // BMI aaaa
                    self.last_cycles = 2;
                    value_w = self.memoryReadRelativeAddress();
                    if (self.PS.PS_N) {
                        self.last_cycles +%= 1;
                        self.PC = value_w;
                    }
                },
                0x31 => { // AND (aa),Y
                    self.last_cycles = 5;
                    value = self.memoryReadIndirectIndexedY();
                    self.A &= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x32: KIL
                // Illegal opcode 0x33: RLA (aa),Y
                // Illegal opcode 0x34: NOP aa,X
                0x35 => { // AND aa,X
                    self.last_cycles = 4;
                    value = self.memoryReadZeroPageIndexedX();
                    self.A &= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x36 => { // ROL aa,X
                    self.last_cycles = 6;
                    value = self.memoryReadZeroPageIndexedX();
                    tmp_PS_C = self.PS.PS_C;
                    self.PS.PS_C = ((value & 0x80) != 0);
                    value = (value << 1) | @as(byte, @intFromBool(tmp_PS_C));
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x37: RLA aa,X
                0x38 => { // SEC
                    self.last_cycles = 2;
                    self.PS.PS_C = true;
                },
                0x39 => { // AND aaaa,Y
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedY();
                    self.A &= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x3A: NOP
                // Illegal opcode 0x3B: RLA aaaa,Y
                // Illegal opcode 0x3C: NOP aaaa,X
                0x3D => { // AND aaaa,X
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedX();
                    self.A &= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x3E => { // ROL aaaa,X
                    self.last_cycles = 7;
                    value = self.memoryReadAbsoluteIndexedX();
                    tmp_PS_C = self.PS.PS_C;
                    self.PS.PS_C = ((value & 0x80) != 0);
                    value = (value << 1) | @as(byte, @intFromBool(tmp_PS_C));
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x3F: RLA aaaa,X
                0x40 => { // RTI
                    self.last_cycles = 6;
                    self.PS = @bitCast(self.stackPull());
                    self.PC = self.stackPullAddress();
                },
                0x41 => { // EOR (aa,X)
                    self.last_cycles = 6;
                    value = self.memoryReadIndexedIndirectX();
                    self.A ^= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x42: KIL
                // Illegal opcode 0x43: SRE (aa,X)
                // Illegal opcode 0x44: NOP aa
                0x45 => { // EOR aa
                    self.last_cycles = 3;
                    value = self.memoryReadZeroPage();
                    self.A ^= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x46 => { // LSR aa
                    self.last_cycles = 5;
                    value = self.memoryReadZeroPage();
                    self.PS.PS_C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x47: SRE aa
                0x48 => { // PHA
                    self.last_cycles = 3;
                    self.stackPush(self.A);
                },
                0x49 => { // EOR #aa
                    self.last_cycles = 2;
                    value = self.memoryReadImmediate();
                    self.A ^= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x4A => { // LSR
                    self.last_cycles = 2;
                    self.PS.PS_C = ((self.A & 0x01) != 0);
                    self.A >>= 1;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x4B: ALR #aa
                0x4C => { // JMP aaaa
                    self.last_cycles = 3;
                    self.PC = self.memoryReadAbsoluteAddress();
                },
                0x4D => { // EOR aaaa
                    self.last_cycles = 4;
                    value = self.memoryReadAbsolute();
                    self.A ^= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x4E => { // LSR aaaa
                    self.last_cycles = 6;
                    value = self.memoryReadAbsolute();
                    self.PS.PS_C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x4F: SRE aaaa
                0x50 => { // BVC aaaa
                    self.last_cycles = 2;
                    value_w = self.memoryReadRelativeAddress();
                    if (!self.PS.PS_V) {
                        self.last_cycles +%= 1;
                        self.PC = value_w;
                    }
                },
                0x51 => { // EOR (aa),Y
                    self.last_cycles = 5;
                    value = self.memoryReadIndirectIndexedY();
                    self.A ^= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x52: KIL
                // Illegal opcode 0x53: SRE (aa),Y
                // Illegal opcode 0x54: NOP aa,X
                0x55 => { // EOR aa,X
                    self.last_cycles = 4;
                    value = self.memoryReadZeroPageIndexedX();
                    self.A ^= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x56 => { // LSR aa,X
                    self.last_cycles = 6;
                    value = self.memoryReadZeroPageIndexedX();
                    self.PS.PS_C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x57: SRE aa,X
                0x58 => { // CLI
                    self.last_cycles = 2;
                    self.PS.PS_I = false;
                },
                0x59 => { // EOR aaaa,Y
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedY();
                    self.A ^= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x5A: NOP
                // Illegal opcode 0x5B: SRE aaaa,Y
                // Illegal opcode 0x5C: NOP aaaa,X
                0x5D => { // EOR aaaa,X
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedX();
                    self.A ^= value;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x5E => { // LSR aaaa,X
                    self.last_cycles = 7;
                    value = self.memoryReadAbsoluteIndexedX();
                    self.PS.PS_C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x5F: SRE aaaa,X
                0x60 => { // RTS
                    self.last_cycles = 6;
                    self.PC = self.stackPullAddress() +% 1;
                },
                0x61 => { // ADC (aa,X)
                    self.last_cycles = 6;
                    value = self.memoryReadIndexedIndirectX();
                    if (self.PS.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.PS.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% @as(byte, @intFromBool(self.PS.PS_C));
                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_Z = (self.A == 0);
                        self.PS.PS_C = (value_w >= 0x100);
                    }
                },
                // Illegal opcode 0x62: KIL
                // Illegal opcode 0x63: RRA (aa,X)
                // Illegal opcode 0x64: NOP aa
                0x65 => { // ADC aa
                    self.last_cycles = 3;
                    value = self.memoryReadZeroPage();
                    if (self.PS.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.PS.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% @as(byte, @intFromBool(self.PS.PS_C));
                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_Z = (self.A == 0);
                        self.PS.PS_C = (value_w >= 0x100);
                    }
                },
                0x66 => { // ROR aa
                    self.last_cycles = 5;
                    value = self.memoryReadZeroPage();
                    tmp_PS_C = self.PS.PS_C;
                    self.PS.PS_C = ((value & 0x01) != 0);
                    value = (value >> 1) | (@as(byte, @as(byte, @intFromBool(tmp_PS_C))) << 7);
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x67: RRA aa
                0x68 => { // PLA
                    self.last_cycles = 4;
                    self.A = self.stackPull();
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x69 => { // ADC #aa
                    self.last_cycles = 2;
                    value = self.memoryReadImmediate();
                    if (self.PS.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.PS.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% @as(byte, @intFromBool(self.PS.PS_C));
                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_Z = (self.A == 0);
                        self.PS.PS_C = (value_w >= 0x100);
                    }
                },
                0x6A => { // ROR
                    self.last_cycles = 2;
                    tmp_PS_C = self.PS.PS_C;
                    self.PS.PS_C = ((self.A & 0x01) != 0);
                    self.A = (value >> 1) | (@as(byte, @intFromBool(tmp_PS_C)) << 7);
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x6B: ARR #aa
                0x6C => { // JMP (aaaa)
                    self.last_cycles = 5;
                    self.PC = self.memoryReadIndirectAbsoluteAddress();
                },
                0x6D => { // ADC aaaa
                    self.last_cycles = 4;
                    value = self.memoryReadAbsolute();
                    if (self.PS.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.PS.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% @as(byte, @intFromBool(self.PS.PS_C));
                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_Z = (self.A == 0);
                        self.PS.PS_C = (value_w >= 0x100);
                    }
                },
                0x6E => { // ROR aaaa
                    self.last_cycles = 6;
                    value = self.memoryReadAbsolute();
                    tmp_PS_C = self.PS.PS_C;
                    self.PS.PS_C = ((value & 0x01) != 0);
                    value = (value >> 1) | (@as(byte, @intFromBool(tmp_PS_C)) << 7);
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x6F: RRA aaaa
                0x70 => { // BVS aaaa
                    self.last_cycles = 2;
                    value_w = self.memoryReadRelativeAddress();
                    if (self.PS.PS_V) {
                        self.last_cycles +%= 1;
                        self.PC = value_w;
                    }
                },
                0x71 => { // ADC (aa),Y
                    self.last_cycles = 5;
                    value = self.memoryReadIndirectIndexedY();
                    if (self.PS.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.PS.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% @as(byte, @intFromBool(self.PS.PS_C));
                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_Z = (self.A == 0);
                        self.PS.PS_C = (value_w >= 0x100);
                    }
                },
                // Illegal opcode 0x72: KIL
                // Illegal opcode 0x73: RRA (aa),Y
                // Illegal opcode 0x74: NOP aa,X
                0x75 => { // ADC aa,X
                    self.last_cycles = 4;
                    value = self.memoryReadZeroPageIndexedX();
                    if (self.PS.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.PS.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% @as(byte, @intFromBool(self.PS.PS_C));
                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_Z = (self.A == 0);
                        self.PS.PS_C = (value_w >= 0x100);
                    }
                },
                0x76 => { // ROR aa,X
                    self.last_cycles = 6;
                    value = self.memoryReadZeroPageIndexedX();
                    tmp_PS_C = self.PS.PS_C;
                    self.PS.PS_C = ((value & 0x01) != 0);
                    value = (value >> 1) | (@as(byte, @intFromBool(tmp_PS_C)) << 7);
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x77: RRA aa,X
                0x78 => { // SEI
                    self.last_cycles = 2;
                    self.PS.PS_I = true;
                },
                0x79 => { // ADC aaaa,Y
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedY();
                    if (self.PS.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.PS.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% @as(byte, @intFromBool(self.PS.PS_C));
                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_Z = (self.A == 0);
                        self.PS.PS_C = (value_w >= 0x100);
                    }
                },
                // Illegal opcode 0x7A: NOP
                // Illegal opcode 0x7B: RRA aaaa,Y
                // Illegal opcode 0x7C: NOP aaaa,X
                0x7D => { // ADC aaaa,X
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedX();
                    if (self.PS.PS_D) {
                        value_w = (self.A & 0x0F) +% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w >= 0x0A)
                            value_w = ((value_w +% 0x06) & 0x0F) +% 0x10;
                        value_w +%= (self.A & 0xF0) +% (value & 0xF0);

                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.PS.PS_Z = (value_w == 0);

                        if (value_w >= 0xA0)
                            value_w +%= 0x60;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_C = (value_w >= 0x100);
                    } else {
                        value_w = self.A +% value +% @as(byte, @intFromBool(self.PS.PS_C));
                        self.PS.PS_N = ((value_w & 0x80) != 0);
                        self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                        self.A = @truncate(value_w & 0xFF);
                        self.PS.PS_Z = (self.A == 0);
                        self.PS.PS_C = (value_w >= 0x100);
                    }
                },
                0x7E => { // ROR aaaa,X
                    self.last_cycles = 7;
                    value = self.memoryReadAbsoluteIndexedX();
                    tmp_PS_C = self.PS.PS_C;
                    self.PS.PS_C = ((value & 0x01) != 0);
                    value = (value >> 1) | (@as(byte, @intFromBool(tmp_PS_C)) << 7);
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x7F: RRA aaaa,X
                // Illegal opcode 0x80: NOP #aa
                0x81 => { // STA (aa,X)
                    self.last_cycles = 6;
                    self.memoryWriteIndexedIndirectX(self.A);
                },
                // Illegal opcode 0x82: NOP #aa
                // Illegal opcode 0x83: SAX (aa,X)
                0x84 => { // STY aa
                    self.last_cycles = 3;
                    self.memoryWriteZeroPage(self.Y);
                },
                0x85 => { // STA aa
                    self.last_cycles = 3;
                    self.memoryWriteZeroPage(self.A);
                },
                0x86 => { // STX aa
                    self.last_cycles = 3;
                    self.memoryWriteZeroPage(self.X);
                },
                // Illegal opcode 0x87: SAX aa
                0x88 => { // DEY
                    self.last_cycles = 2;
                    self.Y -%= 1;
                    self.PS.PS_N = ((self.Y & 0x80) != 0);
                    self.PS.PS_Z = (self.Y == 0);
                },
                // Illegal opcode 0x89: NOP #aa
                0x8A => { // TXA
                    self.last_cycles = 2;
                    self.A = self.X;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0x8B: XAA #aa
                0x8C => { // STY aaaa
                    self.last_cycles = 4;
                    self.memoryWriteAbsolute(self.Y);
                },
                0x8D => { // STA aaaa
                    self.last_cycles = 4;
                    self.memoryWriteAbsolute(self.A);
                },
                0x8E => { // STX aaaa
                    self.last_cycles = 4;
                    self.memoryWriteAbsolute(self.X);
                },
                // Illegal opcode 0x8F: SAX aaaa
                0x90 => { // BCC aaaa
                    self.last_cycles = 2;
                    value_w = self.memoryReadRelativeAddress();
                    if (!self.PS.PS_C) {
                        self.last_cycles +%= 1;
                        self.PC = value_w;
                    }
                },
                0x91 => { // STA (aa),Y
                    self.last_cycles = 6;
                    self.memoryWriteIndirectIndexedY(self.A);
                },
                // Illegal opcode 0x92: KIL
                // Illegal opcode 0x93: AHX (aa),Y
                0x94 => { // STY aa,X
                    self.last_cycles = 4;
                    self.memoryWriteZeroPageIndexedX(self.Y);
                },
                0x95 => { // STA aa,X
                    self.last_cycles = 4;
                    self.memoryWriteZeroPageIndexedX(self.A);
                },
                0x96 => { // STX aa,Y
                    self.last_cycles = 4;
                    self.memoryWriteZeroPageIndexedY(self.X);
                },
                // Illegal opcode 0x97: SAX aa,Y
                0x98 => { // TYA
                    self.last_cycles = 2;
                    self.A = self.Y;
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0x99 => { // STA aaaa,Y
                    self.last_cycles = 5;
                    self.memoryWriteAbsoluteIndexedY(self.A);
                },
                0x9A => { // TXS
                    self.last_cycles = 2;
                    self.SP = self.X;
                },
                // Illegal opcode 0x9B: TAS aaaa,Y
                // Illegal opcode 0x9C: SHY aaaa,X
                0x9D => { // STA aaaa,X
                    self.last_cycles = 5;
                    self.memoryWriteAbsoluteIndexedX(self.A);
                },
                // Illegal opcode 0x9E: SHX aaaa,Y
                // Illegal opcode 0x9F: AHX aaaa,Y
                0xA0 => { // LDY #aa
                    self.last_cycles = 2;
                    self.Y = self.memoryReadImmediate();
                    self.PS.PS_N = ((self.Y & 0x80) != 0);
                    self.PS.PS_Z = (self.Y == 0);
                },
                0xA1 => { // LDA (aa,X)
                    self.last_cycles = 6;
                    self.A = self.memoryReadIndexedIndirectX();
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0xA2 => { // LDX #aa
                    self.last_cycles = 2;
                    self.X = self.memoryReadImmediate();
                    self.PS.PS_N = ((self.X & 0x80) != 0);
                    self.PS.PS_Z = (self.X == 0);
                },
                // Illegal opcode 0xA3: LAX (aa,X)
                0xA4 => { // LDY aa
                    self.last_cycles = 3;
                    self.Y = self.memoryReadZeroPage();
                    self.PS.PS_N = ((self.Y & 0x80) != 0);
                    self.PS.PS_Z = (self.Y == 0);
                },
                0xA5 => { // LDA aa
                    self.last_cycles = 3;
                    self.A = self.memoryReadZeroPage();
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0xA6 => { // LDX aa
                    self.last_cycles = 3;
                    self.X = self.memoryReadZeroPage();
                    self.PS.PS_N = ((self.X & 0x80) != 0);
                    self.PS.PS_Z = (self.X == 0);
                },
                // Illegal opcode 0xA7: LAX aa
                0xA8 => { // TAY
                    self.last_cycles = 2;
                    self.Y = self.A;
                    self.PS.PS_N = ((self.Y & 0x80) != 0);
                    self.PS.PS_Z = (self.Y == 0);
                },
                0xA9 => { // LDA #aa
                    self.last_cycles = 2;
                    self.A = self.memoryReadImmediate();
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0xAA => { // TAX
                    self.last_cycles = 2;
                    self.X = self.A;
                    self.PS.PS_N = ((self.X & 0x80) != 0);
                    self.PS.PS_Z = (self.X == 0);
                },
                // Illegal opcode 0xAB: LAX #aa
                0xAC => { // LDY aaaa
                    self.last_cycles = 4;
                    self.Y = self.memoryReadAbsolute();
                    self.PS.PS_N = ((self.Y & 0x80) != 0);
                    self.PS.PS_Z = (self.Y == 0);
                },
                0xAD => { // LDA aaaa
                    self.last_cycles = 4;
                    self.A = self.memoryReadAbsolute();
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0xAE => { // LDX aaaa
                    self.last_cycles = 4;
                    self.X = self.memoryReadAbsolute();
                    self.PS.PS_N = ((self.X & 0x80) != 0);
                    self.PS.PS_Z = (self.X == 0);
                },
                // Illegal opcode 0xAF: LAX aaaa
                0xB0 => { // BCS aaaa
                    self.last_cycles = 2;
                    value_w = self.memoryReadRelativeAddress();
                    if (self.PS.PS_C) {
                        self.last_cycles +%= 1;
                        self.PC = value_w;
                    }
                },
                0xB1 => { // LDA (aa),Y
                    self.last_cycles = 5;
                    self.A = self.memoryReadIndirectIndexedY();
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                // Illegal opcode 0xB2: KIL
                // Illegal opcode 0xB3: LAX (aa),Y
                0xB4 => { // LDY aa,X
                    self.last_cycles = 4;
                    self.Y = self.memoryReadZeroPageIndexedX();
                    self.PS.PS_N = ((self.Y & 0x80) != 0);
                    self.PS.PS_Z = (self.Y == 0);
                },
                0xB5 => { // LDA aa,X
                    self.last_cycles = 4;
                    self.A = self.memoryReadZeroPageIndexedX();
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0xB6 => { // LDX aa,Y
                    self.last_cycles = 4;
                    self.X = self.memoryReadZeroPageIndexedY();
                    self.PS.PS_N = ((self.X & 0x80) != 0);
                    self.PS.PS_Z = (self.X == 0);
                },
                // Illegal opcode 0xB7: LAX aa,Y
                0xB8 => { // CLV
                    self.last_cycles = 2;
                    self.PS.PS_V = false;
                },
                0xB9 => { // LDA aaaa,Y
                    self.last_cycles = 4;
                    self.A = self.memoryReadAbsoluteIndexedY();
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0xBA => { // TSX
                    self.last_cycles = 2;
                    self.X = self.SP;
                    self.PS.PS_N = ((self.X & 0x80) != 0);
                    self.PS.PS_Z = (self.X == 0);
                },
                // Illegal opcode 0xBB: LAS aaaa,Y
                0xBC => { // LDY aaaa,X
                    self.last_cycles = 4;
                    self.Y = self.memoryReadAbsoluteIndexedX();
                    self.PS.PS_N = ((self.Y & 0x80) != 0);
                    self.PS.PS_Z = (self.Y == 0);
                },
                0xBD => { // LDA aaaa,X
                    self.last_cycles = 4;
                    self.A = self.memoryReadAbsoluteIndexedX();
                    self.PS.PS_N = ((self.A & 0x80) != 0);
                    self.PS.PS_Z = (self.A == 0);
                },
                0xBE => { // LDX aaaa,Y
                    self.last_cycles = 4;
                    self.X = self.memoryReadAbsoluteIndexedY();
                    self.PS.PS_N = ((self.X & 0x80) != 0);
                    self.PS.PS_Z = (self.X == 0);
                },
                // Illegal opcode 0xBF: LAX aaaa,Y
                0xC0 => { // CPY #aa
                    self.last_cycles = 2;
                    value = self.memoryReadImmediate();
                    value = self.Y -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xC1 => { // CMP (aa,X)
                    self.last_cycles = 6;
                    value = self.memoryReadIndexedIndirectX();
                    value = self.A -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                // Illegal opcode 0xC2: NOP #aa
                // Illegal opcode 0xC3: DCP (aa,X)
                0xC4 => { // CPY aa
                    self.last_cycles = 3;
                    value = self.memoryReadZeroPage();
                    value = self.Y -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xC5 => { // CMP aa
                    self.last_cycles = 3;
                    value = self.memoryReadZeroPage();
                    value = self.A -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xC6 => { // DEC aa
                    self.last_cycles = 5;
                    value = self.memoryReadZeroPage();
                    value -%= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xC7: DCP aa
                0xC8 => { // INY
                    self.last_cycles = 2;
                    self.Y +%= 1;
                    self.PS.PS_N = ((self.Y & 0x80) != 0);
                    self.PS.PS_Z = (self.Y == 0);
                },
                0xC9 => { // CMP #aa
                    self.last_cycles = 2;
                    value = self.memoryReadImmediate();
                    value = self.A -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xCA => { // DEX
                    self.last_cycles = 2;
                    self.X -%= 1;
                    self.PS.PS_N = ((self.X & 0x80) != 0);
                    self.PS.PS_Z = (self.X == 0);
                },
                // Illegal opcode 0xCB: AXS #aa
                0xCC => { // CPY aaaa
                    self.last_cycles = 4;
                    value = self.memoryReadAbsolute();
                    value = self.Y -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xCD => { // CMP aaaa
                    self.last_cycles = 4;
                    value = self.memoryReadAbsolute();
                    value = self.A -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xCE => { // DEC aaaa
                    self.last_cycles = 6;
                    value = self.memoryReadAbsolute();
                    value -%= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xCF: DCP aaaa
                0xD0 => { // BNE aaaa
                    self.last_cycles = 2;
                    value_w = self.memoryReadRelativeAddress();
                    if (!self.PS.PS_Z) {
                        self.last_cycles +%= 1;
                        self.PC = value_w;
                    }
                },
                0xD1 => { // CMP (aa),Y
                    self.last_cycles = 5;
                    value = self.memoryReadIndirectIndexedY();
                    value = self.A -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                // Illegal opcode 0xD2: KIL
                // Illegal opcode 0xD3: DCP (aa),Y
                // Illegal opcode 0xD4: NOP aa,X
                0xD5 => { // CMP aa,X
                    self.last_cycles = 4;
                    value = self.memoryReadZeroPageIndexedX();
                    value = self.A -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xD6 => { // DEC aa,X
                    self.last_cycles = 6;
                    value = self.memoryReadZeroPageIndexedX();
                    value -%= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xD7: DCP aa,X
                0xD8 => { // CLD
                    self.last_cycles = 2;
                    self.PS.PS_D = false;
                },
                0xD9 => { // CMP aaaa,Y
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedY();
                    value = self.A -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                // Illegal opcode 0xDA: NOP
                // Illegal opcode 0xDB: DCP aaaa,Y
                // Illegal opcode 0xDC: NOP aaaa,X
                0xDD => { // CMP aaaa,X
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedX();
                    value = self.A -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xDE => { // DEC aaaa,X
                    self.last_cycles = 7;
                    value = self.memoryReadAbsoluteIndexedX();
                    value -%= 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xDF: DCP aaaa,X
                0xE0 => { // CPX #aa
                    self.last_cycles = 2;
                    value = self.memoryReadImmediate();
                    value = self.X -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xE1 => { // SBC (aa,X)
                    self.last_cycles = 6;
                    value = self.memoryReadIndexedIndirectX();
                    value_w = self.A -% value -% @as(byte, @intFromBool(self.PS.PS_C));
                    self.PS.PS_N = ((value_w & 0x80) != 0);
                    self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                    self.PS.PS_Z = (value_w == 0);
                    self.PS.PS_C = (value_w < 0x100);

                    if (self.PS.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w2 & 0x8000 != 0)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000 != 0)
                            value_w -%= 0x60;
                        self.A = @truncate(value_w2 & 0xFF);
                    } else self.A = @truncate(value_w & 0xFF);
                },
                // Illegal opcode 0xE2: NOP #aa
                // Illegal opcode 0xE3: ISC (aa,X)
                0xE4 => { // CPX aa
                    self.last_cycles = 3;
                    value = self.memoryReadZeroPage();
                    value = self.X -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xE5 => { // SBC aa
                    self.last_cycles = 3;
                    value = self.memoryReadZeroPage();
                    value_w = self.A -% value -% @as(byte, @intFromBool(self.PS.PS_C));
                    self.PS.PS_N = ((value_w & 0x80) != 0);
                    self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                    self.PS.PS_Z = (value_w == 0);
                    self.PS.PS_C = (value_w < 0x100);

                    if (self.PS.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w2 & 0x8000 != 0)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000 != 0)
                            value_w -%= 0x60;
                        self.A = @truncate(value_w2 & 0xFF);
                    } else self.A = @truncate(value_w & 0xFF);
                },
                0xE6 => { // INC aa
                    self.last_cycles = 5;
                    value = self.memoryReadZeroPage() +% 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xE7: ISC aa
                0xE8 => { // INX
                    self.last_cycles = 2;
                    self.X +%= 1;
                    self.PS.PS_N = ((self.X & 0x80) != 0);
                    self.PS.PS_Z = (self.X == 0);
                },
                0xE9 => { // SBC #aa
                    self.last_cycles = 2;
                    value = self.memoryReadImmediate();
                    value_w = self.A -% value -% @as(byte, @intFromBool(self.PS.PS_C));
                    self.PS.PS_N = ((value_w & 0x80) != 0);
                    self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                    self.PS.PS_Z = (value_w == 0);
                    self.PS.PS_C = (value_w < 0x100);

                    if (self.PS.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w2 & 0x8000 != 0)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000 != 0)
                            value_w -%= 0x60;
                        self.A = @truncate(value_w2 & 0xFF);
                    } else self.A = @truncate(value_w & 0xFF);
                },
                0xEA => { // NOP
                    self.last_cycles = 2;
                    // do nothing

                },
                // Illegal opcode 0xEB: SBC #aa
                0xEC => { // CPX aaaa
                    self.last_cycles = 4;
                    value = self.memoryReadAbsolute();
                    value = self.X -% value;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.PS.PS_C = !self.PS.PS_N;
                },
                0xED => { // SBC aaaa
                    self.last_cycles = 4;
                    value = self.memoryReadAbsolute();
                    value_w = self.A -% value -% @as(byte, @intFromBool(self.PS.PS_C));
                    self.PS.PS_N = ((value_w & 0x80) != 0);
                    self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                    self.PS.PS_Z = (value_w == 0);
                    self.PS.PS_C = (value_w < 0x100);

                    if (self.PS.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w2 & 0x8000 != 0)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000 != 0)
                            value_w -%= 0x60;
                        self.A = @truncate(value_w2 & 0xFF);
                    } else self.A = @truncate(value_w & 0xFF);
                },
                0xEE => { // INC aaaa
                    self.last_cycles = 6;
                    value = self.memoryReadAbsolute() +% 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xEF: ISC aaaa
                0xF0 => { // BEQ aaaa
                    self.last_cycles = 2;
                    value_w = self.memoryReadRelativeAddress();
                    if (self.PS.PS_Z) {
                        self.last_cycles +%= 1;
                        self.PC = value_w;
                    }
                },
                0xF1 => { // SBC (aa),Y
                    self.last_cycles = 5;
                    value = self.memoryReadIndirectIndexedY();
                    value_w = self.A -% value -% @as(byte, @intFromBool(self.PS.PS_C));
                    self.PS.PS_N = ((value_w & 0x80) != 0);
                    self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                    self.PS.PS_Z = (value_w == 0);
                    self.PS.PS_C = (value_w < 0x100);

                    if (self.PS.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w2 & 0x8000 != 0)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000 != 0)
                            value_w -%= 0x60;
                        self.A = @truncate(value_w2 & 0xFF);
                    } else self.A = @truncate(value_w & 0xFF);
                },
                // Illegal opcode 0xF2: KIL
                // Illegal opcode 0xF3: ISC (aa),Y
                // Illegal opcode 0xF4: NOP aa,X
                0xF5 => { // SBC aa,X
                    self.last_cycles = 4;
                    value = self.memoryReadZeroPageIndexedX();
                    value_w = self.A -% value -% @as(byte, @intFromBool(self.PS.PS_C));
                    self.PS.PS_N = ((value_w & 0x80) != 0);
                    self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                    self.PS.PS_Z = (value_w == 0);
                    self.PS.PS_C = (value_w < 0x100);

                    if (self.PS.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w2 & 0x8000 != 0)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000 != 0)
                            value_w -%= 0x60;
                        self.A = @truncate(value_w2 & 0xFF);
                    } else self.A = @truncate(value_w & 0xFF);
                },
                0xF6 => { // INC aa,X
                    self.last_cycles = 6;
                    value = self.memoryReadZeroPageIndexedX() +% 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xF7: ISC aa,X
                0xF8 => { // SED
                    self.last_cycles = 2;
                    self.PS.PS_D = true;
                },
                0xF9 => { // SBC aaaa,Y
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedY();
                    value_w = self.A -% value -% @as(byte, @intFromBool(self.PS.PS_C));
                    self.PS.PS_N = ((value_w & 0x80) != 0);
                    self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                    self.PS.PS_Z = (value_w == 0);
                    self.PS.PS_C = (value_w < 0x100);

                    if (self.PS.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w2 & 0x8000 != 0)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000 != 0)
                            value_w -%= 0x60;
                        self.A = @truncate(value_w2 & 0xFF);
                    } else self.A = @truncate(value_w & 0xFF);
                },
                // Illegal opcode 0xFA: NOP
                // Illegal opcode 0xFB: ISC aaaa,Y
                // Illegal opcode 0xFC: NOP aaaa,X
                0xFD => { // SBC aaaa,X
                    self.last_cycles = 4;
                    value = self.memoryReadAbsoluteIndexedX();
                    value_w = self.A -% value -% @as(byte, @intFromBool(self.PS.PS_C));
                    self.PS.PS_N = ((value_w & 0x80) != 0);
                    self.PS.PS_V = ((self.A ^ value_w) & 0x80) > 0 and ~((self.A ^ value) & 0x80) > 0;
                    self.PS.PS_Z = (value_w == 0);
                    self.PS.PS_C = (value_w < 0x100);

                    if (self.PS.PS_D) {
                        value_w2 = (self.A & 0x0F) -% (value & 0x0F) +% @as(byte, @intFromBool(self.PS.PS_C));
                        if (value_w2 & 0x8000 != 0)
                            value_w = ((value_w -% 0x06) & 0x0F) -% 0x10;
                        value_w2 +%= (self.A & 0xF0) -% (value & 0xF0);
                        if (value_w2 & 0x8000 != 0)
                            value_w -%= 0x60;
                        self.A = @truncate(value_w2 & 0xFF);
                    } else self.A = @truncate(value_w & 0xFF);
                },
                0xFE => { // INC aaaa,X
                    self.last_cycles = 7;
                    value = self.memoryReadAbsoluteIndexedX() +% 1;
                    self.PS.PS_N = ((value & 0x80) != 0);
                    self.PS.PS_Z = (value == 0);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xFF: ISC aaaa,X
                else => {
                    self.last_cycles = 1;
                    return RunResult.RESULT_ILLEGAL_INSTUCTION;
                },
            }

            self.cycles +%= self.last_cycles; // adjust total cycle spent
            if (self.single_step) return RunResult.RESULT_STEP;
        }
        unreachable; // while(true)
    }
};

test "T.cpuRunSingleStep" {
    var memoryMap = Memory.Memory{};
    memoryMap.setPageRW(0, 3);

    var cpu = CPU6502{};
    cpu.init(&memoryMap);
    cpu.single_step = true; // execute only 1 instruction

    memoryMap.writeZero(0, 0xA9); // LDA #
    memoryMap.writeZero(1, 0xFF);
    cpu.PC = 0; // run from (zero-page) address 0
    _ = cpu.run();

    // Check a not initialised page
    try std.testing.expect(cpu.A == 0xFF);
    try std.testing.expect(cpu.PS.PS_N); // 0xFF is a 2's-complement negative number
    try std.testing.expect(cpu.PC == 2);
}
