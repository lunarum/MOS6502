const std = @import("std");
pub const Memory = @import("memory.zig");

pub const byte = Memory.byte;
pub const word = Memory.word;

pub const MEM_STACK_PAGE: byte = 0x01;
pub const MEM_ZERO_PAGE: byte = 0x00;
pub const MEM_NMI: word = 0xFFFA;
pub const MEM_RESET: word = 0xFFFC;
pub const MEM_IRQ_BREAK: word = 0xFFFE;

pub const RunResult = enum {
    STEP,
    BREAK,
    ILLEGAL_INSTRUCTION,
};

/// Processor Status flags: Carry, Zero, Irq disable, Decimal mode, Break mode, oVerflow and Negative
pub const Flags = packed struct(byte) {
    /// Carry flag
    C: bool = false,
    /// Zero flag
    Z: bool = false,
    /// IRQ disable flag
    I: bool = false,
    /// Decimal mode flag
    D: bool = false,
    /// not a real register value; always false, but may be pushed to the stack as a true by BRK
    B: bool = false,
    /// reserved and not a real register value; always 1
    R: bool = true,
    /// oVerflow flag
    V: bool = false,
    /// Negative flag
    N: bool = false,
};

/// The 6502 processor with registers Accumulator, X and Y index registers, Stack Pointer, Program Counter and Processor Status register
/// Also contains status like the last instruction cycles (last_cycles), total executed cycles (cycles) and if in single step modus (single_step).
/// Finally the memory Manager (memoryManager) is part of this cpu structure, which includes the full memory map.
///
/// All access modes are implemented in separate (private) functions, as is the function to run / continue execution.
/// When newly initialized, the cpu starts from the normal address which is stored at MEM_RESET, else it continues from current PC address.
pub const CPU6502 = struct {
    /// Accumulator
    A: byte = 0,
    /// X index register
    X: byte = 0,
    /// Y index register
    Y: byte = 0,
    /// Stack Pointer
    SP: byte = 0,
    /// Program Counter
    PC: word = MEM_RESET,
    /// Processor Status flags
    PS: Flags = Flags{},

    /// #cycles of last executed instruction
    last_cycles: byte = 0,
    /// instruction cycles spent after start (overflows)
    cycles: i128 = 0,
    /// single step modus
    single_step: bool = true,

    memoryManager: *Memory.MemoryManager = undefined,

    pub fn init(self: *CPU6502, memoryManager: *Memory.MemoryManager) void {
        self.memoryManager = memoryManager;
        self.reset();
    }

    /// Resets the CPU to it's initial state i.e. loads PC with the MEM_RESET vector,
    /// resets the stack pointer and I & D flags and sets the cycle counters to 0.
    pub fn reset(self: *CPU6502) void {
        self.PS.I = true;
        self.PS.D = false;
        self.SP = 0xFF;
        self.PC = self.memoryReadAddress(MEM_RESET);
        self.last_cycles = 0;
        self.cycles = 0;
        self.single_step = false;
    }

    /// Returns the page part of an address, effectively clearing the last 8 bits.
    pub fn get_page_part(address: word) word {
        return address & 0xFF00;
    }

    /// Creates and 16-bit (word) address based on the low and high bytes.
    inline fn createAddress(lowByte: byte, highByte: byte) word {
        return @as(word, highByte) << 8 | @as(word, lowByte);
    }

    /// The Negative flag indicate is the sign bit (bit 7) of value is set.
    inline fn calculateN(self: *CPU6502, value: byte) void {
        self.PS.N = (value & 0x80) != 0;
    }

    /// The oVerflow flag indicates if an overflow has occurred.
    /// For example, the specific test for ADC is: if the two input numbers have the same sign, and the result has a different sign, set overflow. Otherwise clear it.
    /// https://stackoverflow.com/questions/66141379/arithmetic-overflow-for-different-signed-numbers-6502-assembly
    /// v = ~(a^operand) & (a^result) & 0x80;
    inline fn calculateV(self: *CPU6502, operand: byte, result: word) void {
        self.PS.V = (~(self.A ^ operand) & (self.A ^ (result >> 8)) & 0x80) > 0;
    }

    /// The Zero flag simply indicates if value is zero.
    inline fn calculateZ(self: *CPU6502, value: byte) void {
        self.PS.Z = value == 0;
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
    /// Push a value (byte) onto the stack (page 1) at location pointed to by SP, which grows downwards, taking 2 cycles.
    fn stackPush(self: *CPU6502, value: byte) void {
        self.last_cycles += 2;
        self.memoryManager.writeStack(self.SP, value);
        self.SP -%= 1;
    }

    /// stackPull
    /// ---------
    /// Pull a value (byte) from the stack (page 1) at location pointed to by SP, which shrinks upwards, taking 2 cycles.
    fn stackPull(self: *CPU6502) byte {
        self.last_cycles += 2;
        self.SP +%= 1;
        return self.memoryManager.readStack(self.SP);
    }

    /// stackPushAddress
    /// ----------------
    /// Push an address (word) onto the stack (page 1) at location pointed to by SP, taking 2 cycles.
    /// The address is pushed MSB first, but because the stack grows downwards, resulting in a little endian stored address.
    fn stackPushAddress(self: *CPU6502, address: word) void {
        self.memoryManager.writeStack(self.SP, @truncate(address >> 8));
        self.SP -%= 1;
        self.memoryManager.writeStack(self.SP, @truncate(address));
        self.SP -%= 1;
    }

    /// stackPullAddress
    /// ----------------
    /// Pull an address (word) from the stack (page 1) at location pointed to by SP, LSB first, because the stack shrinks upwards, taking 3 cycles.
    fn stackPullAddress(self: *CPU6502) word {
        self.last_cycles += 3;
        self.SP +%= 1;
        const lowByte: word = self.memoryManager.readStack(self.SP);
        self.SP +%= 1;
        const highByte: word = self.memoryManager.readStack(self.SP);
        return highByte << 8 | lowByte;
    }

    /// memoryReadImmediate
    /// -------------------
    /// The byte value read is the byte at the memory location pointed to by the PC, taking 1 cycle.
    ///
    /// For example, LDA #$1A loads the value $1A into the accumulator.
    fn memoryReadImmediate(self: *CPU6502) byte {
        const value = self.memoryManager.read(self.PC);
        self.PC +%= 1;
        self.last_cycles += 1;
        return value;
    }

    /// memoryReadAbsolute
    /// ------------------
    /// The byte value read is the byte stored at the memory location pointed to by the PC,
    /// taking 3 cycles.
    ///
    /// For example, LDA $1234 reads the value in memory location $1234 and stores it into the accumulator.
    fn memoryReadAbsolute(self: *CPU6502) byte {
        self.last_cycles += 3;
        return self.memoryManager.read(createAddress(self.memoryReadImmediate(), self.memoryReadImmediate()));
    }

    /// memoryReadAbsoluteAddress
    /// -------------------------
    /// The address (word) read is the address stored at the memory location pointed to by the PC,
    /// taking 3 cycles
    ///
    /// For example, JMP $1A2B loads the address $1A2B into the PC (jumps to).
    fn memoryReadAbsoluteAddress(self: *CPU6502) word {
        self.last_cycles += 3;
        return createAddress(self.memoryReadImmediate(), self.memoryReadImmediate());
    }

    /// memoryWriteAbsolute
    /// -------------------
    /// The byte value which is written at the memory location location pointed to by the PC,
    /// taking 3 cycles
    ///
    /// For example, STA $1A2B stores the present value of the accumulator in memory location $1A2B.
    fn memoryWriteAbsolute(self: *CPU6502, value: byte) void {
        self.last_cycles += 3;
        self.memoryManager.write(createAddress(self.memoryReadImmediate(), self.memoryReadImmediate()), value);
    }

    /// memoryReadZeroPage
    /// ------------------
    /// The byte value read is the byte stored at the zero page memory location (the LSB byte value,
    /// because the MSB is assumed $00) pointed to by the PC, taking 2 cycles
    ///
    /// For example, LDA $34 reads the value in memory location $0034 and stores it into the accumulator.
    fn memoryReadZeroPage(self: *CPU6502) byte {
        self.last_cycles += 2;
        return self.memoryManager.readZero(self.memoryReadImmediate());
    }

    /// memoryWriteZeroPage
    /// -------------------
    /// The byte value written is stored at the zero page memory location (the LSB byte value,
    /// because the MSB is assumed $00) pointed to by the PC, taking 2 cycles
    ///
    /// For example, STA $34 stores the present value of the accumulator in memory location $0034.
    fn memoryWriteZeroPage(self: *CPU6502, value: byte) void {
        self.last_cycles += 2;
        self.memoryManager.writeZero(self.memoryReadImmediate(), value);
    }

    /// memoryReadAbsoluteIndexedX
    /// --------------------------
    /// The byte value read is the byte stored at the memory location pointed to by the PC
    /// with the X register added, taking 3 or 4 cycles
    ///
    /// For example, LDA $1234, X with the X register containing $06 reads the value in
    ///  memory location $123A and stores it into the accumulator.
    fn memoryReadAbsoluteIndexedX(self: *CPU6502) byte {
        const lowByte = self.memoryReadImmediate();
        const highByte = self.memoryReadImmediate();
        const address = createAddress(lowByte, highByte) +% self.X;
        // crossing a page boundary adds an extra cycle
        self.last_cycles += if (highByte != address >> 8) 4 else 3;
        return self.memoryManager.read(address);
    }

    /// memoryReadAbsoluteIndexedY
    /// --------------------------
    /// The byte value read is the byte stored at the memory location pointed to by the PC
    /// with the Y register added, taking 3 or 4 cycles
    ///
    /// For example, LDA $1234, Y with the Y register containing $06 reads the value in
    /// memory location $123A and stores it into the accumulator.
    fn memoryReadAbsoluteIndexedY(self: *CPU6502) byte {
        const lowByte = self.memoryReadImmediate();
        const highByte = self.memoryReadImmediate();
        const address = createAddress(lowByte, highByte) +% self.Y;
        // crossing a page boundary adds an extra cycle
        self.last_cycles += if (highByte != address >> 8) 4 else 3;
        return self.memoryManager.read(address);
    }

    /// memoryWriteAbsoluteIndexedX
    /// --------------------------
    /// The byte value written is stored at the memory location pointed to by the PC
    /// with the X register added, taking 4 cycles
    ///
    /// For example, STA $1234, X with the X register containing $06 writes the accumulator
    /// to memory location $123A.
    fn memoryWriteAbsoluteIndexedX(self: *CPU6502, value: byte) void {
        const lowByte = self.memoryReadImmediate();
        const highByte = self.memoryReadImmediate();
        const address = createAddress(lowByte, highByte) +% self.X;
        self.last_cycles += 4;
        self.memoryManager.write(address, value);
    }

    /// memoryWriteAbsoluteIndexedY
    /// --------------------------
    /// The byte value written is stored at the memory location pointed to by the PC
    /// with the Y register added, taking 4 cycles
    ///
    /// For example, STA $1234, Y with the Y register containing $06 writes the accumulator
    /// to memory location $123A.
    fn memoryWriteAbsoluteIndexedY(self: *CPU6502, value: byte) void {
        const lowByte = self.memoryReadImmediate();
        const highByte = self.memoryReadImmediate();
        const address = createAddress(lowByte, highByte) +% self.Y;
        self.last_cycles += 4;
        self.memoryManager.write(address, value);
    }

    /// memoryReadIndirectAbsoluteAddress
    /// ---------------------------------
    /// Reads the address (word) pointed to by the PC and then uses that as a pointer
    /// in memory to the returned address (again in little endian format), taking 5 cycles
    ///
    /// Due to a bug in the 6502, when the low (first) byte of the pointer address is 0xFF,
    /// the next (high) byte to be read is read from 0x00 on the same page as the pointer address.
    /// e.g. when the pointer address is $04FF, the low address byte is read from $04FF,
    /// but the high byte is read from $0400 instead of the expected $0500!
    ///
    /// Only JMP uses this addressing mode, so e.g. JMP ($1234), with memory $1234 containing $CD and $1235 containing $AB, jumps to $ABCD.
    fn memoryReadIndirectAbsoluteAddress(self: *CPU6502) word {
        const address = createAddress(self.memoryReadImmediate(), self.memoryReadImmediate());
        const lowByte = self.memoryManager.read(address);
        // 6502 bug: fetching the full address doesn't cross a page boundary but wraps around on the same page
        const highByte = self.memoryManager.read(if (address & 0xFF == 0xFF) get_page_part(address) else address + 1);
        self.last_cycles += 5;
        return createAddress(lowByte, highByte);
    }

    /// memoryReadZeroPageIndexedX
    /// --------------------------
    /// The byte value read is read from the zero page memory location (the LSB byte value,
    /// because the MSB is assumed $00) pointed to by the PC with index register X added
    /// with overflow (i.e. without leaving the zero page), taking 3 cycles.
    ///
    /// For example, LDA $34, X, when X contains $F0 reads the byte value in memory location
    /// $0024 ($34 + $F0 = $0124 => $0024) into the accumulator.
    fn memoryReadZeroPageIndexedX(self: *CPU6502) byte {
        self.last_cycles += 3;
        return self.memoryManager.readZero(self.memoryReadImmediate() +% self.X);
    }

    /// memoryReadZeroPageIndexedY
    /// --------------------------
    /// The byte value read is read from the zero page memory location (the LSB byte value,
    /// because the MSB is assumed $00) pointed to by the PC with index register Y added
    /// with overflow (i.e. without leaving the zero page), taking 3 cycles.
    ///
    /// For example, LDA $34, Y, when Y contains $F0 reads the byte value in memory location
    /// $0024 ($34 + $F0 = $0124 => $0024) into the accumulator.
    fn memoryReadZeroPageIndexedY(self: *CPU6502) byte {
        self.last_cycles += 3;
        return self.memoryManager.readZero(self.memoryReadImmediate() +% self.Y);
    }

    /// memoryWriteZeroPageIndexedX
    /// ---------------------------
    /// The byte value written is stored at the zero page memory location (the LSB byte value,
    /// because the MSB is assumed $00) pointed to by the PC with index register X added
    /// with overflow (i.e. without leaving the zero page), taking 3 cycles.
    ///
    /// For example, STA $34, X, when X contains $F0 stores the present value of the accumulator
    /// in memory location $0024 ($34 + $F0 = $0124 => $0024).
    fn memoryWriteZeroPageIndexedX(self: *CPU6502, value: byte) void {
        self.last_cycles += 3;
        self.memoryManager.writeZero(self.memoryReadImmediate() +% self.X, value);
    }

    /// memoryWriteZeroPageIndexedY
    /// ---------------------------
    /// The byte value written is stored at the zero page memory location (the LSB byte value,
    /// because the MSB is assumed $00) pointed to by the PC with index register Y added
    /// with overflow (i.e. without leaving the zero page), taking 3 cycles.
    ///
    /// For example, STA $34, Y, when Y contains $F0 stores the present value of the accumulator in memory location $0024 ($34 + $F0 = $0124 => $0024).
    fn memoryWriteZeroPageIndexedY(self: *CPU6502, value: byte) void {
        self.last_cycles += 3;
        self.memoryManager.writeZero(self.memoryReadImmediate() +% self.Y, value);
    }

    /// memoryReadIndexedIndirectX
    /// --------------------------
    /// Reads the address (word) on the zero page, (address byte) pointed to by the PC with
    /// register X added to that (wrapping around), in little endian format, taking 5 cycles.
    ///
    /// For example, LDA ($34,X), where X contains 6 and memory on $3A containing $CD and on
    /// $3B containing $AB, results in loading the accumulator with the byte value at $ABCD.
    fn memoryReadIndexedIndirectX(self: *CPU6502) byte {
        const addressZ: byte = self.memoryReadImmediate() +% self.X;
        const address = createAddress(self.memoryManager.readZero(addressZ), self.memoryManager.readZero(addressZ +% 1));
        self.last_cycles += 5;
        return self.memoryManager.read(address);
    }

    /// memoryWriteIndexedIndirectX
    /// --------------------------
    /// Writes the value (byte) to the address (word) found on the zero page at (address byte)
    /// pointed to by the PC with register X added to that (wrapping around), in little endian
    /// format, taking 5 cycles.
    ///
    /// Only used by STA, so with STA ($34,X), where X contains 6 and memory on $3A containing
    /// $CD and on $3B containing $AB, results in writing the accumulator byte value to $ABCD.
    fn memoryWriteIndexedIndirectX(self: *CPU6502, value: byte) void {
        const addressZ: byte = self.memoryReadImmediate() +% self.X;
        const address = createAddress(self.memoryManager.readZero(addressZ), self.memoryManager.readZero(addressZ +% 1));
        self.last_cycles += 5;
        self.memoryManager.write(address, value);
    }

    /// memoryReadIndirectIndexedY
    /// --------------------------
    /// Reads the address (word) on the zero page, (address byte) pointed to by the PC in little
    /// endian format with the register Y added to that found address (wrapping around), taking
    /// 4 or 5 cycles.
    ///
    /// For example, LDA ($34),Y, where Y contains 6 and memory on $34 containing $C7 and on $35
    /// containing $AB, results in loading the accumulator with the byte value at $ABCD.
    fn memoryReadIndirectIndexedY(self: *CPU6502) byte {
        const addressZ: byte = self.memoryReadImmediate();
        const highByte = self.memoryManager.readZero(addressZ +% 1);
        const address = createAddress(self.memoryManager.readZero(addressZ), highByte) +% self.Y;
        // crossing a page boundary adds an extra cycle
        self.last_cycles += if (highByte != address >> 8) 5 else 4;
        return self.memoryManager.read(address);
    }

    /// memoryWriteIndirectIndexedY
    /// --------------------------
    /// Writes the value (byte) to the address (word) found on the zero page at (address byte)
    /// pointed to by the PC in little endian format with the register Y added to that found
    /// address (wrapping around), taking 5 cycles.
    ///
    /// Only used by STA, so with STA ($34),Y, where Y contains 6 and memory on $34 containing
    /// $C7 and on $35 containing $AB, results in writing the accumulator byte value to $ABCD.
    fn memoryWriteIndirectIndexedY(self: *CPU6502, value: byte) void {
        const addressZ: byte = self.memoryReadImmediate();
        const address = createAddress(self.memoryManager.readZero(addressZ), self.memoryManager.readZero(addressZ +% 1)) +% self.Y;
        self.last_cycles += 5;
        self.memoryManager.write(address, value);
    }

    /// memoryReadRelativeAddress
    /// -------------------------
    /// Branching instructions use relative addressing i.e. adding an offset (a byte),
    /// pointed to by the PC to that PC, taking 1 or 2 cycles.
    /// The offset is interpreted as 2's complement, so if bit 7 is set it is used as a
    /// negative offset. To calculate that offset, negate all bits and then add 1.
    /// The resulting address is returned; PC is NOT set.
    ///
    /// For example, BEQ $0F will jump (branch) 15 bytes ahead if the Z-flag is set,
    /// while BEQ $F0 will jump 16 bytes back if the Z-flag is set.
    fn memoryReadRelativeAddress(self: *CPU6502) word {
        const offset = self.memoryReadImmediate();
        // check for a two's complement negative offset
        const address = if (offset & 0x80 == 0x80) self.PC -% (~offset + 1) else self.PC +% offset;
        // crossing the page boundary adds an extra cycle
        self.last_cycles += if (get_page_part(address) != get_page_part(self.PC)) 2 else 1;
        return address;
    }

    /// memoryWriteLast
    /// -------------------------
    /// Write to the last address read from, taking 1 cycle.
    fn memoryWriteLast(self: *CPU6502, value_b: byte) void {
        self.memoryManager.writeLast(value_b);
        self.last_cycles += 1;
    }

    /// ADC: ADd to A with Carry
    /// -------------------------
    /// Add value_b and C to A (16-bits), storing truncated 8 bits into A, adding 1 cycle.
    /// Logic mostly taken from https://github.com/tom-seddon/b2/blob/master/src/6502/c/6502.c
    ///
    /// Dependent on flags D & C and affects flags C, N, V & Z
    fn instructionADC(self: *CPU6502, value_b: byte) void {
        self.last_cycles += 1;

        const word_a: word = @as(word, self.A);
        const word_b: word = @as(word, value_b);
        const carry: byte = (if (self.PS.C) 1 else 0);
        var temp_a: word = word_a +% @as(word, word_b) +% @as(word, carry);

        // Z flag is always calculated as in binary add, see http://www.6502.org/tutorials/decimal_mode.html#A
        self.A = @truncate(temp_a);
        self.calculateZ(self.A);

        if (self.PS.D) { // Decimal mode?
            // Start with lower digit
            temp_a = (word_a & 0x0F) +% (word_b & 0x0F) +% carry;
            if (temp_a > 0x09) { // result with a carry to the upper digit?
                temp_a +%= 6;
            }
            if (temp_a <= 0x0F) {
                // add upper digits
                temp_a = (temp_a & 0x0F) +% (word_a & 0xF0) +% (word_b & 0xF0);
            } else {
                // adding upper digits, while correcting the lower digit to decimal and adding carry to upper digit
                temp_a = (temp_a & 0x0F) +% (word_a & 0xF0) +% (word_b & 0xF0) +% 0x10;
            }

            self.PS.N = temp_a & 0x80 != 0;
            self.PS.V = ((word_a ^ temp_a) & 0x80) != 0 and ((word_a ^ word_b) & 0x80) == 0;
            if (temp_a & 0x1F0 > 0x90) { // overflow result?
                temp_a +%= 0x60; // correct the upper digit to a valid decimal
            }
            self.PS.C = temp_a & 0x0FF0 > 0x00F0;
            self.A = @as(byte, @truncate(temp_a));
        } else {
            self.calculateN(self.A);
            self.PS.C = temp_a & 0xFF00 > 0;
            self.PS.V = (~(word_a ^ word_b) & (word_a ^ temp_a) & 0x80) != 0;
        }
    }

    /// AND: Bitwise AND with A;
    /// -------------------------
    /// Bitwise AND with A, putting result back into A.
    ///
    /// Affects N & Z (based on the result).
    fn instructionAND(self: *CPU6502, value: byte) void {
        self.A &= value;
        self.calculateN(self.A);
        self.calculateZ(self.A);
    }

    /// ASL: Arithmetic Shift Left;
    /// -------------------------
    /// Left shift 1 bit of value_b, adding 0 on the right, taking 1 extra cycle.
    ///
    /// Affects flags C (value_b bit 7), N & Z (based on returned result).
    fn instructionASL(self: *CPU6502, value_b: byte) byte {
        self.last_cycles += 1;
        self.PS.C = ((value_b & 0x80) != 0);
        const value = value_b << 1;
        self.calculateN(value);
        self.calculateZ(value);
        return value;
    }

    /// BIT: test BITs;
    /// -------------------------
    /// Sets flags based on value_b, adding 1 cycle.
    ///
    /// Affects flags N (value_b bit 7), V (value_b bit 6) & Z (value_b & A equals zero).
    fn instructionBIT(self: *CPU6502, value_b: byte) void {
        self.last_cycles += 1;
        self.calculateN(value_b);
        self.PS.V = ((value_b & 0x40) != 0);
        self.calculateZ(value_b & self.A);
    }

    /// Branch: branch, based on a flag value.
    /// -------------------------
    /// Reading the relative address and branch if the given flag is true, taking at least 2 cycles
    /// and 3 or 4 when actually branching.
    fn instructionBranch(self: *CPU6502, flag: bool) void {
        const address = self.memoryReadRelativeAddress();
        if (flag) {
            // branching costs 1 cycle and a page boundary crossing costs an extra cycle
            self.last_cycles +%= if (get_page_part(address) != get_page_part(self.PC)) 2 else 1;
            self.PC = address;
        }
    }

    /// CMP: CoMPare A; sets flags based on comparing (subtracting) value_b and (from) A.
    /// -------------------------
    /// Affects flags C (A >= value_b), N (A < value_b) & Z (A == value_b).
    fn instructionCMP(self: *CPU6502, value_b: byte) void {
        const value = self.A -% value_b;
        self.calculateN(value);
        self.calculateZ(value);
        self.PS.C = self.A >= value_b;
    }

    /// CPX: ComPare X; sets flags based on comparing (subtracting) value_b and (from) X.
    /// -------------------------
    /// Affects flags C (X >= value_b), N (X < value_b) & Z (X == value_b).
    fn instructionCPX(self: *CPU6502, value_b: byte) void {
        const value = self.X -% value_b;
        self.calculateN(value);
        self.calculateZ(value);
        self.PS.C = self.X >= value_b;
    }

    /// CPY: ComPare Y; sets flags based on comparing (subtracting) value_b and (from) Y.
    /// -------------------------
    /// Affects flags C (Y >= value_b), N (Y < value_b) & Z (Y == value_b).
    fn instructionCPY(self: *CPU6502, value_b: byte) void {
        const value = self.Y -% value_b;
        self.calculateN(value);
        self.calculateZ(value);
        self.PS.C = self.Y >= value_b;
    }

    /// EOR: Exclusive OR value_b with A
    /// -------------------------
    /// Affects flags N & Z.
    fn instructionEOR(self: *CPU6502, value_b: byte) void {
        self.A ^= value_b;
        self.calculateN(self.A);
        self.calculateZ(self.A);
    }

    /// ORA: OR value_b with A
    /// -------------------------
    /// Affects flags N & Z.
    fn instructionORA(self: *CPU6502, value_b: byte) void {
        self.A |= value_b;
        self.calculateN(self.A);
        self.calculateZ(self.A);
    }

    /// ROR: ROtate Right
    /// -------------------------
    /// Right shift 1 bit of value_b, adding C flag on the left, taking 1 extra cycle.
    ///
    /// Affects flags C (value_b bit 7), N & Z (based on returned result).
    fn instructionROR(self: *CPU6502, value_b: byte) byte {
        self.last_cycles += 1;
        var value = value_b >> 1;
        if (self.PS.C) {
            value |= 0x80;
        }
        self.PS.C = ((value_b & 0x01) != 0);
        self.calculateN(value);
        self.calculateZ(value);
        return value;
    }

    /// ROL: ROtate Left
    /// -------------------------
    /// Left shift 1 bit of value_b, adding C flag on the right, taking 1 extra cycle.
    ///
    /// Affects flags C (value_b bit 0), N & Z (based on returned result).
    fn instructionROL(self: *CPU6502, value_b: byte) byte {
        self.last_cycles += 1;
        const tmp_PS_C = self.PS.C;
        self.PS.C = ((value_b & 0x80) != 0);
        const value = (value_b << 1) | @as(byte, @intFromBool(tmp_PS_C));
        self.calculateN(value);
        self.calculateZ(value);
        return value;
    }

    /// SBC: SuBtract from A with Carry
    /// -------------------------
    /// Subtract value_b and C to A (16-bits), storing truncated 8 bits into A, taking 1 extra cycle.
    /// Logic mostly taken from https://github.com/tom-seddon/b2/blob/master/src/6502/c/6502.c
    ///
    /// Dependent on flags D & C and affects flags C, N, V & Z
    fn instructionSBC(self: *CPU6502, value_b: byte) void {
        self.last_cycles += 1;

        const word_a: word = @as(word, self.A);
        const word_b: word = @as(word, value_b);
        const negated_b = ~value_b; // 2's-complement negation - 1; subtraction is equal to addition with the negated value_b and 6502 always subtract 1 extra
        const carry: byte = (if (self.PS.C) 1 else 0);
        var temp_a: word = word_a +% @as(word, negated_b) +% @as(word, carry);

        // Z & N flags are always calculated as in binary add, see http://www.6502.org/tutorials/decimal_mode.html#A
        self.A = @truncate(temp_a);
        self.calculateZ(self.A);
        self.calculateN(self.A);
        self.PS.C = temp_a & 0xFF00 > 0;

        if (self.PS.D) { // Decimal mode?
            self.PS.V = ((word_a ^ temp_a) & 0x80) != 0 and ((word_a ^ word_b) & 0x80) != 0;

            // Start with lower digit
            temp_a = (word_a & 0x0F) -% (word_b & 0x0F) +% carry -% 1;
            if (temp_a & 0x10 > 0) { // result with a borrow from the upper digit?
                // subtract upper digits, while correcting lower digit to decimal and borrow from upper digit
                temp_a = ((temp_a -% 0x06) & 0x0F) | ((word_a & 0xF0) -% (word_b & 0xF0) -% 0x10);
            } else {
                // add upper digits
                temp_a = (temp_a & 0x0F) | ((word_a & 0xF0) -% (word_b & 0xF0));
            }
            if (temp_a & 0x100 > 0) { // negative result?
                temp_a -%= 0x60; // ignore negative, but correct the upper digit to decimal
            }
            self.A = @as(byte, @truncate(temp_a));
        } else {
            self.PS.V = ((word_a ^ word_b) & (word_a ^ temp_a) & 0x80) != 0;
        }
    }

    /// run - start executing from the current PC address
    ///
    /// If single-step is true, only a single opcode will be processed.
    ///
    /// returns when single-stepping (after 1 opcode) or when a BRK or illegal opcode is processed.
    pub fn run(self: *CPU6502) RunResult {
        var opcode: byte = undefined;
        var value: byte = undefined;
        var value_w: word = undefined;

        while (true) {
            var result = RunResult.STEP;
            self.last_cycles = 0;
            opcode = self.memoryReadImmediate();
            switch (opcode) {
                0x00 => { // BRK
                    self.stackPushAddress(self.PC +% 1); // so that BRK may be used to replace a 2 byte instruction
                    self.PS.B = true; // Non-IRQ/NMI status push i.e. PS_B must be pushed as true
                    self.stackPush(@bitCast(self.PS));
                    self.PS.B = false;
                    self.PS.I = true;
                    self.PC = self.memoryReadAddress(MEM_IRQ_BREAK);
                    self.last_cycles += 2;
                    result = RunResult.BREAK;
                },
                0x01 => { // ORA (aa,X)
                    self.instructionORA(self.memoryReadIndexedIndirectX());
                },
                // Illegal opcode 0x02: KIL
                // Illegal opcode 0x03: SLO (aa,X)
                // Illegal opcode 0x04: NOP aa
                0x05 => { // ORA aa
                    self.instructionORA(self.memoryReadZeroPage());
                },
                0x06 => { // ASL aa
                    self.memoryWriteLast(self.instructionASL(self.memoryReadZeroPage()));
                },
                // Illegal opcode 0x07: SLO aa
                0x08 => { // PHP
                    self.PS.B = true; // Non-IRQ/NMI status push i.e. PS_B must be pushed as true
                    self.stackPush(@bitCast(self.PS));
                    self.PS.B = false;
                },
                0x09 => { // ORA #aa
                    self.instructionORA(self.memoryReadImmediate());
                },
                0x0A => { // ASL
                    self.A = self.instructionASL(self.A);
                },
                // Illegal opcode 0x0B: ANC #aa
                // Illegal opcode 0x0C: NOP aaaa
                0x0D => { // ORA aaaa
                    self.instructionORA(self.memoryReadAbsolute());
                },
                0x0E => { // ASL aaaa
                    self.memoryWriteLast(self.instructionASL(self.memoryReadAbsolute()));
                },
                // Illegal opcode 0x0F: SLO aaaa
                0x10 => { // BPL aaaa
                    self.instructionBranch(!self.PS.N);
                },
                0x11 => { // ORA (aa),Y
                    self.instructionORA(self.memoryReadIndirectIndexedY());
                },
                // Illegal opcode 0x12: KIL
                // Illegal opcode 0x13: SLO (aa),Y
                // Illegal opcode 0x14: NOP aa,X
                0x15 => { // ORA aa,X
                    self.instructionORA(self.memoryReadZeroPageIndexedX());
                },
                0x16 => { // ASL aa,X
                    self.memoryWriteLast(self.instructionASL(self.memoryReadZeroPageIndexedX()));
                },
                // Illegal opcode 0x17: SLO aa,X
                0x18 => { // CLC
                    self.last_cycles += 1;
                    self.PS.C = false;
                },
                0x19 => { // ORA aaaa,Y
                    self.instructionORA(self.memoryReadAbsoluteIndexedY());
                },
                // Illegal opcode 0x1A: NOP
                // Illegal opcode 0x1B: SLO aaaa,Y
                // Illegal opcode 0x1C: NOP aaaa,X
                0x1D => { // ORA aaaa,X
                    self.instructionORA(self.memoryReadAbsoluteIndexedX());
                },
                0x1E => { // ASL aaaa,X
                    self.memoryWriteLast(self.instructionASL(self.memoryReadAbsoluteIndexedX()));
                },
                // Illegal opcode 0x1F: SLO aaaa,X
                0x20 => { // JSR aaaa
                    value_w = self.memoryReadAbsoluteAddress();
                    self.stackPushAddress(self.PC -% 1);
                    self.PC = value_w;
                },
                0x21 => { // AND (aa,X)
                    self.instructionAND(self.memoryReadIndexedIndirectX());
                },
                // Illegal opcode 0x22: KIL
                // Illegal opcode 0x23: RLA (aa,X)
                0x24 => { // BIT aa
                    self.instructionBIT(self.memoryReadZeroPage());
                },
                0x25 => { // AND aa
                    self.instructionAND(self.memoryReadZeroPage());
                },
                0x26 => { // ROL aa
                    self.memoryWriteLast(self.instructionROL(self.memoryReadZeroPage()));
                },
                // Illegal opcode 0x27: RLA aa
                0x28 => { // PLP
                    self.last_cycles += 1;
                    self.PS = @bitCast(self.stackPull());
                    self.PS.R = true; // Pull value is ignored; value is always true
                    self.PS.B = false; // Pull value is ignored; value is always false
                },
                0x29 => { // AND #aa
                    self.instructionAND(self.memoryReadImmediate());
                },
                0x2A => { // ROL
                    self.A = self.instructionROL(self.A);
                },
                // Illegal opcode 0x2B: ANC #aa
                0x2C => { // BIT aaaa
                    self.instructionBIT(self.memoryReadAbsolute());
                },
                0x2D => { // AND aaaa
                    self.instructionAND(self.memoryReadAbsolute());
                },
                0x2E => { // ROL aaaa
                    self.memoryWriteLast(self.instructionROL(self.memoryReadAbsolute()));
                },
                // Illegal opcode 0x2F: RLA aaaa
                0x30 => { // BMI aaaa
                    self.instructionBranch(self.PS.N);
                },
                0x31 => { // AND (aa),Y
                    self.instructionAND(self.memoryReadIndirectIndexedY());
                },
                // Illegal opcode 0x32: KIL
                // Illegal opcode 0x33: RLA (aa),Y
                // Illegal opcode 0x34: NOP aa,X
                0x35 => { // AND aa,X
                    self.instructionAND(self.memoryReadZeroPageIndexedX());
                },
                0x36 => { // ROL aa,X
                    self.memoryWriteLast(self.instructionROL(self.memoryReadZeroPageIndexedX()));
                },
                // Illegal opcode 0x37: RLA aa,X
                0x38 => { // SEC
                    self.last_cycles += 2;
                    self.PS.C = true;
                },
                0x39 => { // AND aaaa,Y
                    self.instructionAND(self.memoryReadAbsoluteIndexedY());
                },
                // Illegal opcode 0x3A: NOP
                // Illegal opcode 0x3B: RLA aaaa,Y
                // Illegal opcode 0x3C: NOP aaaa,X
                0x3D => { // AND aaaa,X
                    self.instructionAND(self.memoryReadAbsoluteIndexedX());
                },
                0x3E => { // ROL aaaa,X
                    self.memoryWriteLast(self.instructionROL(self.memoryReadAbsoluteIndexedX()));
                },
                // Illegal opcode 0x3F: RLA aaaa,X
                0x40 => { // RTI
                    self.PS = @bitCast(self.stackPull());
                    self.PS.R = true; // Pull value is ignored; value is always true
                    self.PS.B = false; // Pull value is ignored; value is always false
                    self.PC = self.stackPullAddress();
                },
                0x41 => { // EOR (aa,X)
                    self.last_cycles += 6;
                    value = self.memoryReadIndexedIndirectX();
                    self.A ^= value;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                // Illegal opcode 0x42: KIL
                // Illegal opcode 0x43: SRE (aa,X)
                // Illegal opcode 0x44: NOP aa
                0x45 => { // EOR aa
                    self.last_cycles += 3;
                    value = self.memoryReadZeroPage();
                    self.A ^= value;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0x46 => { // LSR aa
                    self.last_cycles += 5;
                    value = self.memoryReadZeroPage();
                    self.PS.C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x47: SRE aa
                0x48 => { // PHA
                    self.stackPush(self.A);
                },
                0x49 => { // EOR #aa
                    self.last_cycles += 2;
                    value = self.memoryReadImmediate();
                    self.A ^= value;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0x4A => { // LSR
                    self.last_cycles += 2;
                    self.PS.C = ((self.A & 0x01) != 0);
                    self.A >>= 1;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                // Illegal opcode 0x4B: ALR #aa
                0x4C => { // JMP aaaa
                    self.last_cycles += 3;
                    self.PC = self.memoryReadAbsoluteAddress();
                },
                0x4D => { // EOR aaaa
                    self.last_cycles += 4;
                    value = self.memoryReadAbsolute();
                    self.A ^= value;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0x4E => { // LSR aaaa
                    self.last_cycles += 6;
                    value = self.memoryReadAbsolute();
                    self.PS.C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x4F: SRE aaaa
                0x50 => { // BVC aaaa
                    self.instructionBranch(!self.PS.V);
                },
                0x51 => { // EOR (aa),Y
                    self.last_cycles += 5;
                    value = self.memoryReadIndirectIndexedY();
                    self.A ^= value;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                // Illegal opcode 0x52: KIL
                // Illegal opcode 0x53: SRE (aa),Y
                // Illegal opcode 0x54: NOP aa,X
                0x55 => { // EOR aa,X
                    self.last_cycles += 4;
                    value = self.memoryReadZeroPageIndexedX();
                    self.A ^= value;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0x56 => { // LSR aa,X
                    self.last_cycles += 6;
                    value = self.memoryReadZeroPageIndexedX();
                    self.PS.C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x57: SRE aa,X
                0x58 => { // CLI
                    self.last_cycles += 2;
                    self.PS.I = false;
                },
                0x59 => { // EOR aaaa,Y
                    self.last_cycles += 4;
                    value = self.memoryReadAbsoluteIndexedY();
                    self.A ^= value;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                // Illegal opcode 0x5A: NOP
                // Illegal opcode 0x5B: SRE aaaa,Y
                // Illegal opcode 0x5C: NOP aaaa,X
                0x5D => { // EOR aaaa,X
                    self.last_cycles += 4;
                    value = self.memoryReadAbsoluteIndexedX();
                    self.A ^= value;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0x5E => { // LSR aaaa,X
                    self.last_cycles += 7;
                    value = self.memoryReadAbsoluteIndexedX();
                    self.PS.C = ((value & 0x01) != 0);
                    value >>= 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0x5F: SRE aaaa,X
                0x60 => { // RTS
                    self.last_cycles += 3;
                    self.PC = self.stackPullAddress() +% 1; // Actual address to continue is 1 byte further (see JSR implementation)
                },
                0x61 => { // ADC (aa,X)
                    self.instructionADC(self.memoryReadIndexedIndirectX());
                },
                // Illegal opcode 0x62: KIL
                // Illegal opcode 0x63: RRA (aa,X)
                // Illegal opcode 0x64: NOP aa
                0x65 => { // ADC aa
                    self.instructionADC(self.memoryReadZeroPage());
                },
                0x66 => { // ROR aa
                    self.memoryWriteLast(self.instructionROR(self.memoryReadZeroPage()));
                },
                // Illegal opcode 0x67: RRA aa
                0x68 => { // PLA
                    self.last_cycles += 1;
                    self.A = self.stackPull();
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0x69 => { // ADC #aa
                    self.instructionADC(self.memoryReadImmediate());
                },
                0x6A => { // ROR
                    self.A = self.instructionROR(self.A);
                },
                // Illegal opcode 0x6B: ARR #aa
                0x6C => { // JMP (aaaa)
                    self.last_cycles += 5;
                    self.PC = self.memoryReadIndirectAbsoluteAddress();
                },
                0x6D => { // ADC aaaa
                    self.instructionADC(self.memoryReadAbsolute());
                },
                0x6E => { // ROR aaaa
                    self.memoryWriteLast(self.instructionROR(self.memoryReadAbsolute()));
                },
                // Illegal opcode 0x6F: RRA aaaa
                0x70 => { // BVS aaaa
                    self.instructionBranch(self.PS.V);
                },
                0x71 => { // ADC (aa),Y
                    self.instructionADC(self.memoryReadIndirectIndexedY());
                },
                // Illegal opcode 0x72: KIL
                // Illegal opcode 0x73: RRA (aa),Y
                // Illegal opcode 0x74: NOP aa,X
                0x75 => { // ADC aa,X
                    self.instructionADC(self.memoryReadZeroPageIndexedX());
                },
                0x76 => { // ROR aa,X
                    self.memoryWriteLast(self.instructionROR(self.memoryReadZeroPageIndexedX()));
                },
                // Illegal opcode 0x77: RRA aa,X
                0x78 => { // SEI
                    self.last_cycles += 2;
                    self.PS.I = true;
                },
                0x79 => { // ADC aaaa,Y
                    self.instructionADC(self.memoryReadAbsoluteIndexedY());
                },
                // Illegal opcode 0x7A: NOP
                // Illegal opcode 0x7B: RRA aaaa,Y
                // Illegal opcode 0x7C: NOP aaaa,X
                0x7D => { // ADC aaaa,X
                    self.instructionADC(self.memoryReadAbsoluteIndexedX());
                },
                0x7E => { // ROR aaaa,X
                    self.memoryWriteLast(self.instructionROR(self.memoryReadAbsoluteIndexedX()));
                },
                // Illegal opcode 0x7F: RRA aaaa,X
                // Illegal opcode 0x80: NOP #aa
                0x81 => { // STA (aa,X)
                    self.last_cycles += 6;
                    self.memoryWriteIndexedIndirectX(self.A);
                },
                // Illegal opcode 0x82: NOP #aa
                // Illegal opcode 0x83: SAX (aa,X)
                0x84 => { // STY aa
                    self.last_cycles += 3;
                    self.memoryWriteZeroPage(self.Y);
                },
                0x85 => { // STA aa
                    self.last_cycles += 3;
                    self.memoryWriteZeroPage(self.A);
                },
                0x86 => { // STX aa
                    self.last_cycles += 3;
                    self.memoryWriteZeroPage(self.X);
                },
                // Illegal opcode 0x87: SAX aa
                0x88 => { // DEY
                    self.last_cycles += 2;
                    self.Y -%= 1;
                    self.calculateN(self.Y);
                    self.calculateZ(self.Y);
                },
                // Illegal opcode 0x89: NOP #aa
                0x8A => { // TXA
                    self.last_cycles += 2;
                    self.A = self.X;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                // Illegal opcode 0x8B: XAA #aa
                0x8C => { // STY aaaa
                    self.last_cycles += 4;
                    self.memoryWriteAbsolute(self.Y);
                },
                0x8D => { // STA aaaa
                    self.last_cycles += 4;
                    self.memoryWriteAbsolute(self.A);
                },
                0x8E => { // STX aaaa
                    self.last_cycles += 4;
                    self.memoryWriteAbsolute(self.X);
                },
                // Illegal opcode 0x8F: SAX aaaa
                0x90 => { // BCC aaaa
                    self.instructionBranch(!self.PS.C);
                },
                0x91 => { // STA (aa),Y
                    self.last_cycles += 6;
                    self.memoryWriteIndirectIndexedY(self.A);
                },
                // Illegal opcode 0x92: KIL
                // Illegal opcode 0x93: AHX (aa),Y
                0x94 => { // STY aa,X
                    self.last_cycles += 4;
                    self.memoryWriteZeroPageIndexedX(self.Y);
                },
                0x95 => { // STA aa,X
                    self.last_cycles += 4;
                    self.memoryWriteZeroPageIndexedX(self.A);
                },
                0x96 => { // STX aa,Y
                    self.last_cycles += 4;
                    self.memoryWriteZeroPageIndexedY(self.X);
                },
                // Illegal opcode 0x97: SAX aa,Y
                0x98 => { // TYA
                    self.last_cycles += 2;
                    self.A = self.Y;
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0x99 => { // STA aaaa,Y
                    self.last_cycles += 5;
                    self.memoryWriteAbsoluteIndexedY(self.A);
                },
                0x9A => { // TXS
                    self.last_cycles += 2;
                    self.SP = self.X;
                },
                // Illegal opcode 0x9B: TAS aaaa,Y
                // Illegal opcode 0x9C: SHY aaaa,X
                0x9D => { // STA aaaa,X
                    self.last_cycles += 5;
                    self.memoryWriteAbsoluteIndexedX(self.A);
                },
                // Illegal opcode 0x9E: SHX aaaa,Y
                // Illegal opcode 0x9F: AHX aaaa,Y
                0xA0 => { // LDY #aa
                    self.last_cycles += 2;
                    self.Y = self.memoryReadImmediate();
                    self.calculateN(self.Y);
                    self.calculateZ(self.Y);
                },
                0xA1 => { // LDA (aa,X)
                    self.last_cycles += 6;
                    self.A = self.memoryReadIndexedIndirectX();
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0xA2 => { // LDX #aa
                    self.last_cycles += 2;
                    self.X = self.memoryReadImmediate();
                    self.calculateN(self.X);
                    self.calculateZ(self.X);
                },
                // Illegal opcode 0xA3: LAX (aa,X)
                0xA4 => { // LDY aa
                    self.last_cycles += 3;
                    self.Y = self.memoryReadZeroPage();
                    self.calculateN(self.Y);
                    self.calculateZ(self.Y);
                },
                0xA5 => { // LDA aa
                    self.last_cycles += 3;
                    self.A = self.memoryReadZeroPage();
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0xA6 => { // LDX aa
                    self.last_cycles += 3;
                    self.X = self.memoryReadZeroPage();
                    self.calculateN(self.X);
                    self.calculateZ(self.X);
                },
                // Illegal opcode 0xA7: LAX aa
                0xA8 => { // TAY
                    self.last_cycles += 2;
                    self.Y = self.A;
                    self.calculateN(self.Y);
                    self.calculateZ(self.Y);
                },
                0xA9 => { // LDA #aa
                    self.last_cycles += 2;
                    self.A = self.memoryReadImmediate();
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0xAA => { // TAX
                    self.last_cycles += 2;
                    self.X = self.A;
                    self.calculateN(self.X);
                    self.calculateZ(self.X);
                },
                // Illegal opcode 0xAB: LAX #aa
                0xAC => { // LDY aaaa
                    self.last_cycles += 4;
                    self.Y = self.memoryReadAbsolute();
                    self.calculateN(self.Y);
                    self.calculateZ(self.Y);
                },
                0xAD => { // LDA aaaa
                    self.last_cycles += 4;
                    self.A = self.memoryReadAbsolute();
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0xAE => { // LDX aaaa
                    self.last_cycles += 4;
                    self.X = self.memoryReadAbsolute();
                    self.calculateN(self.X);
                    self.calculateZ(self.X);
                },
                // Illegal opcode 0xAF: LAX aaaa
                0xB0 => { // BCS aaaa
                    self.instructionBranch(self.PS.C);
                },
                0xB1 => { // LDA (aa),Y
                    self.last_cycles += 5;
                    self.A = self.memoryReadIndirectIndexedY();
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                // Illegal opcode 0xB2: KIL
                // Illegal opcode 0xB3: LAX (aa),Y
                0xB4 => { // LDY aa,X
                    self.last_cycles += 4;
                    self.Y = self.memoryReadZeroPageIndexedX();
                    self.calculateN(self.Y);
                    self.calculateZ(self.Y);
                },
                0xB5 => { // LDA aa,X
                    self.last_cycles += 4;
                    self.A = self.memoryReadZeroPageIndexedX();
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0xB6 => { // LDX aa,Y
                    self.last_cycles += 4;
                    self.X = self.memoryReadZeroPageIndexedY();
                    self.calculateN(self.X);
                    self.calculateZ(self.X);
                },
                // Illegal opcode 0xB7: LAX aa,Y
                0xB8 => { // CLV
                    self.last_cycles += 2;
                    self.PS.V = false;
                },
                0xB9 => { // LDA aaaa,Y
                    self.last_cycles += 4;
                    self.A = self.memoryReadAbsoluteIndexedY();
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0xBA => { // TSX
                    self.last_cycles += 2;
                    self.X = self.SP;
                    self.calculateN(self.X);
                    self.calculateZ(self.X);
                },
                // Illegal opcode 0xBB: LAS aaaa,Y
                0xBC => { // LDY aaaa,X
                    self.last_cycles += 4;
                    self.Y = self.memoryReadAbsoluteIndexedX();
                    self.calculateN(self.Y);
                    self.calculateZ(self.Y);
                },
                0xBD => { // LDA aaaa,X
                    self.last_cycles += 4;
                    self.A = self.memoryReadAbsoluteIndexedX();
                    self.calculateN(self.A);
                    self.calculateZ(self.A);
                },
                0xBE => { // LDX aaaa,Y
                    self.last_cycles += 4;
                    self.X = self.memoryReadAbsoluteIndexedY();
                    self.calculateN(self.X);
                    self.calculateZ(self.X);
                },
                // Illegal opcode 0xBF: LAX aaaa,Y
                0xC0 => { // CPY #aa
                    self.instructionCPY(self.memoryReadImmediate());
                },
                0xC1 => { // CMP (aa,X)
                    self.instructionCMP(self.memoryReadIndexedIndirectX());
                },
                // Illegal opcode 0xC2: NOP #aa
                // Illegal opcode 0xC3: DCP (aa,X)
                0xC4 => { // CPY aa
                    self.instructionCPY(self.memoryReadZeroPage());
                },
                0xC5 => { // CMP aa
                    self.instructionCMP(self.memoryReadZeroPage());
                },
                0xC6 => { // DEC aa
                    self.last_cycles += 5;
                    value = self.memoryReadZeroPage();
                    value -%= 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xC7: DCP aa
                0xC8 => { // INY
                    self.last_cycles += 2;
                    self.Y +%= 1;
                    self.calculateN(self.Y);
                    self.calculateZ(self.Y);
                },
                0xC9 => { // CMP #aa
                    self.instructionCMP(self.memoryReadImmediate());
                },
                0xCA => { // DEX
                    self.last_cycles += 2;
                    self.X -%= 1;
                    self.calculateN(self.X);
                    self.calculateZ(self.X);
                },
                // Illegal opcode 0xCB: AXS #aa
                0xCC => { // CPY aaaa
                    self.instructionCPY(self.memoryReadAbsolute());
                },
                0xCD => { // CMP aaaa
                    self.instructionCMP(self.memoryReadAbsolute());
                },
                0xCE => { // DEC aaaa
                    self.last_cycles += 6;
                    value = self.memoryReadAbsolute();
                    value -%= 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xCF: DCP aaaa
                0xD0 => { // BNE aaaa
                    self.instructionBranch(!self.PS.Z);
                },
                0xD1 => { // CMP (aa),Y
                    self.instructionCMP(self.memoryReadIndirectIndexedY());
                },
                // Illegal opcode 0xD2: KIL
                // Illegal opcode 0xD3: DCP (aa),Y
                // Illegal opcode 0xD4: NOP aa,X
                0xD5 => { // CMP aa,X
                    self.instructionCMP(self.memoryReadZeroPageIndexedX());
                },
                0xD6 => { // DEC aa,X
                    self.last_cycles += 6;
                    value = self.memoryReadZeroPageIndexedX();
                    value -%= 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xD7: DCP aa,X
                0xD8 => { // CLD
                    self.last_cycles += 2;
                    self.PS.D = false;
                },
                0xD9 => { // CMP aaaa,Y
                    self.instructionCMP(self.memoryReadAbsoluteIndexedY());
                },
                // Illegal opcode 0xDA: NOP
                // Illegal opcode 0xDB: DCP aaaa,Y
                // Illegal opcode 0xDC: NOP aaaa,X
                0xDD => { // CMP aaaa,X
                    self.instructionCMP(self.memoryReadAbsoluteIndexedX());
                },
                0xDE => { // DEC aaaa,X
                    self.last_cycles += 7;
                    value = self.memoryReadAbsoluteIndexedX();
                    value -%= 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xDF: DCP aaaa,X
                0xE0 => { // CPX #aa
                    self.instructionCPX(self.memoryReadImmediate());
                },
                0xE1 => { // SBC (aa,X)
                    self.instructionSBC(self.memoryReadIndexedIndirectX());
                },
                // Illegal opcode 0xE2: NOP #aa
                // Illegal opcode 0xE3: ISC (aa,X)
                0xE4 => { // CPX aa
                    self.instructionCPX(self.memoryReadZeroPage());
                },
                0xE5 => { // SBC aa
                    self.instructionSBC(self.memoryReadZeroPage());
                },
                0xE6 => { // INC aa
                    self.last_cycles += 5;
                    value = self.memoryReadZeroPage() +% 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xE7: ISC aa
                0xE8 => { // INX
                    self.last_cycles += 2;
                    self.X +%= 1;
                    self.calculateN(self.X);
                    self.calculateZ(self.X);
                },
                0xE9 => { // SBC #aa
                    self.instructionSBC(self.memoryReadImmediate());
                },
                0xEA => { // NOP
                    self.last_cycles += 2;
                    // do nothing

                },
                // Illegal opcode 0xEB: SBC #aa
                0xEC => { // CPX aaaa
                    self.instructionCPX(self.memoryReadAbsolute());
                },
                0xED => { // SBC aaaa
                    self.instructionSBC(self.memoryReadAbsolute());
                },
                0xEE => { // INC aaaa
                    self.last_cycles += 6;
                    value = self.memoryReadAbsolute() +% 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xEF: ISC aaaa
                0xF0 => { // BEQ aaaa
                    self.instructionBranch(self.PS.Z);
                },
                0xF1 => { // SBC (aa),Y
                    self.instructionSBC(self.memoryReadIndirectIndexedY());
                },
                // Illegal opcode 0xF2: KIL
                // Illegal opcode 0xF3: ISC (aa),Y
                // Illegal opcode 0xF4: NOP aa,X
                0xF5 => { // SBC aa,X
                    self.instructionSBC(self.memoryReadZeroPageIndexedX());
                },
                0xF6 => { // INC aa,X
                    self.last_cycles += 6;
                    value = self.memoryReadZeroPageIndexedX() +% 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xF7: ISC aa,X
                0xF8 => { // SED
                    self.last_cycles += 2;
                    self.PS.D = true;
                },
                0xF9 => { // SBC aaaa,Y
                    self.instructionSBC(self.memoryReadAbsoluteIndexedY());
                },
                // Illegal opcode 0xFA: NOP
                // Illegal opcode 0xFB: ISC aaaa,Y
                // Illegal opcode 0xFC: NOP aaaa,X
                0xFD => { // SBC aaaa,X
                    self.instructionSBC(self.memoryReadAbsoluteIndexedX());
                },
                0xFE => { // INC aaaa,X
                    self.last_cycles += 7;
                    value = self.memoryReadAbsoluteIndexedX() +% 1;
                    self.calculateN(value);
                    self.calculateZ(value);
                    self.memoryManager.writeLast(value);
                },
                // Illegal opcode 0xFF: ISC aaaa,X
                else => {
                    self.last_cycles += 1;
                    result = RunResult.ILLEGAL_INSTRUCTION;
                },
            }

            self.cycles +%= self.last_cycles; // adjust total cycle spent
            if (result != RunResult.STEP) return result;
            if (self.single_step) return RunResult.STEP;
        }
        unreachable; // while(true)
    }
};

test "T.cpuRunSingleStep" {
    var memoryManager = Memory.MemoryManager.init();
    var page0 = [_]byte{0} ** Memory.PAGE_SIZE;
    var descriptor0 = Memory.PageRAM.init(&page0); // zero page
    memoryManager.setPageDescriptor(0, &descriptor0.descriptor);
    var page1 = [_]byte{0} ** Memory.PAGE_SIZE;
    var descriptor1 = Memory.PageRAM.init(&page1); // stack page
    memoryManager.setPageDescriptor(1, &descriptor1.descriptor);

    var cpu = CPU6502{};
    cpu.init(&memoryManager);
    cpu.single_step = true; // execute only 1 instruction

    memoryManager.writeZero(0, 0xA9); // LDA #
    memoryManager.writeZero(1, 0xFF);
    cpu.PC = 0; // run from (zero-page) address 0
    _ = cpu.run();

    // Check a not initialized page
    try std.testing.expect(cpu.A == 0xFF);
    try std.testing.expect(cpu.PS.N); // 0xFF is a 2's-complement negative number
    try std.testing.expect(cpu.PC == 2);
}
