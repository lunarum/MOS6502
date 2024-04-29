const std = @import("std");

pub const byte = u8;
pub const word = u16;

const UNKNOWN_MEMORY_VALUE: byte = 0xAA; // standard dummy value
const ADDRESS_SIZE = 16; // 16 bits = 64KB address space
const MEMORY_SIZE = 1 << ADDRESS_SIZE; // 64KB memory
const PAGE_SIZE = 8; // 8 bits = 256B page size
const PAGES = 1 << PAGE_SIZE; // 8 bits = 256 pages

pub const MemoryReader = *const fn (memory: *Memory, address: word) byte;
pub const MemoryWriter = *const fn (memory: *Memory, address: word, value: byte) void;

pub const PageDescriptor = struct {
    reader: MemoryReader,
    writer: ?MemoryWriter,
};

pub const Memory = struct {
    bytes: [MEMORY_SIZE]byte = [_]byte{0} ** MEMORY_SIZE,
    pages: [PAGES]?PageDescriptor = [_]?PageDescriptor{null} ** PAGES,
    last_read: word = 0, // last address read, except for zero page and stack page access

    inline fn getPageDescriptor(self: *Memory, address: word) ?PageDescriptor {
        const page = address >> PAGE_SIZE;
        return self.pages[page];
    }

    pub fn setPage(self: *Memory, startPage: byte, endPage: byte, memoryReader: MemoryReader, memoryWriter: ?MemoryWriter) void {
        if (endPage == startPage) {
            self.pages[startPage] = PageDescriptor{
                .reader = memoryReader,
                .writer = memoryWriter,
            };
        } else {
            for (startPage..if (endPage == 0) self.pages.len else endPage) |page| {
                self.pages[page] = PageDescriptor{
                    .reader = memoryReader,
                    .writer = memoryWriter,
                };
            }
        }
    }

    pub fn setPageRW(self: *Memory, startPage: byte, endPage: byte) void {
        setPage(self, startPage, endPage, standardReader, standardWriter);
    }

    pub fn setPageRO(self: *Memory, startPage: byte, endPage: byte) void {
        setPage(self, startPage, endPage, standardReader, null);
    }

    /// Standard memory reader
    fn standardReader(self: *Memory, address: word) byte {
        return self.bytes[address];
    }

    /// Get a value from the zero page; no page checking is done (R/W is assumed) and the page reader is not used.
    pub fn readZero(self: *Memory, addressZ: byte) byte {
        return self.bytes[addressZ];
    }

    /// Get a value from the stack page; no page checking is done (R/W is assumed) and the page reader is not used.
    pub fn readStack(self: *Memory, addressSP: byte) byte {
        return self.bytes[0x0100 | @as(word, addressSP)];
    }

    /// Read a memory location using the page descriptor reader
    /// Return UNKNOWN_MEMORY_VALUE if reading is not allowed
    pub fn read(self: *Memory, address: word) byte {
        self.last_read = address;
        if (self.getPageDescriptor(address)) |descriptor| {
            return descriptor.reader(self, address);
        }
        return UNKNOWN_MEMORY_VALUE;
    }

    /// Standard memory writer
    fn standardWriter(self: *Memory, address: word, value: byte) void {
        self.bytes[address] = value;
    }

    /// Set a value in the zero page; no page checking is done (R/W is assumed) and the page writer is not used
    pub fn writeZero(self: *Memory, addressZ: byte, value: byte) void {
        self.bytes[addressZ] = value;
    }

    /// Set a value in the stack page; no page checking is done (R/W is assumed) and the page writer is not used
    pub fn writeStack(self: *Memory, addressSP: byte, value: byte) void {
        self.bytes[0x0100 | @as(word, addressSP)] = value;
    }

    /// Set a value on the address, if writing is enabled, using the page descriptor writer
    /// Ignore writing to non-writable memory
    pub fn write(self: *Memory, address: word, value: byte) void {
        if (self.getPageDescriptor(address)) |descriptor| {
            if (descriptor.writer) |writer| {
                writer(self, address, value);
            }
        }
    }

    /// Set a value on the last read address, if writing is enabled, using the page descriptor writer
    /// Ignore writing to non-writable memory
    pub fn memorySetLast(self: *Memory, value: byte) void {
        if (self.getPageDescriptor(self.last_read)) |descriptor| {
            if (descriptor.writer) |writer| {
                writer(self, self.last_read, value);
            }
        }
    }
};

test "T.memoryWrite-Read" {
    var memory = Memory{};
    memory.writeZero(0, 0x55);
    try std.testing.expect(memory.readZero(0) == 0x55);
    memory.writeStack(0, 0x55);
    try std.testing.expect(memory.readStack(0) == 0x55);
    memory.write(0, 0x33); // should be ignored (no writer set)
    try std.testing.expect(memory.read(0) == UNKNOWN_MEMORY_VALUE); // should default (no reader set)
    try std.testing.expect(memory.readZero(0) == 0x55);
}

test "T.memorySetPage" {
    var memory = Memory{};

    memory.setPageRW(0, 3);
    memory.setPageRW(3, 3);
    memory.setPageRW(4, 4);
    for (0..5) |page| {
        try std.testing.expect(memory.pages[page] != null);
        if (memory.pages[page]) |descriptor| {
            try std.testing.expect(descriptor.reader == Memory.standardReader);
            try std.testing.expect(descriptor.writer == Memory.standardWriter);
        }
    }

    // Check a not initialised page
    try std.testing.expect(memory.pages[5] == null);

    memory.setPageRO(250, 0);
    for (250..256) |page| {
        if (memory.pages[page]) |descriptor| {
            try std.testing.expect(descriptor.reader == Memory.standardReader);
            try std.testing.expect(descriptor.writer == null);
        }
    }
}
