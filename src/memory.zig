const std = @import("std");

pub const byte = u8;
pub const word = u16;

const UNKNOWN_MEMORY_DATA: byte = 0xAA; // standard dummy data
const ADDRESS_SIZE = 16; // 16 bits = 64KB address space
const MEMORY_SIZE = 1 << ADDRESS_SIZE; // 64KB memory
const PAGE_SIZE = 8; // 8 bits = 256B page size
const PAGES = 1 << PAGE_SIZE; // 8 bits = 256 pages

pub const MemoryReader = *const fn (memory: *Memory, address: word) byte;
pub const MemoryWriter = *const fn (memory: *Memory, address: word, data: byte) void;

pub const PageDescriptor = struct {
    reader: MemoryReader,
    writer: ?MemoryWriter,
};

pub const Memory = struct {
    bytes: [MEMORY_SIZE]byte = [_]byte{0} ** MEMORY_SIZE,
    pages: [PAGES]?PageDescriptor = [_]?PageDescriptor{null} ** PAGES,
    addressBus: word = 0, // last used (read or written) address
    dataBus: byte = 0, // last used (read or written) data

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
        self.setPage(startPage, endPage, standardReader, standardWriter);
    }

    pub fn setPageRO(self: *Memory, startPage: byte, endPage: byte) void {
        self.setPage(startPage, endPage, standardReader, null);
    }

    /// Standard memory reader
    fn standardReader(self: *Memory, address: word) byte {
        return self.bytes[address];
    }

    /// Get a data from the zero page; no page checking is done (R/W is assumed) and the page reader is not used.
    pub fn readZero(self: *Memory, addressZ: byte) byte {
        self.addressBus = addressZ;
        self.dataBus = self.bytes[addressZ];
        return self.dataBus;
    }

    /// Get a data from the stack page; no page checking is done (R/W is assumed) and the page reader is not used.
    pub fn readStack(self: *Memory, addressSP: byte) byte {
        self.addressBus = 0x0100 | @as(word, addressSP);
        self.dataBus = self.bytes[self.addressBus];
        return self.dataBus;
    }

    /// Read a memory location using the page descriptor reader
    /// Return UNKNOWN_MEMORY_DATA if reading is not allowed
    pub fn read(self: *Memory, address: word) byte {
        self.addressBus = address;
        if (self.getPageDescriptor(address)) |descriptor| {
            self.dataBus = descriptor.reader(self, address);
        } else {
            self.dataBus = UNKNOWN_MEMORY_DATA;
        }
        return self.dataBus;
    }

    /// Standard memory writer
    fn standardWriter(self: *Memory, address: word, data: byte) void {
        self.bytes[address] = data;
    }

    /// Set a data in the zero page; no page checking is done (R/W is assumed) and the page writer is not used
    pub fn writeZero(self: *Memory, addressZ: byte, data: byte) void {
        self.addressBus = addressZ;
        self.dataBus = data;
        self.bytes[addressZ] = data;
    }

    /// Set a data in the stack page; no page checking is done (R/W is assumed) and the page writer is not used
    pub fn writeStack(self: *Memory, addressSP: byte, data: byte) void {
        self.addressBus = 0x0100 | @as(word, addressSP);
        self.dataBus = data;
        self.bytes[self.addressBus] = data;
    }

    /// Set a data on the address, if writing is enabled, using the page descriptor writer
    /// Ignore writing to non-writable memory
    pub fn write(self: *Memory, address: word, data: byte) void {
        self.addressBus = address;
        self.dataBus = data;
        if (self.getPageDescriptor(address)) |descriptor| {
            if (descriptor.writer) |writer| {
                writer(self, address, data);
            }
        }
    }

    /// Set a data on the last read address, if writing is enabled, using the page descriptor writer
    /// Ignore writing to non-writable memory
    pub fn writeLast(self: *Memory, data: byte) void {
        self.dataBus = data;
        if (self.getPageDescriptor(self.addressBus)) |descriptor| {
            if (descriptor.writer) |writer| {
                writer(self, self.addressBus, data);
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
    try std.testing.expect(memory.read(0) == UNKNOWN_MEMORY_DATA); // should default (no reader set)
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
