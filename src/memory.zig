const std = @import("std");

pub const byte = u8;
pub const word = u16;

pub const UNKNOWN_MEMORY_DATA: byte = 0xAA; // standard dummy data
pub const MEMORY_SIZE = 65536; // 16 bits = 64KB address space
pub const PAGE_SIZE = 256; // 8 bits = 256 bytes page size
pub const PAGES = 256; // 8 bits = 256 pages

/// Page descriptor interface, which defines a read() and write() functions for a (and including a memory) page.
/// Analog to interface definition Shape4 from: https://zig.news/yglcode/code-study-interface-idiomspatterns-in-zig-standard-libraries-4lkj
const PageDescriptor = struct {
    reader: *const fn (ptr: *PageDescriptor, address: byte) byte,
    writer: *const fn (ptr: *PageDescriptor, address: byte, value: byte) void,

    pub fn read(self: *PageDescriptor, address: byte) byte {
        return self.reader(self, address);
    }

    pub fn write(self: *PageDescriptor, address: byte, value: byte) void {
        self.writer(self, address, value);
    }
};

/// Page descriptor for an empty page; always returns UNKNOWN_MEMORY_DATA on reads and ignores writes.
/// Analog to interface implementation Box4 from: https://zig.news/yglcode/code-study-interface-idiomspatterns-in-zig-standard-libraries-4lkj
const PageEmpty = struct {
    descriptor: PageDescriptor,

    pub fn init() PageEmpty {
        return .{ .descriptor = .{ .reader = read, .writer = write } };
    }

    pub fn read(page_descriptor: *PageDescriptor, address: byte) byte {
        _ = page_descriptor;
        _ = address;
        return UNKNOWN_MEMORY_DATA;
    }

    pub fn write(page_descriptor: *PageDescriptor, address: byte, value: byte) void {
        _ = page_descriptor;
        _ = address;
        _ = value;
        // do nothing
    }
};

/// Standard (RAM) page descriptor containing a reference to a memory page as data.
/// Analog to interface implementation Box4 from: https://zig.news/yglcode/code-study-interface-idiomspatterns-in-zig-standard-libraries-4lkj
pub const PageRAM = struct {
    page: *[PAGE_SIZE]byte,
    descriptor: PageDescriptor,

    pub fn init(ram_page: *[PAGE_SIZE]byte) PageRAM {
        const page_ram = .{ .page = ram_page, .descriptor = .{ .reader = read, .writer = write } };
        return page_ram;
    }

    pub fn read(page_descriptor: *PageDescriptor, address: byte) byte {
        const self = @as(*PageRAM, @fieldParentPtr("descriptor", page_descriptor));
        return self.page[address];
    }

    pub fn write(page_descriptor: *PageDescriptor, address: byte, value: byte) void {
        const self = @as(*PageRAM, @fieldParentPtr("descriptor", page_descriptor));
        self.page[address] = value;
    }
};

/// Read only memory page containing a reference to a ROM page as data.
/// Analog to interface implementation Box4 from: https://zig.news/yglcode/code-study-interface-idiomspatterns-in-zig-standard-libraries-4lkj
pub const PageROM = struct {
    page: *[PAGE_SIZE]byte,
    descriptor: PageDescriptor,

    pub fn init(rom_page: *[PAGE_SIZE]byte) PageROM {
        const page_rom = .{ .page = rom_page, .descriptor = .{ .reader = read, .writer = write } };
        return page_rom;
    }

    pub fn read(page_descriptor: *PageDescriptor, address: byte) byte {
        const self = @as(*PageROM, @fieldParentPtr("descriptor", page_descriptor));
        return self.page[address];
    }

    pub fn write(page_descriptor: *PageDescriptor, address: byte, value: byte) void {
        _ = page_descriptor;
        _ = address;
        _ = value;
        // do nothing
    }
};

var page_empty: PageEmpty = PageEmpty.init();

pub const MemoryManager = struct {
    page_descriptors: [PAGES](*PageDescriptor) = [_](*PageDescriptor){&page_empty.descriptor} ** PAGES,
    address_bus: word, // last used (read or written) address
    data_bus: byte, // last used (read or written) data

    pub fn init() MemoryManager {
        var page = PageEmpty.init();
        const memory: MemoryManager = .{
            .page_descriptors = [_]*PageDescriptor{&page.descriptor} ** PAGES,
            .address_bus = 0,
            .data_bus = 0,
        };
        return memory;
    }

    inline fn getPageDescriptor(self: *MemoryManager, address: word) *PageDescriptor {
        const page = address >> 8;
        return self.page_descriptors[page];
    }

    /// Set the page descriptor for the given page.
    /// Param page is the page number.
    /// Param descriptor must implement the PageDescriptor interface.
    pub fn setPageDescriptor(self: *MemoryManager, page: byte, descriptor: *PageDescriptor) void {
        self.page_descriptors[page] = descriptor;
    }

    /// Read data from the zero page using the pages[0] descriptor reader
    pub inline fn readZero(self: *MemoryManager, address_zero: byte) byte {
        self.address_bus = @as(word, address_zero);
        const descriptor = self.page_descriptors[0];
        self.data_bus = descriptor.read(address_zero);
        return self.data_bus;
    }

    /// Read data from the stack page using the pages[1] descriptor reader
    pub fn readStack(self: *MemoryManager, address_stack: byte) byte {
        self.address_bus = 0x0100 | @as(word, address_stack);
        const descriptor = self.page_descriptors[1];
        self.data_bus = descriptor.read(address_stack);
        return self.data_bus;
    }

    /// Read data from a memory page using the appropriate page descriptor reader
    pub fn read(self: *MemoryManager, address: word) byte {
        self.address_bus = address;
        const descriptor = self.getPageDescriptor(address);
        self.data_bus = descriptor.read(@truncate(address & 0xFF));
        return self.data_bus;
    }

    /// Set data on a memory page, using the pages[0] descriptor writer
    pub fn writeZero(self: *MemoryManager, address_zero: byte, data: byte) void {
        self.address_bus = @as(word, address_zero);
        self.data_bus = data;
        const descriptor = self.page_descriptors[0];
        descriptor.write(address_zero, data);
    }

    /// Set data on a memory page, using the pages[1] descriptor writer
    pub fn writeStack(self: *MemoryManager, address_stack: byte, data: byte) void {
        self.address_bus = 0x0100 | @as(word, address_stack);
        self.data_bus = data;
        const descriptor = self.page_descriptors[1];
        descriptor.write(address_stack, data);
    }

    /// Set data on a memory page, using the appropriate page descriptor writer
    pub fn write(self: *MemoryManager, address: word, data: byte) void {
        self.address_bus = address;
        self.data_bus = data;
        const descriptor = self.getPageDescriptor(address);
        descriptor.write(@truncate(address & 0xFF), data);
    }

    /// Set a data on the last used memory address, using the appropriate page descriptor writer
    pub fn writeLast(self: *MemoryManager, data: byte) void {
        self.data_bus = data;
        const descriptor = self.getPageDescriptor(self.address_bus);
        descriptor.write(@truncate(self.address_bus & 0xFF), data);
    }
};

test "T.memoryUninitializedWriteRead" {
    var memory = MemoryManager.init();
    try std.testing.expect(memory.read(0) == UNKNOWN_MEMORY_DATA);
    memory.write(0, 0);
    try std.testing.expect(memory.read(0) == UNKNOWN_MEMORY_DATA); // No change
}

test "T.memoryZeroWriteRead" {
    var memory = MemoryManager.init();
    var zero_page = [_]byte{0} ** PAGE_SIZE;
    var zero_descriptor = PageRAM.init(&zero_page);
    memory.setPageDescriptor(0, &zero_descriptor.descriptor);
    try std.testing.expect(zero_descriptor.page == &zero_page);
    memory.writeZero(0, 0x55);
    try std.testing.expect(zero_page[0] == 0x55);
    try std.testing.expect(memory.readZero(0) == 0x55);
}

test "T.memoryStackWriteRead" {
    var memory = MemoryManager.init();
    var stack_page = [_]byte{0} ** PAGE_SIZE;
    var stack_descriptor = PageRAM.init(&stack_page);
    memory.setPageDescriptor(1, &stack_descriptor.descriptor);
    try std.testing.expect(stack_descriptor.page == &stack_page);
    memory.writeStack(0, 0x55);
    try std.testing.expect(stack_page[0] == 0x55);
    try std.testing.expect(memory.readStack(0) == 0x55);
}

test "T.memoryWriteRead" {
    var memory_pages: [PAGES][PAGE_SIZE]byte = .{.{0} ** PAGE_SIZE} ** PAGES;
    var memory = MemoryManager.init();
    for (0..PAGES) |page| {
        var page_descriptor = PageRAM.init(&memory_pages[page]);
        memory.setPageDescriptor(@truncate(page), &page_descriptor.descriptor);
        for (0..PAGE_SIZE) |index| {
            const address: word = @as(word, @truncate(page)) << 8 | @as(word, @truncate(index));
            const value: byte = @as(byte, @truncate(index));
            try std.testing.expect(memory.read(address) == 0);
            memory.write(address, value);
            try std.testing.expect(memory.read(address) == value);
        }
    }
}

test "T.memoryDescriptorReadOnly" {
    var memory_pages: [PAGES][PAGE_SIZE]byte = .{.{0} ** PAGE_SIZE} ** PAGES;
    var memory = MemoryManager.init();
    for (0..PAGES) |page| {
        var page_descriptor = PageROM.init(&memory_pages[page]);
        memory.setPageDescriptor(@truncate(page), &page_descriptor.descriptor);
        for (0..PAGE_SIZE) |index| {
            const address: word = @as(word, @truncate(page)) << 8 | @as(word, @truncate(index));
            const value: byte = @as(byte, @truncate(index));
            try std.testing.expect(memory.read(address) == 0);
            memory.write(address, value);
            try std.testing.expect(memory.read(address) == 0); // No change
        }
    }
}
