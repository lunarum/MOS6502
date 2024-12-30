# MOS 6502 CPU emulator

An MOS 6502 emulator written in Zig. My [C version 6502 emulator](https://github.com/lunarum/VIC20-Emulator/tree/master/Emulate6502) is used as a basis to (re)write the Zig version.

Processor tests are from [Tom Harte's GitHub repo](https://github.com/SingleStepTests/ProcessorTests/tree/main/6502/v1)

Currently Zig 0.13 is used with VS Code with the Zig extension as IDE.

## Adding this library to your Zig project

Add the following to your .dependencies in your _build.zig.zon_ file:

``` zig
    .{
        ...
        .dependencies {
            ...
            .mos6502 = .{
                .url = "https://github.com/lunarum/MOS6502/archive/refs/tags/v1.1.0.tar.gz",
                .hash = "12207ba5ad8c613785826135f5fa5bb80f5966faa1c49562259c8d2529afd7ef595d",
            },
        },
        ...
    }
```

Change your _build.zig_ file by adding the following mos6502 lines (find the const exe lines, insert the mos6502 lines after that and before the b.installArtifact() line, where b is your *std.Build parameter):

``` zig
    ...
    const exe = b.addExecutable(.{
        ...
    });

    const mos6502 = b.dependency("mos6502", .{
        .target = target,
        .optimize = optimize,
    });
    exe.root_module.addImport("mos6502", mos6502.module("mos6502"));
    exe.linkLibrary(mos6502.artifact("mos6502"));

    b.installArtifact(exe);
```

Now add to following to your code to access the CPU6502 and Memory structs:

``` zig
const mos6502 = @import("mos6502");

const CPU6502 = mos6502.CPU6502;
const byte = mos6502.byte;
const word = mos6502.word;
const MemoryManager = mos6502.Memory.MemoryManager;
```

Creating a system with 64KB RAM memory and writing / reading (see also the tests in Memory.zig):

``` zig
    var memory_pages: [PAGES][PAGE_SIZE]byte = .{.{0} ** PAGE_SIZE} ** PAGES; // 64KB memory
    var ram_pages: [PAGES]PageRAM = undefined; // 256 PageRAM descriptors
    var memory = MemoryManager.init();
    for (0..PAGES) |page| {
        ram_pages[page] = PageRAM.init(&memory_pages[page]); // create and init a PageRAM descriptor and associate it with a memory page
        memory.setPageDescriptor(@truncate(page), &ram_pages[page].descriptor); // make page descriptor known to the MemoryManager
        for (0..PAGE_SIZE) |index| {
            const address: word = @as(word, @truncate(page)) << 8 | @as(word, @truncate(index));
            const value: byte = @as(byte, @truncate(index));
            memory.write(address, value);
            _ = memory.read(address);
        }
    }
```
