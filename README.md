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
                .url = "https://github.com/lunarum/MOS6502/archive/refs/tags/v1.0.0.tar.gz",
                .hash = "12203372b7dde2fd6c52e706696ecb821daefe203c2d6cd1997c5aa8c62ff8f7e064",
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
const Memory = mos6502.Memory.Memory;
```
