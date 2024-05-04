const std = @import("std");
const Memory = @import("memory.zig");
const CPU = @import("cpu.zig");

const JSon6502Status = struct {
    pc: u16,
    s: u8,
    a: u8,
    x: u8,
    y: u8,
    p: u8,
    ram: []struct {
        u16,
        u8,
    },
};
const JSonTest6502 = struct { name: []u8, initial: JSon6502Status, final: JSon6502Status, cycles: []struct {
    u16,
    u8,
    []u8,
} };
const JSonTest6502s = []JSonTest6502;

pub fn main() !void {
    // From documentation; create allocator which frees everything in the end.
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    // var path: [std.fs.MAX_PATH_BYTES]u8 = undefined;
    // const cwd = try std.fs.realpath(".", &path);
    // std.debug.print("CWD [{s}]\n", .{cwd});

    var dir = try std.fs.cwd().openDir("./resources/ProcessorTests", std.fs.Dir.OpenDirOptions{ .iterate = true });
    defer dir.close();

    try dir.setAsCwd();
    // const cwd2 = try std.fs.realpath(".", &path);
    // std.debug.print("CWD [{s}]\n", .{cwd2});

    var iter = dir.iterate();
    var errors: u32 = 0;
    var do_test = false;
    while (try iter.next()) |entry| {
        if (entry.kind == std.fs.File.Kind.file) {
            if (do_test or std.mem.eql(u8, entry.name, "40.json")) {
                do_test = true;
                std.debug.print("\n******** File [{s}] ", .{entry.name});

                var json_file = try dir.openFile(entry.name, .{});
                defer json_file.close();

                const json_text = try json_file.readToEndAlloc(allocator, 10 * 1024 * 1024);
                defer allocator.free(json_text);

                errors += parseJSon(allocator, json_text) catch |err| {
                    if (err == error.InvalidItem)
                        std.debug.print("{s} ", .{entry.name});
                    continue;
                };
                if (errors >= 8) {
                    // the whole implementation propably has some serious problems; no need to test any further
                    std.debug.print("\n******** To many errors, aborting ********\n", .{});
                    break;
                }
            }
        }
    }
}

fn printFlags(flags: CPU.Flags) void {
    const f: Memory.byte = @bitCast(flags);
    std.debug.print("{x:2}:", .{f});
    if (flags.PS_C) std.debug.print("C", .{}) else std.debug.print("c", .{}); // Carry
    if (flags.PS_Z) std.debug.print("Z", .{}) else std.debug.print("z", .{}); // Zero
    if (flags.PS_I) std.debug.print("I", .{}) else std.debug.print("i", .{}); // IRQ disable
    if (flags.PS_D) std.debug.print("D", .{}) else std.debug.print("d", .{}); // Decimal mode
    if (flags.PS_B) std.debug.print("B", .{}) else std.debug.print("b", .{}); // (in) Break command
    if (flags.PS_1) std.debug.print("1", .{}) else std.debug.print("0", .{}); // not used
    if (flags.PS_V) std.debug.print("V", .{}) else std.debug.print("v", .{}); // oVerflow
    if (flags.PS_N) std.debug.print("N", .{}) else std.debug.print("n", .{}); // Negative
}

fn parseJSon(allocator: std.mem.Allocator, json_text: []u8) !u32 {
    // const dummy_text =
    //     \\ [
    //     \\  {
    //     \\      "name": "00 3f f7",
    //     \\      "initial": { "pc": 35714, "s": 81, "a": 203, "x": 117, "y": 162, "p": 106,
    //     \\         "ram": [ [35714, 0], [35715, 63], [35716, 247], [65534, 212], [65535, 37], [9684, 237]]},
    //     \\      "final": { "pc": 9684, "s": 78, "a": 203, "x": 117, "y": 162, "p": 110,
    //     \\         "ram": [ [335, 122], [336, 132], [337, 139], [9684, 237], [35714, 0], [35715, 63], [35716, 247], [65534, 212], [65535, 37]]},
    //     \\      "cycles": [ [35714, 0, "read"], [35715, 63, "read"], [337, 139, "write"], [336, 132, "write"], [335, 122, "write"], [65534, 212, "read"], [65535, 37, "read"]]
    //     \\  }
    //     \\ ]
    //     \\
    // ;
    // _ = json_text;

    const parsed = try std.json.parseFromSlice(JSonTest6502s, allocator, json_text, .{});
    defer parsed.deinit();

    var memory6502 = Memory.Memory{};
    memory6502.setPageRW(0, 0); // 64K of writable memory

    var cpu6502 = CPU.CPU6502{};
    cpu6502.init(&memory6502);
    cpu6502.single_step = true; // execute only 1 instruction

    var errors: u32 = 0;
    const entries = parsed.value;
    for (entries) |entry| {
        const ok = try executeTest(&cpu6502, entry);
        if (!ok) {
            std.debug.print("\nERROR in test {s:8}: PC{x:4},SP{x:2},A{x:2},X{x:2},Y{x:2},", .{
                entry.name,
                entry.initial.pc,
                entry.initial.s,
                entry.initial.a,
                entry.initial.x,
                entry.initial.y,
            });
            printFlags(@bitCast(entry.initial.p));
            std.debug.print(" => PC{x:4},SP{x:2},A{x:2},X{x:2},Y{x:2},", .{
                entry.final.pc,
                entry.final.s,
                entry.final.a,
                entry.final.x,
                entry.final.y,
            });
            printFlags(@bitCast(entry.final.p));
            std.debug.print(" != PC{x:4},SP{x:2},A{x:2},X{x:2},Y{x:2},", .{
                cpu6502.PC,
                cpu6502.SP,
                cpu6502.A,
                cpu6502.X,
                cpu6502.Y,
            });
            printFlags(cpu6502.PS);
            std.debug.print("\nMEMORY ", .{});
            for (entry.final.ram) |ram_entry| {
                std.debug.print("{x:4}:{x:2} ", .{
                    ram_entry.@"0",
                    ram_entry.@"1",
                });
                const b = cpu6502.memoryManager.read(ram_entry.@"0");
                if (b != ram_entry.@"1") {
                    std.debug.print("!= {x:2} ", .{b});
                }
            }
            errors += 1;
            if (errors >= 8) {
                // implementation is propably wrong; no need to test any further
                std.debug.print("\n******** To many errors in this opcode, skipping the rest ********", .{});
                break;
            }
        }
    }
    return errors;
}

fn executeTest(cpu6502: *CPU.CPU6502, entry: JSonTest6502) !bool {
    cpu6502.PC = entry.initial.pc;
    cpu6502.SP = entry.initial.s;
    cpu6502.A = entry.initial.a;
    cpu6502.X = entry.initial.x;
    cpu6502.Y = entry.initial.y;
    cpu6502.PS = @bitCast(entry.initial.p);
    for (entry.initial.ram) |ram_entry| {
        cpu6502.memoryManager.write(ram_entry.@"0", ram_entry.@"1");
    }

    cpu6502.single_step = true;
    if (cpu6502.run() == CPU.RunResult.RESULT_ILLEGAL_INSTUCTION) {
        return error.InvalidItem;
    }

    for (entry.final.ram) |ram_entry| {
        if (cpu6502.memoryManager.read(ram_entry.@"0") != ram_entry.@"1") {
            return false;
        }
    }
    const flags: u8 = @bitCast(cpu6502.PS);
    return cpu6502.PC == entry.final.pc and
        cpu6502.SP == entry.final.s and
        cpu6502.A == entry.final.a and
        cpu6502.X == entry.final.x and
        cpu6502.Y == entry.final.y and
        flags == entry.final.p;
}
