const std = @import("std");
const fs = std.fs;

pub fn main() !void {
    // From documentation; create allocator which frees everything in the end.
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    var path: [1024]u8;
    try _ = fs.cwd().realpath(".", path);
    std.debug.print("File [{s}]\n", .{path});

    const options = fs.Dir.OpenDirOptions{
        .iterate = true,
    };
    var dir = try fs.cwd().openDir("../resources/ProcessorTests", options);
    defer dir.close();
    var iter = dir.iterate();
    while (try iter.next()) |entry| {
        // std.debug.print("File {s} ({})\n", .{ entry.name, entry.kind });
        if (entry.kind == fs.File.Kind.file) {
            const filename = try std.fmt.allocPrint(allocator, "..\\resources\\ProcessorTests\\{s}", .{entry.name});
            defer allocator.free(filename);
            std.debug.print("File [{s}]\n", .{filename});

            var json_file = try dir.openFile(filename, .{});
            defer json_file.close();
            const json_text = try json_file.readToEndAlloc(allocator, 10 * 10124);
            defer allocator.free(json_text);

            var lines = std.mem.split(u8, json_text, "\n");
            var count: usize = 0;
            while (lines.next()) |line| : (count += 1) {
                std.debug.print("{d:>2}: {s}\n", .{ count, line });
            }
            break;
        }
    }
}
