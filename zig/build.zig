const std = @import("std");
const Builder = @import("std").build.Builder;
const builtin = @import("builtin");

pub const target = std.zig.CrossTarget{
    .cpu_arch = .thumb,
    .os_tag = .freestanding,
    .abi = .gnueabihf,
    .cpu_model = std.zig.CrossTarget.CpuModel{ .explicit = &std.Target.arm.cpu.cortex_m4 },
};

pub fn build(b: *Builder) void {
    const mode = b.standardReleaseOptions();
    const lib = b.addStaticLibrary("zigshell", "src/shell.zig");
    lib.single_threaded = true;
    lib.addIncludeDir("/usr/arm-none-eabi/include");
    lib.addIncludeDir("/github/workspace/nRF5_SDK_17/modules/nrfx/drivers/include");
    lib.addIncludeDir("/github/workspace/nRF5_SDK_17/modules/nrfx");
    lib.addIncludeDir("/github/workspace/nRF5_SDK_17/modules/nrfx/mdk");
    lib.addIncludeDir("/github/workspace/nRF5_SDK_17/integration/nrfx");
    lib.addIncludeDir("/github/workspace/modules/nrfsdk");
    lib.addIncludeDir("/github/workspace/modules/ledctrl");
    lib.addIncludeDir("/github/workspace/nRF5_SDK_17/components/toolchain/cmsis/include");
    lib.addIncludeDir("/github/workspace/nRF5_SDK_17/components/libraries/util");
    lib.addIncludeDir("/github/workspace/nRF5_SDK_17/components/softdevice/s140/headers");
    lib.addIncludeDir("src");
    lib.addLibPath("/github/workspace/_build/modules/nrfsdk");
    lib.linkSystemLibraryName("libnrfx_sdk");
    lib.setTarget(target);
    lib.setBuildMode(mode);
    lib.install();

    var shell_tests = b.addTest("src/shell.zig");
    shell_tests.setBuildMode(mode);

    const test_step = b.step("test", "Run library tests");
    test_step.dependOn(&shell_tests.step);
}
