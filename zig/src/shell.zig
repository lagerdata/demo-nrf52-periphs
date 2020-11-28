const std = @import("std");
const testing = std.testing;
const c = @cImport({
    // See https://github.com/ziglang/zig/issues/515
    //FLOAT_ABI_HARD NRF52840_XXAA
    @cDefine("FLOAT_ABI_HARD", {});
    @cDefine("NRF52840_XXAA", {});
    @cDefine("BOARD_PCA10056", {});
    // do stuff
    @cInclude("/github/workspace/nRF5_SDK_17/modules/nrfx/drivers/include/nrfx_uart.h");
    @cInclude("/github/workspace/nRF5_SDK_17/modules/nrfx/drivers/include/nrfx_timer.h");
    @cInclude("/github/workspace/modules/imu/mpu9250.h");
});

const g_uart0 = c.nrfx_uart_t {
    .p_reg        = c.NRF_UART0,
    .drv_inst_idx = c.NRFX_UART0_INST_IDX,
};

const g_polling = c.nrfx_timer_t {
    .p_reg = c.NRF_TIMER2,
    .instance_id = c.NRFX_TIMER2_INST_IDX,
    .cc_channel_count = c.TIMER2_CC_NUM,
};

// const c_menu = c.c_menu;

export fn add(a: i32, b: i32) i32 {
    return a + b;
}

const MPU9250_BIT_GYRO_FS_SEL_1000DPS = c.MPU9250_BIT_GYRO_FS_SEL.MPU9250_BIT_GYRO_FS_SEL_1000DPS;
const MPU9250_BIT_ACCEL_FS_SEL_8G = c.MPU9250_BIT_ACCEL_FS_SEL.MPU9250_BIT_ACCEL_FS_SEL_8G;
const MPU9250_BIT_DLPF_CFG_250HZ = c.MPU9250_BIT_DLPF_CFG.MPU9250_BIT_DLPF_CFG_250HZ;
const MPU9250_BIT_A_DLPFCFG_460HZ = c.MPU9250_BIT_A_DLPFCFG.MPU9250_BIT_A_DLPFCFG_460HZ;
export fn trigger_imu_stream() void {
    _ = c.mpu9250_start_measure(MPU9250_BIT_GYRO_FS_SEL_1000DPS, MPU9250_BIT_ACCEL_FS_SEL_8G, MPU9250_BIT_DLPF_CFG_250HZ, MPU9250_BIT_A_DLPFCFG_460HZ);
}


const c_shell_context = "SHELL";
threadlocal var g_data_buf = [_]u8{0} ** 32;
var g_streaming_imu = false;
var g_raw = false;

extern fn handle_rx_bytes([*c]const c.nrfx_uart_xfer_evt_t) void;

export fn uart_event_handler(p_event: [*c]const c.nrfx_uart_event_t, p_context: ?*c_void) void {
    switch (p_event.*.type) {
        c.nrfx_uart_evt_type_t.NRFX_UART_EVT_RX_DONE => {
            handle_rx_bytes(&(p_event.*.data.rxtx));
            const result = c.nrfx_uart_rx(&g_uart0, &g_data_buf, 1);
        },
        else => return,
    }
}

test "basic add functionality" {
    testing.expect(add(3, 7) == 10);
}
