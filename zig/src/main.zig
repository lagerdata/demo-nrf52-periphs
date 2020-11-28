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
    @cInclude("/github/workspace/modules/imu/mpu9250.h");
});

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

// var g_uart0: c.nrfx_uart_t = c.NRFX_UART_INSTANCE(0);

// const c_shell_context = "SHELL";
// var g_data_buf: u8[32];
// const c_menu = "****************************************\r\n" ++ "*                                      *\r\n" ++ "*   Welcome to Lager's Peripheral Demo *\r\n" ++ "*--------------------------------------*\r\n" ++ "*                                      *\r\n" ++ "*                                      *\r\n" ++ "* 'h' - Print \"Hello World\"            *\r\n" ++ "* 'i' - Stream IMU Output Float        *\r\n" ++ "* 'r' - Stream IMU Output Raw          *\r\n" ++ "* 's' - Print a sum                    *\r\n" ++ "* 'l' - Turn On Led                    *\r\n" ++ "* 'k' - Turn Off Led                   *\r\n" ++ "* 't' - Toggle Led                     *\r\n" ++ "*                                      *\r\n" ++ "*                                      *\r\n" ++ "****************************************\r\n";

// var g_polling: nrfx_timer_t = c.NRFX_TIMER_INSTANCE(2);
// var g_streaming_imu = false;
// var g_raw = false;

// extern fn handle_rx_bytes([*c]const c.nrfx_uart_xfer_evt_t) void;

// export fn uart_event_handler(p_event: [*c]const c.nrfx_uart_event_t, p_context: ?*c_void) void {
//     switch (p_event.*.type) {
//         c.nrfx_uart_evt_type_t.NRFX_UART_EVT_RX_DONE => {
//             handle_rx_bytes(&(p_event.*.data.rxtx));
//             c.nrfx_uart_rx(&g_uart0, &g_data_buf[0], 1);
//         },
//         else => return,
//     }
// }

test "basic add functionality" {
    testing.expect(add(3, 7) == 10);
}
