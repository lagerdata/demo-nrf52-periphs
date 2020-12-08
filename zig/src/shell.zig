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

threadlocal var g_uart0 = c.nrfx_uart_t{
    .p_reg = c.NRF_UART0,
    .drv_inst_idx = c.NRFX_UART0_INST_IDX,
};

threadlocal var g_polling = c.nrfx_timer_t{
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

var c_shell_context = "SHELL";
threadlocal var g_data_buf = [_]u8{0} ** 32;
var g_streaming_imu = false;
var g_raw = false;

export fn uart_event_handler(p_event: [*c]const c.nrfx_uart_event_t, p_context: ?*c_void) void {
    switch (p_event.*.type) {
        c.nrfx_uart_evt_type_t.NRFX_UART_EVT_RX_DONE => {
            handle_rx_bytes(&(p_event.*.data.rxtx));
            const result = c.nrfx_uart_rx(&g_uart0, &g_data_buf, 1);
        },
        else => return,
    }
}

fn handle_rx_bytes(p_rxtx: *const c.nrfx_uart_xfer_evt_t) void {
    switch (p_rxtx.p_data) {
        'h' => {
            const hello = "Hello World\r\n";
            _ = c.nrfx_uart_tx(&g_uart0, hello, hello.len);
            return;
        },
        else => return,
    }
}

// switch(*p_rxtx->p_data){
//     case 'h':{
//         nrfx_uart_tx(&g_uart0, (uint8_t const *)"Hello World\r\n", sizeof("Hello World\r\n"));
//         break;
//     }
//     case 's': {
//         int32_t sum;
//         sum = add(40, 2);
//         char buf[32];
//         sprintf((char *)buf, "Sum: %ld\r\n", sum);
//         nrfx_uart_tx(&g_uart0, (uint8_t const *)buf, strlen(buf) + 1);
//         break;
//     }
//     case 'i':{
//         if(true == g_streaming_imu){
//             g_streaming_imu = false;
//         }else{
//             trigger_imu_stream();
//             g_streaming_imu = true;
//             g_raw = false;
//         }
//         break;
//     }

//     case 'r':{
//         if(true == g_streaming_imu){
//             g_streaming_imu = false;
//         }else{
//             trigger_imu_stream();
//             g_streaming_imu = true;
//             g_raw = true;
//         }
//         break;
//     }

//     case 'l':{
//         const char led_on_msg[] = "Turning on LED 0.\r\n";
//         nrfx_uart_tx(&g_uart0, (uint8_t const *)led_on_msg, sizeof(led_on_msg));
//         ledctrl_onoff(true, 0);

//         break;
//     }

//     case 'k':{
//         const char led_on_msg[] = "Turning off LED 0.\r\n";
//         nrfx_uart_tx(&g_uart0, (uint8_t const *)led_on_msg, sizeof(led_on_msg));
//         ledctrl_onoff(false, 0);

//         break;
//     }

//     case 't':{
//         if(false ==ledctrl_is_led_num_on(0)){
//             const char led_on_msg[] = "Turning on LED 0.\r\n";
//             nrfx_uart_tx(&g_uart0, (uint8_t const *)led_on_msg, sizeof(led_on_msg));
//             ledctrl_onoff(true, 0);
//         }else{
//             const char led_off_msg[] = "Turning off LED 0.\r\n";
//             nrfx_uart_tx(&g_uart0, (uint8_t const *)led_off_msg, sizeof(led_off_msg));
//             ledctrl_onoff(false, 0);
//         }
//         break;
//     }

//     default:{
//         nrfx_uart_tx(&g_uart0, (uint8_t const *)c_menu, sizeof(c_menu));
//         break;
//     }
// }

export fn shell_init() void {
    const uart_cfg = c.nrfx_uart_config_t{
        .pseltxd = 6,
        .pselrxd = 8,
        .pselcts = c.NRF_UART_PSEL_DISCONNECTED,
        .pselrts = c.NRF_UART_PSEL_DISCONNECTED,
        .p_context = @ptrCast(*c_void, &c_shell_context),
        .hwfc = c.nrf_uart_hwfc_t.NRF_UART_HWFC_ENABLED,
        .parity = c.nrf_uart_parity_t.NRF_UART_PARITY_EXCLUDED,
        .baudrate = c.nrf_uart_baudrate_t.NRF_UART_BAUDRATE_115200,
        .interrupt_priority = 7,
    };
    _ = c.nrfx_uart_init(&g_uart0, &uart_cfg, uart_event_handler);
    _ = c.nrfx_uart_rx_enable(&g_uart0);
    _ = c.nrfx_uart_rx(&g_uart0, &g_data_buf[0], 1);

    const cfg = c.nrfx_timer_config_t{
        .frequency = c.nrf_timer_frequency_t.NRF_TIMER_FREQ_16MHz,
        .mode = c.nrf_timer_mode_t.NRF_TIMER_MODE_TIMER,
        .bit_width = c.nrf_timer_bit_width_t.NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = 6,
        .p_context = null,
    };
    _ = c.nrfx_timer_init(&g_polling, &cfg, polling_timer_event_handler);
    const cc_time: u32 = c.nrfx_timer_ms_to_ticks(&g_polling, 10);
    _ = c.nrfx_timer_extended_compare(&g_polling, c.nrf_timer_cc_channel_t.NRF_TIMER_CC_CHANNEL0, cc_time, c.nrf_timer_short_mask_t.NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    _ = c.nrfx_timer_enable(&g_polling);
}

export fn polling_timer_event_handler(event_type: c.nrf_timer_event_t, p_context: ?*c_void) void {
    switch (event_type) {
        c.nrf_timer_event_t.NRF_TIMER_EVENT_COMPARE0 => {
            if (true == g_streaming_imu) { //check if we're streaming IMU data
                // uint8_t accel_buf[6];
                // mpu9250_drv_read_accel(&accel_buf);
                // if(true == g_raw){
                //     nrfx_uart_tx(&g_uart0, (uint8_t const *)accel_buf, sizeof(accel_buf));
                // }else{
                //     MPU9250_accel_val accel_val;
                //     mpu9250_drv_process_raw_accel(&accel_val, &accel_buf);
                //     char buf[64];
                //     size_t len = sprintf((char *)buf, "ACCEL(x,y,z):%f,%f,%f\r\n",accel_val.x, accel_val.y, accel_val.z);
                //     nrfx_uart_tx(&g_uart0, (uint8_t const *)buf, len);
                // }
            }
        },
        else => return,
    }
}
