#define _SHELL_C_SRC

//-------------------------MODULES USED-------------------------------------
#include <string.h>
#include "nrfx_uart.h"
#include "nrfx_timer.h"
#include "ledctrl.h"
#include "mpu9250.h"

//-------------------------DEFINITIONS AND MACORS---------------------------



//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------
static void uart_event_handler(nrfx_uart_event_t const * p_event, void * p_context);
static void handle_rx_bytes(nrfx_uart_xfer_evt_t * p_rxtx);
static void polling_timer_event_handler(nrf_timer_event_t event_type, void * p_context);
static void trigger_imu_stream(void);
//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------
static nrfx_uart_t g_uart0 = NRFX_UART_INSTANCE(0);
const char c_shell_context[] = "SHELL";
static uint8_t g_data_buf[32] = {0};
const char c_menu[] = "\
****************************************\r\n\
*                                      *\r\n\
*   Welcome to Lager's Peripheral Demo *\r\n\
*--------------------------------------*\r\n\
*                                      *\r\n\
*                                      *\r\n\
* 'h' - Print \"Hello World\"            *\r\n\
* 'i' - Stream IMU Output Raw          *\r\n\
* 'l' - Turn On Led                    *\r\n\
* 'k' - Turn Off Led                   *\r\n\
* 't' - Toggle Led                     *\r\n\
*                                      *\r\n\
*                                      *\r\n\
****************************************\r\n";

static nrfx_timer_t g_polling = NRFX_TIMER_INSTANCE(2);
static bool g_streaming_imu = false;

//-------------------------EXPORTED FUNCTIONS-------------------------------
void shell_init(void)
{

    nrfx_uart_config_t uart_cfg = {
                                   .pseltxd = 6,
                                   .pselrxd = 8,
                                   .pselcts = NRF_UART_PSEL_DISCONNECTED,
                                   .pselrts = NRF_UART_PSEL_DISCONNECTED,
                                   .p_context = (void *)c_shell_context,
                                   .hwfc = NRF_UART_HWFC_ENABLED,
                                   .parity = NRF_UART_PARITY_EXCLUDED,
                                   .baudrate = NRF_UART_BAUDRATE_115200,
                                   .interrupt_priority = 3};
    nrfx_uart_init(&g_uart0, &uart_cfg, uart_event_handler);
    nrfx_uart_rx_enable(&g_uart0);
    nrfx_uart_rx(&g_uart0, &g_data_buf[0], 1);

    nrfx_timer_config_t cfg = {
                               .frequency = NRF_TIMER_FREQ_16MHz,
                               .mode = NRF_TIMER_MODE_TIMER,
                               .bit_width = NRF_TIMER_BIT_WIDTH_32,
                               .interrupt_priority = 6,
                               .p_context = NULL};
    nrfx_timer_init(&g_polling, &cfg, polling_timer_event_handler);
    uint32_t cc_time = nrfx_timer_ms_to_ticks(&g_polling, 100);
    nrfx_timer_extended_compare(&g_polling, NRF_TIMER_CC_CHANNEL0, cc_time, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrfx_timer_enable(&g_polling);


}


//-------------------------LOCAL FUNCTIONS----------------------------------
static void trigger_imu_stream(void)
{
    mpu9250_start_measure(MPU9250_BIT_GYRO_FS_SEL_1000DPS, MPU9250_BIT_ACCEL_FS_SEL_8G, MPU9250_BIT_DLPF_CFG_250HZ, MPU9250_BIT_A_DLPFCFG_460HZ);

}

static void uart_event_handler(nrfx_uart_event_t const * p_event, void * p_context)
{
    switch(p_event->type){
        case NRFX_UART_EVT_RX_DONE:{
            handle_rx_bytes((nrfx_uart_xfer_evt_t *)&p_event->data.rxtx);
            nrfx_uart_rx(&g_uart0, &g_data_buf[0], 1);
            break;
        }

        case NRFX_UART_EVT_TX_DONE:{
            break;
        }

        case NRFX_UART_EVT_ERROR:{
            break;
        }

        default:{
            break;
        }
    }
}

static void handle_rx_bytes(nrfx_uart_xfer_evt_t * p_rxtx)
{
    switch(*p_rxtx->p_data){
        case 'h':{
            nrfx_uart_tx(&g_uart0, (uint8_t const *)"Hello World\r\n", sizeof("Hello World\r\n"));
            break;
        }
        case 'i':{
            nrfx_uart_tx(&g_uart0, (uint8_t const *)"IMU streaming not implemented, could you please help?\r\n", sizeof("IMU streaming not implemented, could you please help?\r\n"));
            if(true == g_streaming_imu){
                g_streaming_imu = false;
            }else{
                trigger_imu_stream();
                g_streaming_imu = true;
            }
            break;
        }

        case 'l':{
            const char led_on_msg[] = "Turning on LED 0.\r\n";
            nrfx_uart_tx(&g_uart0, (uint8_t const *)led_on_msg, sizeof(led_on_msg));
            ledctrl_onoff(true, 0);

            break;
        }

        case 'k':{
            const char led_on_msg[] = "Turning off LED 0.\r\n";
            nrfx_uart_tx(&g_uart0, (uint8_t const *)led_on_msg, sizeof(led_on_msg));
            ledctrl_onoff(false, 0);

            break;
        }

        case 't':{
            if(false ==ledctrl_is_led_num_on(0)){
                const char led_on_msg[] = "Turning on LED 0.\r\n";
                nrfx_uart_tx(&g_uart0, (uint8_t const *)led_on_msg, sizeof(led_on_msg));
                ledctrl_onoff(true, 0);
            }else{
                const char led_off_msg[] = "Turning off LED 0.\r\n";
                nrfx_uart_tx(&g_uart0, (uint8_t const *)led_off_msg, sizeof(led_off_msg));
                ledctrl_onoff(false, 0);
            }
            break;
        }

        default:{
            nrfx_uart_tx(&g_uart0, (uint8_t const *)c_menu, sizeof(c_menu));
            break;
        }
    }
}


static void polling_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    switch(event_type){
        case NRF_TIMER_EVENT_COMPARE0:{
            if(true == g_streaming_imu){//check if we're streaming IMU data
                uint8_t accel_buf[6];
                mpu9250_drv_read_accel(&accel_buf);
                MPU9250_accel_val accel_val;
                mpu9250_drv_process_raw_accel(&accel_val, &accel_buf);
                char buf[64];
                size_t len = sprintf((char *)buf, "ACCEL(x,y,z):%f,%f,%f\r\n",accel_val.x, accel_val.y, accel_val.z);
                nrfx_uart_tx(&g_uart0, (uint8_t const *)buf, len);

            }
            break;
        }

        case NRF_TIMER_EVENT_COMPARE1:{
            break;
        }

        case NRF_TIMER_EVENT_COMPARE2:{
            break;
        }

        case NRF_TIMER_EVENT_COMPARE3:{
            break;
        }

        default:{
            break;
        }
    }

}
