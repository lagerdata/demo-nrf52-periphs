#define _SHELL_C_SRC

//-------------------------MODULES USED-------------------------------------
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
* 'l' - Blink LEDs                     *\r\n\
*                                      *\r\n\
*                                      *\r\n\
****************************************\r\n";

static nrfx_timer_t g_polling = NRFX_TIMER_INSTANCE(2);
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
                                   .interrupt_priority = 8};
    nrfx_uart_init(&g_uart0, &uart_cfg, uart_event_handler);
    nrfx_uart_rx_enable(&g_uart0);
    nrfx_uart_rx(&g_uart0, &g_data_buf[0], 1);

    nrfx_timer_config_t cfg = {
                               .frequency = NRF_TIMER_FREQ_16MHz,
                               .mode = NRF_TIMER_MODE_TIMER,
                               .bit_width = NRF_TIMER_BIT_WIDTH_32,
                               .interrupt_priority = 3,
                               .p_context = NULL};
    nrfx_timer_init(&g_polling, &cfg, polling_timer_event_handler);
    uint32_t cc_time = nrfx_timer_ms_to_ticks(&g_polling, 1);
    nrfx_timer_extended_compare(&g_polling, NRF_TIMER_CC_CHANNEL0, cc_time, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrfx_timer_enable(&g_polling);


}


//-------------------------LOCAL FUNCTIONS----------------------------------
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
            break;
        }
        case 'l':{
            if(false ==ledctrl_is_led_show_active()){
                nrfx_uart_tx(&g_uart0, (uint8_t const *)"Turning on LED show.\r\n", sizeof("Turning on LED show.\r\n"));
                ledctrl_start_led_show();
            }else{
                nrfx_uart_tx(&g_uart0, (uint8_t const *)"Turning off LED show.\r\n", sizeof("Turning off LED show.\r\n"));
                ledctrl_stop_led_show();
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
