#define _SHELL_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "nrfx_uart.h"
#include "ledctrl.h"

//-------------------------DEFINITIONS AND MACORS---------------------------



//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------
static void uart_event_handler(nrfx_uart_event_t const * p_event, void * p_context);
static void handle_rx_bytes(nrfx_uart_xfer_evt_t * p_rxtx);

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
