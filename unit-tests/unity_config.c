#define _UNITY_CONFIG_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "unity_config.h"



//-------------------------DEFINITIONS AND MACORS---------------------------



//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------



//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------



//-------------------------EXPORTED FUNCTIONS-------------------------------
void target_putc(char a)
{
    nrf_uart_txd_set(TARGET_UART, a);
    while(!nrf_uart_event_check(TARGET_UART, NRF_UART_EVENT_TXDRDY));
    NRF_UART0->EVENTS_TXDRDY = 0;
    //Super Hacky delay that gets /dev/ttyS0 working properly on RPi4
    //Apparently on RPi4 /dev/ttyS0 is bit banged and so sending data
    //to it too quickly can screw the ttyS0 port up 
    volatile int i;
    for(i=0;i<10000;i++);
}

void target_init_putc(void)
{
    nrf_uart_configure(TARGET_UART, TARGET_PARITY, TARGET_HWFC);
    nrf_uart_baudrate_set(TARGET_UART, TARGET_BAUDRATE);
    nrf_uart_txrx_pins_set(TARGET_UART, TARGET_TX_PIN, 0);
    nrf_uart_enable(TARGET_UART);
    nrf_uart_task_trigger(TARGET_UART, NRF_UART_TASK_STARTTX);
}

void target_flush_putc(void)
{
    nrf_uart_task_trigger(TARGET_UART, NRF_UART_TASK_STOPTX);
    nrf_uart_task_trigger(TARGET_UART, NRF_UART_TASK_STARTTX);
}

void target_close_putc(void)
{
    nrf_uart_task_trigger(TARGET_UART, NRF_UART_TASK_STOPTX);
    nrf_uart_disable(TARGET_UART);
}



//-------------------------LOCAL FUNCTIONS----------------------------------
