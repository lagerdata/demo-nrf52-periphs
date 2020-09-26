#ifndef _UNITY_CONFIG_INCLUDED
#define _UNITY_CONFIG_INCLUDED
//-------------------------MODULES USED-------------------------------------
#include "nrf52840.h"
#include "nrf_uart.h"


//-------------------------DEFINITIONS AND MACORS---------------------------



#define UNITY_OUTPUT_CHAR(a)    target_putc(a)
#define UNITY_OUTPUT_START()    target_init_putc()
#define UNITY_OUTPUT_FLUSH()    target_flush_putc()
#define UNITY_OUTPUT_COMPLETE() target_close_putc()

#define UNITY_EXCLUDE_MATH_H

#define TARGET_UART NRF_UART0
#define TARGET_BAUDRATE NRF_UART_BAUDRATE_115200
#define TARGET_HWFC NRF_UART_HWFC_DISABLED
#define TARGET_PARITY NRF_UART_PARITY_EXCLUDED
#define TARGET_TX_PIN 6
#define TARGET_TX_PORT PORT_ZERO
//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------EXPORTED VARIABLES ------------------------------
#ifndef _UNITY_CONFIG_C_SRC



#endif



//-------------------------EXPORTED FUNCTIONS-------------------------------
void target_putc(char a);
void target_init_putc(void);
void target_flush_putc(void);
void target_close_putc(void);


#endif
