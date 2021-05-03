#ifndef _SHELL_INCLUDED
#define _SHELL_INCLUDED
//-------------------------MODULES USED-------------------------------------



//-------------------------DEFINITIONS AND MACORS---------------------------



//-------------------------TYPEDEFS AND STRUCTURES--------------------------
typedef struct {
    bool streaming_imu;
    bool raw;
} ShellContext;


//-------------------------EXPORTED VARIABLES ------------------------------
#ifndef _SHELL_C_SRC



#endif



//-------------------------EXPORTED FUNCTIONS-------------------------------
void shell_init(void);
void shell_uart_tx(uint8_t * p_buf, size_t len);

#endif
