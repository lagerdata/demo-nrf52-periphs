#define _MAIN_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "nrfx.h"
#include "nrfx_timer.h"
#include "ledctrl.h"
#include "shell.h"
//-------------------------DEFINITIONS AND MACORS---------------------------



//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------



//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------





//-------------------------EXPORTED FUNCTIONS-------------------------------
int main(void)
{


    ledctrl_init();
    shell_init();

    while(1){
        __WFI();
    }

    return 0;
}

void HardFault_Handler(void)
{
    while(1);
}
//-------------------------LOCAL FUNCTIONS----------------------------------
