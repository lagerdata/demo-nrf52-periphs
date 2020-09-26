#define _MAIN_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "unity.h"
#include "ledctrl.h"
#include "test_ledctrl.h"
//-------------------------DEFINITIONS AND MACORS---------------------------



//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------



//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------



//-------------------------EXPORTED FUNCTIONS-------------------------------
void setUp(void)
{
    ledctrl_init();
}

void tearDown(void)
{

}

int main(void)
{

    UnityBegin("modules/ledctrl.c");
    DO_TEST(test_check_min_number_blinks);
    DO_TEST(test_check_max_number_blinks);
    DO_TEST(test_check_min_blink_duration);
    DO_TEST(test_check_max_blink_duration);
    DO_TEST(test_total_blink_length);
    UnityEnd();

    return 0;
}

void HardFault_Handler(void)
{
    while(1);
}
//-------------------------LOCAL FUNCTIONS----------------------------------
