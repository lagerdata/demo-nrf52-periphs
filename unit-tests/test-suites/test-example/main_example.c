#define _MAIN_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "unity.h"
#include "test_example.h"
//-------------------------DEFINITIONS AND MACORS---------------------------



//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------



//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------



//-------------------------EXPORTED FUNCTIONS-------------------------------
void setUp(void)
{

}

void tearDown(void)
{

}

int main(void)
{

    UnityBegin("Test Examples");
    DO_TEST(test_thatSucceeds);
    DO_TEST(test_succeedsWithCondition);
    UnityEnd();


    return 0;
}

void HardFault_Handler(void)
{
    while(1);
}
//-------------------------LOCAL FUNCTIONS----------------------------------
