#define _TEST1_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "unity.h"


//-------------------------DEFINITIONS AND MACORS---------------------------



//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------



//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------



//-------------------------EXPORTED FUNCTIONS-------------------------------


void test_thatSucceeds(void)
{
	TEST_PASS_MESSAGE("This test passed");
}

void test_succeedsWithCondition(void)
{
	TEST_ASSERT_EQUAL_INT(42, 42);
}



//-------------------------LOCAL FUNCTIONS----------------------------------
