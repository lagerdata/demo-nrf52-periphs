#define _TEST_EXAMPLE_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "unity.h"
#include "nrf_delay.h"
#include "ledctrl.h"
#include "test_ledctrl.h"

//-------------------------DEFINITIONS AND MACORS---------------------------



//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------
static void wait_until_done(uint32_t ms_timeout);


//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------



//-------------------------EXPORTED FUNCTIONS-------------------------------
void test_check_min_number_blinks(void)
{
	uint32_t num_blink = 0;
	uint32_t ms_blink_duration = 10;
	int32_t ret = ledctrl_blinkled(num_blink, ms_blink_duration, 0);
	TEST_ASSERT_EQUAL_INT(LEDCTRL_ERR, ret);
	num_blink = 1;
	ret = ledctrl_blinkled(num_blink, ms_blink_duration, 0);
	TEST_ASSERT_EQUAL_INT(LEDCTRL_OK, ret);
    wait_until_done(100);

	num_blink = 2;
	ret = ledctrl_blinkled(num_blink, ms_blink_duration, 0);
	TEST_ASSERT_EQUAL_INT(LEDCTRL_OK, ret);
    wait_until_done(100);
}

void test_check_max_number_blinks(void)
{
	uint32_t num_blink = 10;
	uint32_t ms_blink_duration = 10;
	int32_t ret = ledctrl_blinkled(num_blink, ms_blink_duration);
	TEST_ASSERT_EQUAL_INT(LEDCTRL_ERR, ret);

	num_blink = 9;
	ret = ledctrl_blinkled(num_blink, ms_blink_duration);
	TEST_ASSERT_EQUAL_INT(LEDCTRL_OK, ret);
    wait_until_done(200);

	num_blink = 8;
	ret = ledctrl_blinkled(num_blink, ms_blink_duration);
	TEST_ASSERT_EQUAL_INT(LEDCTRL_OK, ret);
    wait_until_done(200);
}


//-------------------------LOCAL FUNCTIONS----------------------------------
static void wait_until_done(uint32_t ms_timeout)
{
    while((true ==ledctrl_isActive(0)) || (0==ms_timeout)){
        nrf_delay_ms(1);
        ms_timeout--;
    }
}
