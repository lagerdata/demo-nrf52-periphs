#define _LEDCTRL_C_SRC

//-------------------------MODULES USED-------------------------------------
#include <string.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrfx_timer.h"
#include "ledctrl.h"


//-------------------------DEFINITIONS AND MACORS---------------------------
#define MIN_NUM_BLINKS 1
#define MAX_NUM_BLINKS 9
#define MIN_BLINK_DURATION 1
#define MAX_BLINK_DURATION 4999
#define MIN_ROTATION_SPEED 1

#define LED_PIN(x) (13+x)

//-------------------------TYPEDEFS AND STRUCTURES--------------------------
typedef struct{
    uint32_t num_blinks;
    uint32_t ms_blink_duration;
    uint32_t ms_cnt;
    bool led_on_state;
}led_info_t;

//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------
static void led_timer_event_handler(nrf_timer_event_t event_type, void * p_context);


//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------
static nrfx_timer_t g_timer0 = NRFX_TIMER_INSTANCE(0);
const char c_led_context[] = "LEDCTRL";
static led_info_t g_led_info[4];
//-------------------------EXPORTED FUNCTIONS-------------------------------
int32_t ledctrl_init(void)
{
    nrf_gpio_cfg_output(LED_PIN(0));
    nrf_gpio_pin_set(LED_PIN(0));
    nrf_gpio_cfg_output(LED_PIN(1));
    nrf_gpio_pin_set(LED_PIN(1));
    nrf_gpio_cfg_output(LED_PIN(2));
    nrf_gpio_pin_set(LED_PIN(2));
    nrf_gpio_cfg_output(LED_PIN(3));
    nrf_gpio_pin_set(LED_PIN(3));

    memset(&g_led_info, 0, sizeof(g_led_info));
    nrfx_timer_config_t cfg = {
                               .frequency = NRF_TIMER_FREQ_16MHz,
                               .mode = NRF_TIMER_MODE_TIMER,
                               .bit_width = NRF_TIMER_BIT_WIDTH_32,
                               .interrupt_priority = 5,
                               .p_context = (void *)&c_led_context};
    nrfx_timer_init(&g_timer0, &cfg, led_timer_event_handler);
    uint32_t cc_time = nrfx_timer_ms_to_ticks(&g_timer0, 1);
    nrfx_timer_extended_compare(&g_timer0, NRF_TIMER_CC_CHANNEL0, cc_time, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrfx_timer_extended_compare(&g_timer0, NRF_TIMER_CC_CHANNEL1, cc_time, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);
    nrfx_timer_extended_compare(&g_timer0, NRF_TIMER_CC_CHANNEL2, cc_time, NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, true);
    nrfx_timer_extended_compare(&g_timer0, NRF_TIMER_CC_CHANNEL3, cc_time, NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK, true);

    return LEDCTRL_OK;
}

int32_t ledctrl_blinkled(uint32_t num_blink, uint32_t ms_blink_duration, uint32_t num_led)
{
	if(num_blink < MIN_NUM_BLINKS){
		return LEDCTRL_ERR;
	}
	if(num_blink > MAX_NUM_BLINKS){
		return LEDCTRL_ERR;
	}
	if(ms_blink_duration < MIN_BLINK_DURATION){
		return LEDCTRL_ERR;
	}
	if(ms_blink_duration > MAX_BLINK_DURATION){
		return LEDCTRL_ERR;
	}
    g_led_info[num_led].num_blinks = num_blink*2;
    g_led_info[num_led].ms_blink_duration = ms_blink_duration;
    g_led_info[num_led].ms_cnt = g_led_info[num_led].ms_blink_duration;

    ledctrl_onoff(true, num_led);
    nrfx_timer_clear(&g_timer0);
    nrfx_timer_enable(&g_timer0);
	return LEDCTRL_OK;
}

bool ledctrl_isActive(uint32_t led_num)
{
    return (bool)g_led_info[led_num].num_blinks;
}

int32_t ledctrl_onoff(bool led_on, uint32_t led_num)
{

	if(led_on){
        nrf_gpio_pin_set(LED_PIN(led_num));
        g_led_info[led_num].led_on_state = true;
	}else{
        nrf_gpio_pin_clear(LED_PIN(led_num));
        g_led_info[led_num].led_on_state = false;
	}

	return LEDCTRL_OK;
}

bool ledctrl_is_led_num_on(uint32_t led_num)
{
    return g_led_info[led_num].led_on_state;
}

//-------------------------LOCAL FUNCTIONS----------------------------------
static void led_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    uint32_t i = 0;
    switch(event_type){
        case NRF_TIMER_EVENT_COMPARE0:
            i = 0;
        case NRF_TIMER_EVENT_COMPARE1:
            i = 1;
        case NRF_TIMER_EVENT_COMPARE2:
            i = 2;
        case NRF_TIMER_EVENT_COMPARE3:{
            i = 3;
            if(0 == --g_led_info[i].ms_cnt){
                g_led_info[i].num_blinks--;
                if(0 == g_led_info[i].num_blinks){
                    nrfx_timer_disable(&g_timer0);
                    return;
                }
                g_led_info[i].ms_cnt = g_led_info[i].ms_blink_duration;
                nrf_gpio_pin_toggle(LED_PIN(i));
            }
            break;
        }

        default:{
            break;
        }
    }
}
