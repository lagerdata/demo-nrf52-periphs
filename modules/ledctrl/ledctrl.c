#define _LEDCTRL_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "nrf_gpio.h"
#include "nrfx_timer.h"
#include "ledctrl.h"


//-------------------------DEFINITIONS AND MACORS---------------------------
#define MIN_NUM_BLINKS 1
#define MAX_NUM_BLINKS 9
#define MIN_BLINK_DURATION 1
#define MAX_BLINK_DURATION 4999

#define LED_PIN 13
//-------------------------TYPEDEFS AND STRUCTURES--------------------------
typedef struct{
    uint32_t num_blinks;
    uint32_t ms_blink_duration;
    uint32_t ms_cnt;
}led_info_t;


//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------
static void led_timer_event_handler(nrf_timer_event_t event_type, void * p_context);


//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------
static nrfx_timer_t g_timer0 = NRFX_TIMER_INSTANCE(0);
const char c_led_context[] = "LEDCTRL";
const uint32_t c_led_timer_1MS_rollover_cnt = 63;
static led_info_t g_led_info = {
                                .num_blinks = 0,
                                .ms_blink_duration = 0,
                                .ms_cnt = 0};
//-------------------------EXPORTED FUNCTIONS-------------------------------
int32_t ledctrl_init(void)
{
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_set(LED_PIN);
    nrfx_timer_config_t cfg = {
                               .frequency = NRF_TIMER_FREQ_16MHz,
                               .mode = NRF_TIMER_MODE_TIMER,
                               .bit_width = NRF_TIMER_BIT_WIDTH_8,
                               .interrupt_priority = 7,
                               .p_context = (void *)&c_led_context};
    nrfx_timer_init(&g_timer0, &cfg, led_timer_event_handler);


    return LEDCTRL_OK;
}

int32_t ledctrl_blinkled(uint32_t num_blink, uint32_t ms_blink_duration)
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
    g_led_info.num_blinks = num_blink*2;
    g_led_info.ms_blink_duration = ms_blink_duration;
    g_led_info.ms_cnt = g_led_info.ms_blink_duration;

    ledctrl_onoff(true);
    nrfx_timer_clear(&g_timer0);
    nrfx_timer_enable(&g_timer0);
	return LEDCTRL_OK;
}

bool ledctrl_isActive(void)
{
    return (bool)g_led_info.num_blinks;
}

int32_t ledctrl_onoff(bool led_on)
{

	if(led_on){
        nrf_gpio_pin_clear(LED_PIN);
	}else{
        nrf_gpio_pin_set(LED_PIN);
	}

	return LEDCTRL_OK;
}



//-------------------------LOCAL FUNCTIONS----------------------------------
static void led_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    static uint32_t ms_rollover_cnt = 0;

    ms_rollover_cnt++;
    if(0 == (ms_rollover_cnt%c_led_timer_1MS_rollover_cnt)){
        if(0 == --g_led_info.ms_cnt){
            g_led_info.num_blinks--;
            if(0 == g_led_info.num_blinks){
                nrfx_timer_disable(&g_timer0);
                ms_rollover_cnt = 0;
                return;
            }
            g_led_info.ms_cnt = g_led_info.ms_blink_duration;
            nrf_gpio_pin_toggle(LED_PIN);
        }

    }

}
