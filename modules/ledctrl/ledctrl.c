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

typedef struct{
    uint32_t led_show_state;
    bool is_led_show_active;
}led_show_t;
//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------
static void led_timer_event_handler(nrf_timer_event_t event_type, void * p_context);
static void led_show_timer_event_handler(nrf_timer_event_t event_type, void * p_context);

//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------
static nrfx_timer_t g_timer0 = NRFX_TIMER_INSTANCE(0);
static nrfx_timer_t g_timer1 = NRFX_TIMER_INSTANCE(1);
const char c_led_context[] = "LEDCTRL";
const uint32_t c_led_timer_1MS_rollover_cnt = 63;
static led_info_t g_led_info = {
                                .num_blinks = 0,
                                .ms_blink_duration = 0,
                                .ms_cnt = 0};
static led_show_t g_led_show_info = {
                                     .led_show_state = 0,
                                     .is_led_show_active = false};
//-------------------------EXPORTED FUNCTIONS-------------------------------
int32_t ledctrl_init(void)
{
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_set(LED_PIN);

    nrfx_timer_config_t cfg = {
                               .frequency = NRF_TIMER_FREQ_16MHz,
                               .mode = NRF_TIMER_MODE_TIMER,
                               .bit_width = NRF_TIMER_BIT_WIDTH_32,
                               .interrupt_priority = 5,
                               .p_context = (void *)&c_led_context};
    nrfx_timer_init(&g_timer0, &cfg, led_timer_event_handler);
    uint32_t cc_time = nrfx_timer_ms_to_ticks(&g_timer0, 1);
    nrfx_timer_extended_compare(&g_timer0, NRF_TIMER_CC_CHANNEL0, cc_time, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrfx_timer_config_t led_cfg = {
                                   .frequency = NRF_TIMER_FREQ_16MHz,
                                   .mode = NRF_TIMER_MODE_TIMER,
                                   .bit_width = NRF_TIMER_BIT_WIDTH_32,
                                   .interrupt_priority = 6,
                                   .p_context = NULL};
    nrfx_timer_init(&g_timer1, &led_cfg, led_show_timer_event_handler);
    cc_time = nrfx_timer_ms_to_ticks(&g_timer1, 100);
    nrfx_timer_extended_compare(&g_timer1, NRF_TIMER_CC_CHANNEL0, cc_time, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);



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

int32_t ledctrl_start_led_show(void)
{
    if(false == g_led_show_info.is_led_show_active){
        nrfx_timer_enable(&g_timer1);
        g_led_show_info.is_led_show_active = true;
    }
    return 0;
}

int32_t ledctrl_stop_led_show(void)
{
    nrfx_timer_disable(&g_timer1);
    nrfx_timer_clear(&g_timer1);
    g_led_show_info.led_show_state = 0;
    g_led_show_info.is_led_show_active = false;
    return 0;
}

bool ledctrl_is_led_show_active(void)
{
    return g_led_show_info.is_led_show_active;
}

//-------------------------LOCAL FUNCTIONS----------------------------------
static void led_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{

    switch(event_type){
        case NRF_TIMER_EVENT_COMPARE0:{
            if(0 == --g_led_info.ms_cnt){
                g_led_info.num_blinks--;
                if(0 == g_led_info.num_blinks){
                    nrfx_timer_disable(&g_timer0);
                    return;
                }
                g_led_info.ms_cnt = g_led_info.ms_blink_duration;
                nrf_gpio_pin_toggle(LED_PIN);
            }
            break;
        }
        default:{
            break;
        }
    }
}

void led_show_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    switch(event_type){
        case NRF_TIMER_EVENT_COMPARE0:{

            if(true == ledctrl_isActive()){
                return;
            }
            switch(g_led_show_info.led_show_state){
                case 0:{
                    ledctrl_blinkled(9, 200);
                    break;
                }

                case 1:{
                    ledctrl_blinkled(8, 400);
                    break;
                }

                case 2:{
                    ledctrl_blinkled(6, 800);
                    break;
                }

                case 3:{
                    ledctrl_blinkled(4, 1000);
                    break;
                }

                case 4:{
                    ledctrl_blinkled(2, 2000);
                    break;
                }

                default:{
                    g_led_show_info.led_show_state = 0;
                    break;
                }
            }
            g_led_show_info.led_show_state = ++g_led_show_info.led_show_state>4?0:g_led_show_info.led_show_state;
            break;
        }
        default:{
            break;
        }
    }
}
