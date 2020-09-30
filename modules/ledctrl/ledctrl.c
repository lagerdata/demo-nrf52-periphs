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
static led_info_t g_led_info[4];
static led_show_t g_led_show_info = {
                                     .led_show_state = 0,
                                     .is_led_show_active = false};
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
    g_led_info[num_led].ms_cnt = g_led_info[0].ms_blink_duration;

    ledctrl_onoff(true, num_led);
    nrfx_timer_clear(&g_timer0);
    nrfx_timer_enable(&g_timer0);
	return LEDCTRL_OK;
}

int32_t ledctrl_rotateled(uint32_t num_rotations, uint32_t ms_rotation_speed)
{
    if(num_rotations < 1){
        return LEDCTRL_ERR;
    }

    if(ms_rotation_speed < MIN_ROTATION_SPEED){
        return LEDCTRL_ERR;
    }

    g_led_info[0].num_blinks = num_rotations*2;
    g_led_info[0].ms_blink_duration = ms_rotation_speed;
    g_led_info[0].ms_cnt = g_led_info[0].ms_blink_duration;
    nrf_delay_ms(ms_rotation_speed);
    g_led_info[1].num_blinks = num_rotations*2;
    g_led_info[1].ms_blink_duration = ms_rotation_speed;
    g_led_info[1].ms_cnt = g_led_info[1].ms_blink_duration;
    nrf_delay_ms(ms_rotation_speed);
    g_led_info[2].num_blinks = num_rotations*2;
    g_led_info[2].ms_blink_duration = ms_rotation_speed;
    g_led_info[2].ms_cnt = g_led_info[2].ms_blink_duration;
    nrf_delay_ms(ms_rotation_speed);
    g_led_info[3].num_blinks = num_rotations*2;
    g_led_info[3].ms_blink_duration = ms_rotation_speed;
    g_led_info[3].ms_cnt = g_led_info[3].ms_blink_duration;

    nrfx_timer_clear(&g_timer0);
    nrfx_timer_enable(&g_timer0);


    return LEDCTRL_OK;
}

bool ledctrl_isActive(void)
{
    return (bool)g_led_info[0].num_blinks;
}

int32_t ledctrl_onoff(bool led_on, uint32_t led_num)
{

	if(led_on){
        nrf_gpio_pin_clear(LED_PIN(led_num));
	}else{
        nrf_gpio_pin_set(LED_PIN(led_num));
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

void led_show_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    switch(event_type){
        case NRF_TIMER_EVENT_COMPARE0:{

            if(true == ledctrl_isActive()){
                return;
            }
            switch(g_led_show_info.led_show_state){
                case 0:{
                    ledctrl_blinkled(9, 200, 0);
                    break;
                }

                case 1:{
                    ledctrl_blinkled(8, 400, 1);
                    break;
                }

                case 2:{
                    ledctrl_blinkled(6, 800, 2);
                    break;
                }

                case 3:{
                    ledctrl_blinkled(4, 1000, 3);
                    break;
                }

                case 4:{
                    ledctrl_blinkled(2, 2000, 4);
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
