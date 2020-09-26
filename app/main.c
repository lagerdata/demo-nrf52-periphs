#define _MAIN_C_SRC

//-------------------------MODULES USED-------------------------------------
#include "nrfx_timer.h"
#include "ledctrl.h"
#include "shell.h"
//-------------------------DEFINITIONS AND MACORS---------------------------



//-------------------------TYPEDEFS AND STRUCTURES--------------------------



//-------------------------PROTOTYPES OF LOCAL FUNCTIONS--------------------
static void main_timer_event_handler(nrf_timer_event_t event_type, void * p_context);


//-------------------------EXPORTED VARIABLES ------------------------------



//-------------------------GLOBAL VARIABLES---------------------------------
static nrfx_timer_t g_timer1 = NRFX_TIMER_INSTANCE(1);
const uint32_t c_main_timer_1000MS_rollover_cnt = 62500;
const char c_main_context[] = "MAIN";


//-------------------------EXPORTED FUNCTIONS-------------------------------
int main(void)
{
    nrfx_timer_config_t led_cfg = {
                               .frequency = NRF_TIMER_FREQ_16MHz,
                               .mode = NRF_TIMER_MODE_TIMER,
                               .bit_width = NRF_TIMER_BIT_WIDTH_8,
                               .interrupt_priority = 7,
                               .p_context = (void*)c_main_context};
    nrfx_timer_init(&g_timer1, &led_cfg, main_timer_event_handler);
    ledctrl_init();
    nrfx_timer_enable(&g_timer1);

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
static void main_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    static uint32_t led_state = 0;
    static uint32_t ms_rollover_cnt = 0;
    ms_rollover_cnt++;
    if(0 == (ms_rollover_cnt%c_main_timer_1000MS_rollover_cnt)){
        if(true == ledctrl_isActive()){
            return;
        }
        switch(led_state){
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
                led_state = 0;
                break;
            }
        }
        led_state = ++led_state>4?0:led_state;
    }
}

