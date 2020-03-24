
#ifndef APP_AP_TIMER_H
#define APP_AP_TIMER_H

#include "types.h"
#include "ap_timer.h"

typedef void (*app_aptm_hdl_t)(void);

void app_aptimer_osal_timer_handler_m(uint8_t id);
int app_ap_timer_start_m(uint8_t tmID, uint32_t ms, app_aptm_hdl_t callback);
int app_ap_timer_stop_m(uint8_t tmID);
int app_ap_timer_init_m(uint8_t taskID, uint16_t osal_evt_1, uint16_t osal_evt_2, uint16_t osal_evt_3, uint16_t osal_evt_4);


#endif

