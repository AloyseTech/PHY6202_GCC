
#ifndef APP_AP_TIMER_H
#define APP_AP_TIMER_H

#include "types.h"

typedef void (*app_aptm_hdl_t)(void);

void app_aptimer_osal_timer_handler(void);
int app_ap_timer_start(uint8_t taskID, uint16_t osal_evt, uint32_t ms, app_aptm_hdl_t callback);
int app_ap_timer_stop(void);


#endif

