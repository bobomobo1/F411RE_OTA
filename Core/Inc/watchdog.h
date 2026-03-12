#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "stm32f4xx_hal.h"

/* Initialize Independent Watchdog */
void watchdog_init(void);

/* Refresh watchdog counter */
void watchdog_refresh(void);

#endif