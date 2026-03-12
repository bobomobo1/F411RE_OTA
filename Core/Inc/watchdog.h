#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "stm32f4xx_hal.h"

/* Initialize Independent Watchdog */
void Watchdog_Init(void);

/* Refresh watchdog counter */
void Watchdog_Refresh(void);

#endif