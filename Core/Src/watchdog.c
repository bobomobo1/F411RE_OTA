#include "watchdog.h"

/* Watchdog handle */
IWDG_HandleTypeDef hiwdg;

/*

* Initialize Independent Watchdog
* Timeout ≈ 2 seconds
  */
  void watchdog_init(void)
  {
  hiwdg.Instance = IWDG;

  /* Prescaler divides LSI clock */
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64; // adjust prescaler

  /*
  LSI ≈ 32kHz
  Timeout = (Reload + 1) / (LSI / Prescaler)

   for this configuration -
       Prescaler = 64 
      Reload = 2000

  (2000+1)/(32000/64) ≈ 4 seconds
  */
  hiwdg.Init.Reload = 2000; 

  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  }

/* Feed / refresh watchdog */
void watchdog_refresh(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}
