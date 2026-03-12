#include "watchdog.h"

/* Watchdog handle */
IWDG_HandleTypeDef hiwdg;

/*

* Initialize Independent Watchdog
* Timeout ≈ 2 seconds
  */
  void Watchdog_Init(void)
  {
  hiwdg.Instance = IWDG;

  /* Prescaler divides LSI clock */
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;

  /*
  LSI ≈ 32kHz
  Timeout = (Reload + 1) / (LSI / Prescaler)

  (1000+1)/(32000/64) ≈ 2 seconds
  */
  hiwdg.Init.Reload = 1000;

  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
  while(1);  // Initialization error
  }
  }

/* Feed / refresh watchdog */
void Watchdog_Refresh(void)
{
HAL_IWDG_Refresh(&hiwdg);
}
