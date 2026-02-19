#include "esp_ota_flash.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

// Erases entire staging section of memory(sector 4)
void ota_flash_erase_staging(void){
    FLASH_EraseInitTypeDef erase;
    uint32_t error;
    HAL_FLASH_Unlock();
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = 4;
    erase.NbSectors = 1;
    HAL_FLASHEx_Erase(&erase, &error); // May need to swtich to IT 
    HAL_FLASH_Lock();
}

void ota_flash_write(uint32_t addr, uint8_t *data, uint16_t len){
    HAL_FLASH_Unlock();
    // Flash each byte
    for(uint8_t i=0;i<len;i++){
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr+i, data[i]); // May need to switch to IT
    }
    HAL_FLASH_Lock();
}

void ota_flash_jump(void){
    uint32_t sp = (uint32_t)FLASH_STAGING_START;
    uint32_t reset = (uint32_t)FLASH_STAGING_START+4;
    void (*app)(void) = (void*)reset;
    __disable_irq();
    __set_MSP(sp);
    app();
}