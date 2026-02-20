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
    for (uint8_t i = 0; i < 8; i++) { // 64 bytes / 8 bytes per double-word = 8 iterations
        uint64_t value = 0;
        // Combine 8 bytes into a uint64_t
        for (uint8_t j = 0; j < 8; j++) {
            value |= ((uint64_t)data[i*8 + j]) << (8*j);
        }
        // Program 8 bytes at a time
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i*8, value) != HAL_OK) {
            // Handle error
        }
    }
    HAL_FLASH_Lock();
}

// NOT WORKING AS OF NOW
void ota_flash_jump(void){
    uint32_t sp = *(volatile uint32_t*)FLASH_STAGING_START;
    uint32_t reset = *(volatile uint32_t*)FLASH_STAGING_START+4;
    void (*app)(void) = (void(*)(void))reset;
    __disable_irq();
    HAL_DeInit();
    HAL_RCC_DeInit();
    SCB->VTOR = FLASH_STAGING_START;
    __set_MSP(sp);
    __enable_irq();
    app();
}