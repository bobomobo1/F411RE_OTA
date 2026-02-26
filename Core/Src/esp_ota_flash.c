#include "esp_ota_flash.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include <stdio.h>

// Erases entire staging section of memory(sector 4)
void ota_flash_erase_staging(void){
    FLASH_EraseInitTypeDef erase;
    uint32_t error;
    HAL_FLASH_Unlock();
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = 5;
    erase.NbSectors = 1;
    if(HAL_FLASHEx_Erase(&erase, &error)!=HAL_OK){ // May need to swtich to IT 
        // Handle error
        printf("Error Erasing flash\r\n");
    }
    HAL_FLASH_Lock();
}

void ota_flash_write(uint32_t addr, uint8_t *data, uint16_t len){
    HAL_FLASH_Unlock();
    // Calculate how many full words we have
    uint16_t word_count = len / 4;
    uint16_t remaining_bytes = len % 4;
    uint32_t *word_data = (uint32_t*)data;
    // Flash 4 bytes (32-bit word) at a time
    for(uint16_t i = 0; i < word_count; i++){
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i*4, word_data[i]);
    }
    // Flash remaining bytes if len is not multiple of 4
    if(remaining_bytes){
        uint32_t last_word = 0xFFFFFFFF; // Fill remaining with 0xFF
        for(uint16_t i=0; i<remaining_bytes; i++){
            ((uint8_t*)&last_word)[i] = data[word_count*4 + i];
        }
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + word_count*4, last_word)!=HAL_OK){
            // Handle errror
            printf("ERROR FLASHING\r\n");
            return;
        }
    }
    HAL_FLASH_Lock();
}

// NOT WORKING AS OF NOW
void ota_flash_jump(void){
    uint32_t sp = *(volatile uint32_t*)FLASH_STAGING_START;
    uint32_t reset = *(volatile uint32_t*)(FLASH_STAGING_START+4);
    void (*app)(void) = (void(*)(void))reset;
    __disable_irq();
    HAL_DeInit();
    HAL_RCC_DeInit();
    SCB->VTOR = FLASH_STAGING_START;
    __set_MSP(sp);
    __enable_irq();
    app();
}