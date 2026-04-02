#include "esp_ota_flash.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include <stdio.h>

// Erases entire staging section of memory
void ota_flash_erase_sector(uint8_t eraseSector){
    FLASH_EraseInitTypeDef erase;
    uint32_t error;
    HAL_FLASH_Unlock();
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = eraseSector;
    erase.NbSectors = 1;
    if(HAL_FLASHEx_Erase(&erase, &error)!=HAL_OK){ // May need to swtich to IT 
        // Handle error
        printf("Error Erasing flash\r\n");
        HAL_FLASH_Lock();
        return;
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

void ota_flash_jump(uint32_t jump_address){
    uint32_t main_sp = *((uint32_t*)jump_address);
    uint32_t main_reset = *((uint32_t*)(jump_address + 4));
    // 1. Validate stack pointer
    if(main_sp < 0x20000000 || main_sp > 0x20020000){
        printf("Invalid stack pointer\r\n");
        return;
    }
    // 2. Set vector table
    SCB->VTOR = jump_address;
    // 3. Set main stack pointer
    __set_MSP(main_sp);
    // 4. Jump to reset handler 
    typedef void (*pFunction)(void);
    pFunction app_reset = (pFunction)main_reset;
    __enable_irq();           
    app_reset();             
}

void ota_move_to_main(uint16_t packet_number){
    // Get firmware size
    uint32_t total_packets = packet_number + 1;
    uint32_t firmware_size = total_packets * TX_DATA_SIZE;
    printf("Moving firmware: %lu bytes\r\n", firmware_size);
    ota_flash_erase_sector(FLASH_MAIN_SECTOR);
    // Copy staging to main
    uint32_t src_addr = FLASH_STAGING_START;
    uint32_t dst_addr = FLASH_MAIN_START;
    uint32_t remaining = firmware_size;
    while(remaining > 0){
        uint16_t chunk_size;
        if(remaining >= TX_DATA_SIZE){
            chunk_size = TX_DATA_SIZE;
        } else {
            chunk_size = remaining;
        }
        ota_flash_write(dst_addr, (uint8_t*)src_addr, chunk_size);
        src_addr += chunk_size;
        dst_addr += chunk_size;
        remaining -= chunk_size;
    }
    // Verify our code is the same from staging 
    uint8_t *src = (uint8_t*)FLASH_STAGING_START;
    uint8_t *dst = (uint8_t*)FLASH_MAIN_START;
    for(uint32_t i = 0; i < firmware_size; i++){
        if(src[i] != dst[i]){
            printf("Verification failed at byte %lu\r\n", i);
            return;
        }
    }
}