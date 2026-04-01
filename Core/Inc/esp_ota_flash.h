#ifndef __ESP_OTA_FLASH_H__
#define __ESP_OTA_FLASH_H__

#include <stdint.h>

// UART defs
#define TX_START_DELIM_1      0x7E
#define TX_START_DELIM_2      0x7E
#define TX_END_DELIM_1        0x7F
#define TX_END_DELIM_2        0x7F
#define TX_DELIMITER_SIZE     2
#define TX_HEADER_SIZE        3 // 2 Bytes(uint16) for packet # and 1 byte(uint8) for packet size
#define TX_CRC_SIZE           4
#define TX_DATA_SIZE          128
#define TX_CHUNK_SIZE         TX_DELIMITER_SIZE + TX_HEADER_SIZE + TX_DATA_SIZE + TX_CRC_SIZE + TX_DELIMITER_SIZE

#define TX_END_DELIM_1_INDEX  TX_CHUNK_SIZE - TX_DELIMITER_SIZE - TX_DELIMITER_SIZE
#define TX_END_DELIM_2_INDEX  TX_END_DELIM_1_INDEX + 1

#define TX_START_CRC_INDEX    TX_END_DELIM_1_INDEX - TX_CRC_SIZE

// Flash defs
#define FLASH_STAGING_START   0x08040000 // Start of sector 6
#define FLASH_STAGING_SECTOR  6

#define FLASH_MAIN_START      0x08010000 // Start of sector 4
#define FLASH_MAIN_SECTOR     4

#define FLASH_FLAG_START	  0x0800C000 // Start of sector 3
#define FLASH_FLAG_SECTOR	  3

#define FLASH_SIZE_START      (FLASH_FLAG_START + (sizeof(uint32_t))) // Starts after our packet count flag 

void ota_flash_erase_sector(uint8_t eraseSector);
void ota_flash_write(uint32_t addr, uint8_t *data, uint16_t len);
void ota_flash_jump(uint32_t jump_address);
void ota_move_to_main(uint16_t packet_number);

#endif // __ESP_OTA_FLASH_H__