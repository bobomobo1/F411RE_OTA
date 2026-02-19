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

// Flash defs
#define FLASH_STAGING_START   0x08010000 // Start of sector 4


void ota_flash_erase_staging(void);
void ota_flash_write(uint32_t addr, uint8_t *data, uint16_t len);
void ota_flash_jump(void);

#endif // __ESP_OTA_FLASH_H__