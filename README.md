# STM32F411RE OTA Bootloader
### Course: EECE8041 Engineering Capstone Project

This project implements an **over-the-air (OTA) bootloader** for the STM32F411RE microcontroller that can communicate with the ESP8266 wifi module.

### Features
- **UART-based OTA:** Receives firmware packets from a host device (ESP) with a simple start/end delimiter protocol.  
- **Packet validation:** Each firmware packet is verified using a CRC32 check to ensure data integrity.  
- **Watchdog protection:** Independent watchdog (IWDG) ensures the MCU resets safely if a transfer stalls or firmware is corrupted.  
- **Flash management:** Supports staging, validation, and final transfer of firmware to main flash.  
- **Flags for state tracking:** Uses flash-resident flags to track firmware validity (`pending`, `valid`, `done`, `reset`).  
- **Automatic recovery:** On boot, the bootloader checks flags and moves valid firmware to main memory or resets safely if previous update failed.  

### Peripherals Used
- **USART1:** OTA firmware reception from ESP.  
- **USART2:** Debugging/printf output.  
- **CRC Unit:** Validates incoming firmware packets.  
- **Independent Watchdog (IWDG):** Ensures system reliability during OTA.   
