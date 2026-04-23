/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "esp_ota_flash.h"
#include "stm32f4xx_hal_crc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar (void)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t count = 0;
const uint8_t ack = 0x79;
const uint8_t ready_response = 0x78;
const uint8_t nack = 0x1F; 
const uint8_t stop_send_ack = 0x1E;
uint8_t rx_buff[TX_CHUNK_SIZE]; 
volatile uint8_t rx_complete_flag = 0; 
volatile uint8_t switch_to_main_flag = 0;
uint8_t rx_byte;
uint8_t rx_index = 0;
typedef enum {WAIT_START_1, WAIT_START_2, WAIT_OTA_SEQUENCE, RECEIVE_PACKET} rx_state_t; // State machine for rx_packets
rx_state_t rx_state = WAIT_START_1;
uint16_t packet_number;
uint8_t  total_packets;
uint32_t packet_CRC;
uint32_t flash_pointer = FLASH_STAGING_START; // Used to keep track of where we are flashing
const uint32_t pending_flag = 0xBBBBBBBB;
const uint32_t valid_flag = 0xAAAAAAAA;
const uint32_t done_flag = 0xCCCCCCCC;
const uint32_t reset_flag = 0xDDDDDDDD;

static uint8_t bad_packet_count = 0;
static uint16_t last_packet_number = 0xFFFF;

uint8_t ota_start_count = 0;
uint8_t ota_main_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  // Check if we reset because of a watchdog timer
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)){
    // Handle it
    printf("Our Firmware Failed\r\n");
    __HAL_RCC_CLEAR_RESET_FLAGS();
    ota_flash_erase_sector(FLASH_FLAG_SECTOR); // Reset flags  
  }
  // Check flag (just for testing right now)
  uint32_t flag = *(uint32_t*)FLASH_FLAG_START; 
  printf("Flag: %X\r\n", flag);
  uint16_t flash_packet_number = *(uint16_t*)FLASH_SIZE_START;
  printf("Number of packets: %d\r\n", flash_packet_number);
  uint32_t stage_valid_count = *(uint32_t*)FLASH_VALID_COUNT_START;
  printf("Number valid attempts: %d\r\n", stage_valid_count);
  if (flag == valid_flag){ // 'Valid'
    ota_move_to_main(flash_packet_number);
    ota_flash_erase_sector(FLASH_FLAG_SECTOR); // Reset flags  
    ota_flash_write(FLASH_FLAG_START, (uint8_t*)&done_flag, sizeof(done_flag));
    ota_flash_jump(FLASH_MAIN_START);
  } else if(flag == pending_flag){
    ota_flash_jump(FLASH_STAGING_START);
  } else if (flag == done_flag){ // Finished so we should just be jumping into main
    ota_flash_jump(FLASH_MAIN_START);
  } else if(flag == reset_flag){ // We just reset from main
    HAL_UART_Transmit(&huart1, &ready_response, 1, HAL_MAX_DELAY);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_IWDG_Refresh(&hiwdg);
    if(switch_to_main_flag){
      ota_flash_erase_sector(FLASH_FLAG_SECTOR); // Reset flags  
      ota_flash_write(FLASH_FLAG_START, (uint8_t*)&done_flag, sizeof(done_flag));
      switch_to_main_flag = 0;
      ota_flash_jump(FLASH_MAIN_START);
    }
    if(rx_complete_flag)
    {
      HAL_IWDG_Refresh(&hiwdg);
      // Check end delimiters
      if(rx_buff[TX_END_DELIM_1_INDEX] != TX_END_DELIM_1 || rx_buff[TX_END_DELIM_2_INDEX] != TX_END_DELIM_2)
      {
        printf("Invalid end delimiter!\r\n");
        rx_complete_flag = 0;
      }
      // Little endian here so we need to push the last bytes to the left cuz they are MSB 
      packet_number = rx_buff[0] | (rx_buff[1] << 8);
      total_packets = rx_buff[2];
      packet_CRC =  rx_buff[TX_START_CRC_INDEX] |
                        (rx_buff[TX_START_CRC_INDEX+1] << 8) |
                        (rx_buff[TX_START_CRC_INDEX+2] << 16) |
                        (rx_buff[TX_START_CRC_INDEX+3] << 24);
      // Check the CRC of the packet to see if it matches
      uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&rx_buff[3], TX_DATA_SIZE/4);
      if(crc != packet_CRC){
        last_packet_number = packet_number;
        //Handle error
        printf("CRC Mismatch for packet %u!\r\n", packet_number);
        bad_packet_count++;
        rx_complete_flag = 0;
        if(bad_packet_count >= 3){ // We have had too many failed packet sends here 
          HAL_UART_Transmit(&huart1, &stop_send_ack, 1, HAL_MAX_DELAY); // Stop send ack
          bad_packet_count = 0;
          flash_pointer = FLASH_STAGING_START;
          ota_flash_jump(FLASH_MAIN_START);
          continue; // Leave loop
        }
        HAL_UART_Transmit(&huart1, &nack, 1, HAL_MAX_DELAY); // NACK
        continue;
      }
      if(last_packet_number == packet_number){
        // If we are sending the last packet and we make it past the CRC check reset bad_packet_count
        bad_packet_count = 0;
      }
      printf("Packet #: %u, Total Packets: %u, Incoming CRC: %lu Local CRC: %lu\r\n", packet_number, total_packets, packet_CRC, crc);
      if(packet_number == 0){
        // Start off by erasing the sector
        ota_flash_erase_sector(FLASH_STAGING_SECTOR);
        HAL_IWDG_Refresh(&hiwdg);
      }
      // Start flashing here
      ota_flash_write(flash_pointer, &rx_buff[3], TX_DATA_SIZE);
      HAL_IWDG_Refresh(&hiwdg);
      flash_pointer+=TX_DATA_SIZE;
      // ACK
      HAL_UART_Transmit(&huart1, &ack, 1, HAL_MAX_DELAY);
      rx_complete_flag = 0;
      if(packet_number >= total_packets){
        stage_valid_count = 0;
        // Then we are at final chunk
        flash_pointer = FLASH_STAGING_START;
        HAL_IWDG_Refresh(&hiwdg);
        // Set validity flag to 'pending'
        ota_flash_erase_sector(FLASH_FLAG_SECTOR);
        ota_flash_write(FLASH_FLAG_START, (uint8_t*)&pending_flag, sizeof(pending_flag));
        ota_flash_write(FLASH_SIZE_START, (uint8_t*)&packet_number, sizeof(packet_number)); // Put packet number into flash to use later
        ota_flash_write(FLASH_VALID_COUNT_START, (uint8_t*)&stage_valid_count, sizeof(uint32_t)); // Put packet number into flash to use later
        ota_flash_jump(FLASH_STAGING_START);
      }
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 2000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
     /* Place your implementation of fputc here */
      /* e.g. write a character to the USART1 and Loop until the end of transmission */
      HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
      return ch;
}

GETCHAR_PROTOTYPE
{
     uint8_t ch = 0;
     // Clear the Overrun flag just before receiving the first character
     __HAL_UART_CLEAR_OREFLAG(&huart2);

     HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 5);
     return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    switch(rx_state)
    {
      case WAIT_START_1: // Waiting for first delim
        if(rx_byte == TX_START_DELIM_1)
            rx_state = WAIT_START_2;
        break;
      case WAIT_START_2: // Watiing for second delim
        if(rx_byte == TX_START_DELIM_2)
        {
            rx_index = 0; // Start collecting packet
            rx_state = WAIT_OTA_SEQUENCE;
        }
        else
            rx_state = WAIT_START_1; // Not a valid start
        break;
      case WAIT_OTA_SEQUENCE:
        if(rx_byte == TX_START_OTA_HEX) {
          ota_start_count++;
          if(ota_start_count >= 6) {
            HAL_UART_Transmit(&huart1, &ready_response, 1, HAL_MAX_DELAY);
            rx_state = WAIT_START_1; // Ready for real packets
          }
        } else if(rx_byte == TX_START_BOOTLOADER_HEX){
          ota_main_count++;
          if(ota_main_count >=6){
            switch_to_main_flag = 1;
            rx_state = WAIT_START_1;
          }
        } else {
          // Not OTA start
          rx_index = 0;
          rx_buff[rx_index++] = rx_byte;
          rx_state = RECEIVE_PACKET;
        }
        break;
      case RECEIVE_PACKET: // Start getting data
        rx_buff[rx_index++] = rx_byte;
        if(rx_index >= TX_CHUNK_SIZE - 2) // Already consumed 2 start bytes
        {
            rx_complete_flag = 1; // Packet is ready
            rx_state = WAIT_START_1; // Reset
            rx_index = 0; // Reset buffer
        }
        break;
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
