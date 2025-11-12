/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void uart_printf (const char *fmt, ...);
void BL_send_ack(uint8_t command_code, uint8_t len);
void BL_send_nack(void);
void BL_UART_read_data(void);
uint8_t BL_CRC_check(uint8_t *pData, uint32_t len, uint32_t crc_host);
void userApp(void);
uint8_t flash_erase(uint8_t sector_num, uint8_t num_of_sectors);
/*Bootloader handler function prototypes*/
void BL_getver_handler(uint8_t *rx_buffer);
void BL_getrdp_handler(uint8_t *rx_buffer);
void BL_flash_erase_handler(uint8_t *rx_buffer);
void BL_mem_write_handler(uint8_t *rx_buffer);
void BL_control_rw_protect_handler(uint8_t *rx_buffer);
void BL_mem_read_handler(uint8_t *rx_buffer);
void BL_read_sector_status_handler(uint8_t *rx_buffer);
void BL_read_otp_handler(uint8_t *rx_buffer);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U
#define BL_RX_SIZE	200

#define BL_VERSION	0x10

#define BL_ACK 		0xA5
#define BL_NACK		0x7F

#define VERIFY_CRC_SUCCESS 	0
#define VERIFY_CRC_FAIL		1
/*Bootloader command codes*/
# define BL_GET_VER				0x51 // command used to read the bootloader version for the MCU
# define BL_GET_RDP_STATUS		0x54 // command to read the flash read protection privelege
# define BL_FLASH_ERASE			0x56 //command to erase all or sections of the user flash
# define BL_MEM_WRITE			0x57 //command used to write data into different memories of the MCU
# define BL_MEM_READ			0x58 //command used to read data from different memories of the MCU
# define BL_CONTROL_RW_PROTECT	0x59 //command used to enable or disable read/write priveleges on different sectors of the MCU
# define BL_READ_SECTOR_STATUS	0x60 //command used to read privelegs on sectors
# define BL_OTP_READ			0x61 // command used to read OTP



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
