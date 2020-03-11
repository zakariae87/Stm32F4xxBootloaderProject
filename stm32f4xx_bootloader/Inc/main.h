/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
#define BOOT_USER_LED_Pin 					GPIO_PIN_6
#define BOOT_USER_LED_GPIO_Port 		GPIOF

#define BOOT_USER_BUTTON_Pin 				GPIO_PIN_0
#define BOOT_USER_BUTTON_Port  			GPIOA

/* Application flash address */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U

/* version 1.0 */
#define BL_VERSION 0x10


/* ACK and NACK bytes*/
#define BL_ACK   0XA5
#define BL_NACK  0X7F

/*CRC Macros states*/
#define VERIFY_CRC_FAIL    1
#define VERIFY_CRC_SUCCESS 0


/*Bootloader function prototypes -----------------------------------------------*/
void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t lenght);
uint8_t get_bootloader_version(void);
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host);



void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);


/*Bootloader commands -----------------------------------------------*/

//Pattern command
//<command name >	<command_code>

/* This command is used to read the bootloader version from the MCU */
#define BL_GET_VER				0x51



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
