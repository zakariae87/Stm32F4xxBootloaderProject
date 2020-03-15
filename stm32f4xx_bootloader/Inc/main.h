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
uint16_t get_mcu_chip_id(void);
uint8_t get_flash_rdp_level(void);

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_getcid_cmd(uint8_t *pBuffer);
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer);

/*Bootloader commands -----------------------------------------------*/

//Pattern command
//<command name >	<command_code>

/* This command is used to read the bootloader version from the MCU */
#define BL_GET_VER							0x51
//This command is used to know what are the commands supported by the bootloader
#define BL_GET_HELP							0x52

//This command is used to read the MCU chip identification number
#define BL_GET_CID							0x53

//This command is used to read the FLASH Read Protection level.
#define BL_GET_RDP_STATUS				0x54

//This command is used to jump bootloader to specified address.
#define BL_GO_TO_ADDR						0x55

//This command is used to mass erase or sector erase of the user flash .
#define BL_FLASH_ERASE          0x56

//This command is used to write data in to different memories of the MCU
#define BL_MEM_WRITE						0x57

//This command is used to enable or disable read/write protect on different sectors of the user flash .
#define BL_EN_R_W_PROTECT				0x58

//This command is used to read data from different memories of the microcontroller.
#define BL_MEM_READ							0x59

//This command is used to read all the sector protection status.
#define BL_READ_SECTOR_P_STATUS	0x5A

//This command is used to read the OTP contents.
#define BL_OTP_READ							0x5B

////This command is used to disable read/write protect on different sectors of the user flash .
#define BL_DIS_R_W_PROTECT			0x5C

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
