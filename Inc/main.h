/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <cmsis_os.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define EEPROM_ADDRESS 0xA0
#define EEPROM_CAPACITY 4096
#define EEPROM_PAGE_SIZE 32
#define EEPROM_WRITE_TIME 5

#define BT_WAKE_Pin GPIO_PIN_13
#define BT_WAKE_GPIO_Port GPIOC
#define USB_POWER_Pin GPIO_PIN_0
#define USB_POWER_GPIO_Port GPIOH
#define USB_POWER_EXTI_IRQn EXTI0_IRQn
#define SW_POWER_Pin GPIO_PIN_1
#define SW_POWER_GPIO_Port GPIOH
#define SW_POWER_EXTI_IRQn EXTI1_IRQn
#define AUDIO_IN_Pin GPIO_PIN_0
#define AUDIO_IN_GPIO_Port GPIOA
#define BT_SLEEP_Pin GPIO_PIN_1
#define BT_SLEEP_GPIO_Port GPIOA
#define AUDIO_IN_AMP_Pin GPIO_PIN_3
#define AUDIO_IN_AMP_GPIO_Port GPIOA
#define DAC_AUDIO_OUT_Pin GPIO_PIN_4
#define DAC_AUDIO_OUT_GPIO_Port GPIOA
#define DC_OFFSET_Pin GPIO_PIN_5
#define DC_OFFSET_GPIO_Port GPIOA
#define BAT_DIVIDER_Pin GPIO_PIN_7
#define BAT_DIVIDER_GPIO_Port GPIOA
#define BAT_LEVEL_Pin GPIO_PIN_0
#define BAT_LEVEL_GPIO_Port GPIOB
#define AUDIO_ATTEN_Pin GPIO_PIN_2
#define AUDIO_ATTEN_GPIO_Port GPIOB
#define VDD_EN_Pin GPIO_PIN_12
#define VDD_EN_GPIO_Port GPIOB
#define USB_CE_Pin GPIO_PIN_13
#define USB_CE_GPIO_Port GPIOB
#define PTT_B_Pin GPIO_PIN_14
#define PTT_B_GPIO_Port GPIOB
#define PTT_A_Pin GPIO_PIN_15
#define PTT_A_GPIO_Port GPIOB
#define LED_BT_Pin GPIO_PIN_8
#define LED_BT_GPIO_Port GPIOA
#define LED_RX_Pin GPIO_PIN_9
#define LED_RX_GPIO_Port GPIOA
#define LED_TX_Pin GPIO_PIN_10
#define LED_TX_GPIO_Port GPIOA
#define BT_STATE2_Pin GPIO_PIN_4
#define BT_STATE2_GPIO_Port GPIOB
#define BT_STATE2_EXTI_IRQn EXTI4_IRQn
#define BT_STATE1_Pin GPIO_PIN_5
#define BT_STATE1_GPIO_Port GPIOB
#define BT_STATE1_EXTI_IRQn EXTI9_5_IRQn
#define BT_CMD_Pin GPIO_PIN_6
#define BT_CMD_GPIO_Port GPIOB
#define BT_RESET_Pin GPIO_PIN_7
#define BT_RESET_GPIO_Port GPIOB
#define SW_BOOT_Pin GPIO_PIN_3
#define SW_BOOT_GPIO_Port GPIOH
#define SW_BOOT_EXTI_IRQn EXTI3_IRQn

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

/* USER CODE BEGIN Private defines */
#define CMD_USB_CDC_CONNECT  1
#define CMD_USB_CDC_DISCONNECT 2
#define CMD_POWER_BUTTON_DOWN 3
#define CMD_POWER_BUTTON_UP 4
#define CMD_BOOT_BUTTON_DOWN 5
#define CMD_BOOT_BUTTON_UP 6
#define CMD_BT_CONNECT 7
#define CMD_BT_DISCONNECT 8
#define CMD_BT_CONNECT 7
#define CMD_SET_PTT_SIMPLEX 9
#define CMD_SET_PTT_MULTIPLEX 10
#define CMD_SHUTDOWN 11
#define CMD_USB_CONNECTED 12
#define CMD_USB_CHARGE_ENABLE 13
#define CMD_USB_DISCOVERY_COMPLETE 14
#define CMD_USB_DISCOVERY_ERROR 15
#define CMD_USB_DISCONNECTED 16

#define CMD_BT_DEEP_SLEEP 17    // disconnected
#define CMD_BT_ACCESS 18        // disconnected

#define CMD_BT_TX 19            // connected
#define CMD_BT_IDLE 20          // connected

#define CMD_RUN 21
#define CMD_LPRUN 22
#define CMD_SLEEP 23
#define CMD_STOP 24

#define CMD_USB_SUSPEND 25
#define CMD_USB_RESUME 26

extern int reset_requested;
extern char serial_number_64[17];
extern uint8_t mac_address[6];
extern char error_message[80];
extern int go_back_to_sleep;
extern int usb_wake_state;
extern int charging_enabled;
extern int reset_button;
extern osMutexId hardwareInitMutexHandle;

#define CxxErrorHandler() _Error_Handler(const_cast<char*>(__FILE__), __LINE__)

#ifdef __cplusplus
 extern "C" {
#endif

void SysClock48(void);
void SysClock80(void);
void SysClock4(void);

#ifdef __cplusplus
}
#endif

#define SystemClock_Config_48MHz SystemClock_Config

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
