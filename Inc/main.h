/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <cmsis_os.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

// Work around VS Code Intellisense bug
#ifdef __INTELLISENSE__
#define __FILE_NAME__ __FILE__
#endif

#define DELAY(x) do { if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) osDelay(x); else HAL_Delay(x); } while (0);

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_RTC_Init(void);
void MX_USART3_UART_Init(void);
void MX_DAC1_Init(void);
void MX_ADC1_Init(void);
void MX_CRC_Init(void);
void MX_I2C1_Init(void);
void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
void MX_RNG_Init(void);
void MX_IWDG_Init(void);
void MX_TIM1_Init(void);
void MX_OPAMP1_Init(void);

/* USER CODE BEGIN EFP */

void _Error_Handler(char *file, int line) __attribute__ ((noreturn));
void _Error_Handler2(char *file, int line, HAL_StatusTypeDef status) __attribute__ ((noreturn));
void error_code(int8_t a, int8_t b) __attribute__ ((noreturn));
void SystemClock_Config(void);
void SysClock48(void);
void SysClock72(void);
void SysClock2(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
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

/* USER CODE BEGIN Private defines */
#define USBD_MAX_POWER 0xFAU  /* 500 mA */

// Backup domain registers
#define BKUP_TNC_LOWPOWER_STATE     RTC->BKP0R      /* TNC state when entering low-power mode */
#define BKUP_BT_EEPROM_CRC          RTC->BKP1R         /* Bluetooth module EEPROM CRC */
#define BKUP_MAC_ADDRESS_1          RTC->BKP2R         /* Bluetooth module MAC address OUI part */
#define BKUP_MAC_ADDRESS_2          RTC->BKP3R         /* Bluetooth module MAC address device ID part */
#define BKUP_POWER_CONFIG           RTC->BKP4R         /* Power configuration from EEPROM */

#define TNC_LOWPOWER_SHUTDOWN       0x00000001
#define TNC_LOWPOWER_STOP2          0x00000002
#define TNC_LOWPOWER_STOP1          0x00000004
#define TNC_LOWPOWER_VUSB           0x00000008
#define TNC_LOWPOWER_VBAT           0x00000010
#define TNC_LOWPOWER_LOW_BAT        0x00000020
#define TNC_LOWPOWER_OVP            0x00000040
#define TNC_LOWPOWER_RECONFIG       0x00000080
#define TNC_LOWPOWER_DFU            0x00000100

#define POWER_CONFIG_WAKE_FROM_USB  0x00000001
#define POWER_CONFIG_SLEEP_ON_USB   0x00000002

typedef enum {
	RESET_CAUSE_UNKNOWN,
	RESET_CAUSE_SOFT,	// Software reset
	RESET_CAUSE_HARD,	// Reset button
	RESET_CAUSE_BOR,	// Brown-out reset
	RESET_CAUSE_WUF,	// GPIO wake-up
	RESET_CAUSE_WUTF,	// Timer wake-up
    RESET_CAUSE_IWDG    // Independent watchdog
} ResetCause;

// Compatibility defines
#define BATTERY_ADC_HANDLE hadc1
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_15
#define DEMODULATOR_ADC_HANDLE hadc1
#define DEMODULATOR_ADC_CHANNEL ADC_CHANNEL_8
#define LED_PWM_TIMER_HANDLE htim1
#define SERIAL_UART huart3

#define TNC_HAS_LSCO
#define TNC_HAS_SWO
#define TNC_HAS_LSE
// #define TNC_HAS_HSE
// #define TNC_HAS_MCO
#define TNC_HAS_BT
#define TNC_HAS_BAT
#define TNC_HAS_USB

#define CMD_USB_CDC_CONNECT  (1 << 16)
#define CMD_USB_CDC_DISCONNECT (2 << 16)
#define CMD_POWER_BUTTON_DOWN (3 << 16)
#define CMD_POWER_BUTTON_UP (4 << 16)
#define CMD_BOOT_BUTTON_DOWN (5 << 16)
#define CMD_BOOT_BUTTON_UP (6 << 16)
#define CMD_BT_CONNECT (7 << 16)
#define CMD_BT_DISCONNECT (8 << 16)
#define CMD_SET_PTT_SIMPLEX (9 << 16)
#define CMD_SET_PTT_MULTIPLEX (10 << 16)
#define CMD_SHUTDOWN (11 << 16)
#define CMD_USB_CONNECTED (12 << 16)
#define CMD_USB_CHARGE_ENABLE (13 << 16)
#define CMD_USB_DISCOVERY_COMPLETE (14 << 16)
#define CMD_USB_DISCOVERY_ERROR (15 << 16)
#define CMD_USB_DISCONNECTED (16 << 16)

#define CMD_BT_DEEP_SLEEP (17 << 16)    // disconnected
#define CMD_BT_ACCESS (18 << 16)        // disconnected

#define CMD_BT_TX (19 << 16)            // connected
#define CMD_BT_IDLE (20 << 16)          // connected

#define CMD_RUN (21 << 16)
#define CMD_LPRUN (22 << 16)
#define CMD_SLEEP (23 << 16)
#define CMD_STOP (24 << 16)

#define CMD_USB_SUSPEND (25 << 16)
#define CMD_USB_RESUME (26 << 16)

#define CMD_OVP_ERROR (27 << 16)
#define CMD_NO_OVP_ERROR (28 << 16)

#define CMD_USB_CHARGER_CONNECTED (29 << 16)
#define CMD_USB_HOST_CONNECTED (30 << 16)
#define CMD_USB_HOST_ENUMERATED (31 << 16)

#define CMD_AUDIO_INIT_COMPLETE (32 << 16)

#define CMD_CHECK_BATTERY (33 << 16)
#define CMD_VREFINT_WATCHDOG (34 << 16)
#define CMD_RESTORE_SYSCLK (35 << 16)


extern int reset_requested;
extern char serial_number_64[24];
extern uint8_t mac_address[6];
extern char error_message[80];
extern int go_back_to_sleep;
extern int usb_wake_state;
extern int charging_enabled;
extern int reset_button;
extern int stop_now;
extern uint16_t mobilinkd_model;
extern uint16_t mobilinkd_date_code;
extern uint32_t mobilinkd_serial_number;

#define CxxErrorHandler() _Error_Handler(const_cast<char*>(__FILE_NAME__), __LINE__)
#define CxxErrorHandler2(x) _Error_Handler2(const_cast<char*>(__FILE_NAME__), __LINE__, x)

#define MORSE_0 0x00
#define MORSE_1 0x10
#define MORSE_2 0x18
#define MORSE_3 0x1C
#define MORSE_4 0x1E
#define MORSE_5 0x1F
#define MORSE_6 0x0F
#define MORSE_7 0x07
#define MORSE_8 0x03
#define MORSE_9 0x01

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
