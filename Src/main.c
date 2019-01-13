
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_core.h"
#include "IOEventTask.h"
#include "PortInterface.h"
#include "LEDIndicator.h"
#include "bm78.h"
#include "base64.h"
#include "KissHardware.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

IWDG_HandleTypeDef hiwdg;

OPAMP_HandleTypeDef hopamp1;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 256 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId ioEventTaskHandle;
uint32_t ioEventTaskBuffer[ 384 ];
osStaticThreadDef_t ioEventTaskControlBlock;
osThreadId ledBlinkerHandle;
uint32_t ledBlinkerBuffer[ 128 ];
osStaticThreadDef_t ledBlinkerControlBlock;
osThreadId audioInputTaskHandle;
uint32_t audioInputTaskBuffer[ 512 ];
osStaticThreadDef_t audioInputTaskControlBlock;
osThreadId modulatorTaskHandle;
uint32_t modulatorTaskBuffer[ 384 ];
osStaticThreadDef_t modulatorTaskControlBlock;
osMessageQId ioEventQueueHandle;
uint8_t ioEventQueueBuffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t ioEventQueueControlBlock;
osMessageQId serialInputQueueHandle;
uint8_t serialInputQueueBuffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t serialInputQueueControlBlock;
osMessageQId serialOutputQueueHandle;
uint8_t serialOutputQueueBuffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t serialOutputQueueControlBlock;
osMessageQId audioInputQueueHandle;
uint8_t audioInputQueueBuffer[ 4 * sizeof( uint8_t ) ];
osStaticMessageQDef_t audioInputQueueControlBlock;
osMessageQId hdlcInputQueueHandle;
uint8_t hdlcInputQueueBuffer[ 3 * sizeof( uint32_t ) ];
osStaticMessageQDef_t hdlcInputQueueControlBlock;
osMessageQId hdlcOutputQueueHandle;
uint8_t hdlcOutputQueueBuffer[ 3 * sizeof( uint32_t ) ];
osStaticMessageQDef_t hdlcOutputQueueControlBlock;
osMessageQId dacOutputQueueHandle;
uint8_t dacOutputQueueBuffer[ 128 * sizeof( uint8_t ) ];
osStaticMessageQDef_t dacOutputQueueControlBlock;
osMessageQId adcInputQueueHandle;
uint8_t adcInputQueueBuffer[ 3 * sizeof( uint32_t ) ];
osStaticMessageQDef_t adcInputQueueControlBlock;
osTimerId beaconTimer1Handle;
osStaticTimerDef_t beaconTimer1ControlBlock;
osTimerId beaconTimer2Handle;
osStaticTimerDef_t beaconTimer2ControlBlock;
osTimerId beaconTimer3Handle;
osStaticTimerDef_t beaconTimer3ControlBlock;
osTimerId beaconTimer4Handle;
osStaticTimerDef_t beaconTimer4ControlBlock;
osTimerId usbShutdownTimerHandle;
osStaticTimerDef_t usbShutdownTimerControlBlock;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int lost_power = 0;
int reset_requested = 0;
char serial_number_64[17] = {0};
// Make sure it is not overwritten during resets (bss3).
uint8_t mac_address[6] __attribute__((section(".bss3"))) = {0};
char error_message[80] __attribute__((section(".bss3"))) = {0};
// USB power control -- need to renegotiate USB charging in STOP mode.
int go_back_to_sleep __attribute__((section(".bss3")));
int stop_now __attribute__((section(".bss3")));
int charging_enabled __attribute__((section(".bss3")));
int usb_wake_state __attribute__((section(".bss3")));

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_RNG_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_OPAMP1_Init(void);
void StartDefaultTask(void const * argument);
extern void startIOEventTask(void const * argument);
extern void startLedBlinkerTask(void const * argument);
extern void startAudioInputTask(void const * argument);
extern void startModulatorTask(void const * argument);
extern void beacon(void const * argument);
extern void shutdown(void const * argument);
void encode_serial_number(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void stop2(void) __attribute__((noinline));
void configure_gpio_for_stop(void) __attribute__((noinline));
void power_down_vdd(void);
void power_up_vdd(void);
void configure_wakeup_gpio(void);
void enable_debug_gpio(void);
void init_rtc_date_time(void);
void init_rtc_alarm(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

extern PCD_HandleTypeDef hpcd_USB_FS;


void configure_gpio_for_stop()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_NVIC_DisableIRQ(EXTI3_IRQn);

    // BT_STATE1
    HAL_GPIO_DeInit(BT_STATE1_GPIO_Port, BT_STATE1_Pin);
    HAL_NVIC_DisableIRQ(BT_STATE1_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(BT_STATE1_EXTI_IRQn);

    // BT_STATE2
    HAL_GPIO_DeInit(BT_STATE2_GPIO_Port, BT_STATE2_Pin);
    HAL_NVIC_DisableIRQ(BT_STATE2_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(BT_STATE2_EXTI_IRQn);

    // SW_BOOT
    HAL_GPIO_DeInit(SW_BOOT_GPIO_Port, SW_BOOT_Pin);
    HAL_NVIC_DisableIRQ(SW_BOOT_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(SW_BOOT_EXTI_IRQn);

    // LEDs
    HAL_GPIO_DeInit(GPIOA, LED_BT_Pin|LED_TX_Pin|LED_RX_Pin);

    // I2C
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    // USB
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    // UART
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

    // Battery level circuit.
    HAL_GPIO_DeInit(BAT_LEVEL_GPIO_Port, BAT_LEVEL_Pin);
    HAL_GPIO_DeInit(BAT_DIVIDER_GPIO_Port, BAT_DIVIDER_Pin);

    if (charging_enabled)
    {
        HAL_GPIO_WritePin(GPIOB, USB_CE_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_DeInit(USB_CE_GPIO_Port, USB_CE_Pin);  // Hi-Z
    }

    // Bluetooth module
    HAL_GPIO_DeInit(GPIOC, BT_WAKE_Pin);
    HAL_GPIO_DeInit(GPIOB, BT_RESET_Pin|BT_CMD_Pin);
    HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);
    HAL_Delay(250);

    // Analog pins
    HAL_GPIO_DeInit(GPIOA, AUDIO_IN_Pin|AUDIO_IN_AMP_Pin|DAC_AUDIO_OUT_Pin|DC_OFFSET_Pin);
    HAL_GPIO_DeInit(GPIOB, AUDIO_ATTEN_Pin);

    // PTT pins
    HAL_GPIO_DeInit(GPIOB, PTT_A_Pin|PTT_B_Pin);
}

void power_down_vdd()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_RESET);
    for (int i = 0; i < 4800; ++i) asm volatile("nop");
}

void power_up_vdd()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = VDD_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(VDD_EN_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_SET);
}

void configure_wakeup_gpio()
{
    if (!__HAL_RCC_GPIOH_IS_CLK_ENABLED()) Error_Handler();

    GPIO_InitTypeDef GPIO_InitStruct;

    // Reset wakeup pins
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    HAL_GPIO_DeInit(GPIOH, USB_POWER_Pin|SW_POWER_Pin);

    // Wake up whenever there is a change in VUSB to handle connect/disconnect events.
    GPIO_InitStruct.Pin = USB_POWER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;   // needed to act as a voltage divider
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    // Only wake up after the button has been released.  This avoids the case
    // where the TNC is woken up on button down and then immediately put back
    // to sleep when the BUTTON_UP interrupt is received.
    GPIO_InitStruct.Pin = SW_POWER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

void enable_debug_gpio()
{
    if (!__HAL_RCC_GPIOA_IS_CLK_ENABLED()) Error_Handler();
    if (!__HAL_RCC_GPIOB_IS_CLK_ENABLED()) Error_Handler();

    GPIO_InitTypeDef GPIO_InitStruct;

    // DEBUG PINS
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF0_SWJ;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // USART CTS is connected to a device on VDD
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * Shutdown is used to enter stop mode in a clean state.  This ensures that
 * all IP have been reset & re-initialized to their default state when
 * entering low-power stop mode.  This is a work-around until we can
 * determine what causes a high-discharge state after USB is enabled.
 *
 * @param argument is unused.
 */
void shutdown(void const * argument)
{
    UNUSED(argument);
    stop_now = 1;
    HAL_NVIC_SystemReset();
}

/*
 * Same algorithm as here: https://github.com/libopencm3/libopencm3/blob/master/lib/stm32/desig.c
 */
void encode_serial_number()
{
    uint8_t *uid = (uint8_t *)UID_BASE;

    uint8_t serial[6];
    serial[0] = uid[11];
    serial[1] = uid[10] + uid[2];
    serial[2] = uid[9];
    serial[3] = uid[8] + uid[0];
    serial[4] = uid[7];
    serial[5] = uid[6];

    snprintf(
        serial_number_64,
        sizeof(serial_number_64),
        "%02X%02X%02X%02X%02X%02X",
        serial[0], serial[1], serial[2],
        serial[3], serial[4], serial[5]
    );
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // If not a software reset, reset the flags.  This prevents odd behavior
  // during initial power on and hardware resets where SRAM2 may be in an
  // inconsistent state.  During a soft reset, it should be initialized.
  if (!(RCC->CSR & RCC_CSR_SFTRSTF)) {
      go_back_to_sleep = 0;
      stop_now = 0;
      usb_wake_state = 0;
  }
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#ifdef KISS_LOGGING
  printf("start\r\n");
#endif

  // Note that it is important that all GPIO interrupts are disabled until
  // the FreeRTOS kernel has started.  All GPIO interrupts  send messages
  // to the ioEventTask thread.  Attempts to use any message queues before
  // FreeRTOS has started will lead to problems.  Because of this, these
  // interrupts are enabled only when the ioEventTask thread starts.

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_OPAMP1_Init();
  /* USER CODE BEGIN 2 */
  if (stop_now) stop2();

  MX_TIM1_Init();           // Initialize the LED PWM timer and GPIOs.
  SCB->SHCSR |= 0x70000;    // Enable fault handlers;
  if (!go_back_to_sleep) {
      indicate_turning_on();    // LEDs on during boot.
  }

  encode_serial_number();

  if (!go_back_to_sleep) {
      // The Bluetooth module is powered on during MX_GPIO_Init().  BT_CMD
      // has a weak pull-up on the BT module and is in OD mode.  Pull the
      // pin low during boot to enter Bluetooth programming mode.  Here the
      // BT_CMD pin is switched to input mode to detect the state.  The
      // TNC must be reset to exit programming mode.

      // Wait for BT module to settle.
      GPIO_InitTypeDef GPIO_InitStructure;

      GPIO_InitStructure.Pin = BT_CMD_Pin;
      GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
      GPIO_InitStructure.Pull = GPIO_PULLUP;
      HAL_GPIO_Init(BT_CMD_GPIO_Port, &GPIO_InitStructure);
      HAL_Delay(10);

      if (HAL_GPIO_ReadPin(BT_CMD_GPIO_Port, BT_CMD_Pin) == GPIO_PIN_RESET) {
          // Special test mode for programming the Bluetooth module.  The TNC
          // has the BT_CMD pin actively being pulled low.  In this case we
          // power on the BT module with BT_CMD held low and wait here without
          // initializing the UART.  We only exit via reset.
          HAL_UART_MspDeInit(&huart3);

          HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_RESET);
          HAL_Delay(1);
          HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_SET);
          HAL_Delay(200);

          printf("Bluetooth programming mode\r\n");

          while (1);
      }

      // Not in BT programming mode.  Switch BT_CMD back to OD mode.
      HAL_GPIO_WritePin(BT_CMD_GPIO_Port, BT_CMD_Pin, GPIO_PIN_SET);
      GPIO_InitStructure.Pin = BT_CMD_Pin;
      GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
      GPIO_InitStructure.Pull = GPIO_PULLUP;
      HAL_GPIO_Init(BT_CMD_GPIO_Port, &GPIO_InitStructure);
  }

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of beaconTimer1 */
  osTimerStaticDef(beaconTimer1, beacon, &beaconTimer1ControlBlock);
  beaconTimer1Handle = osTimerCreate(osTimer(beaconTimer1), osTimerPeriodic, NULL);

  /* definition and creation of beaconTimer2 */
  osTimerStaticDef(beaconTimer2, beacon, &beaconTimer2ControlBlock);
  beaconTimer2Handle = osTimerCreate(osTimer(beaconTimer2), osTimerPeriodic, NULL);

  /* definition and creation of beaconTimer3 */
  osTimerStaticDef(beaconTimer3, beacon, &beaconTimer3ControlBlock);
  beaconTimer3Handle = osTimerCreate(osTimer(beaconTimer3), osTimerPeriodic, NULL);

  /* definition and creation of beaconTimer4 */
  osTimerStaticDef(beaconTimer4, beacon, &beaconTimer4ControlBlock);
  beaconTimer4Handle = osTimerCreate(osTimer(beaconTimer4), osTimerPeriodic, NULL);

  /* definition and creation of usbShutdownTimer */
  osTimerStaticDef(usbShutdownTimer, shutdown, &usbShutdownTimerControlBlock);
  usbShutdownTimerHandle = osTimerCreate(osTimer(usbShutdownTimer), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 256, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ioEventTask */
  osThreadStaticDef(ioEventTask, startIOEventTask, osPriorityLow, 0, 384, ioEventTaskBuffer, &ioEventTaskControlBlock);
  ioEventTaskHandle = osThreadCreate(osThread(ioEventTask), NULL);

  /* definition and creation of ledBlinker */
  osThreadStaticDef(ledBlinker, startLedBlinkerTask, osPriorityIdle, 0, 128, ledBlinkerBuffer, &ledBlinkerControlBlock);
  ledBlinkerHandle = osThreadCreate(osThread(ledBlinker), NULL);

  /* definition and creation of audioInputTask */
  osThreadStaticDef(audioInputTask, startAudioInputTask, osPriorityAboveNormal, 0, 512, audioInputTaskBuffer, &audioInputTaskControlBlock);
  audioInputTaskHandle = osThreadCreate(osThread(audioInputTask), NULL);

  /* definition and creation of modulatorTask */
  osThreadStaticDef(modulatorTask, startModulatorTask, osPriorityAboveNormal, 0, 384, modulatorTaskBuffer, &modulatorTaskControlBlock);
  modulatorTaskHandle = osThreadCreate(osThread(modulatorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of ioEventQueue */
  osMessageQStaticDef(ioEventQueue, 16, uint32_t, ioEventQueueBuffer, &ioEventQueueControlBlock);
  ioEventQueueHandle = osMessageCreate(osMessageQ(ioEventQueue), NULL);

  /* definition and creation of serialInputQueue */
  osMessageQStaticDef(serialInputQueue, 16, uint32_t, serialInputQueueBuffer, &serialInputQueueControlBlock);
  serialInputQueueHandle = osMessageCreate(osMessageQ(serialInputQueue), NULL);

  /* definition and creation of serialOutputQueue */
  osMessageQStaticDef(serialOutputQueue, 16, uint32_t, serialOutputQueueBuffer, &serialOutputQueueControlBlock);
  serialOutputQueueHandle = osMessageCreate(osMessageQ(serialOutputQueue), NULL);

  /* definition and creation of audioInputQueue */
  osMessageQStaticDef(audioInputQueue, 4, uint8_t, audioInputQueueBuffer, &audioInputQueueControlBlock);
  audioInputQueueHandle = osMessageCreate(osMessageQ(audioInputQueue), NULL);

  /* definition and creation of hdlcInputQueue */
  osMessageQStaticDef(hdlcInputQueue, 3, uint32_t, hdlcInputQueueBuffer, &hdlcInputQueueControlBlock);
  hdlcInputQueueHandle = osMessageCreate(osMessageQ(hdlcInputQueue), NULL);

  /* definition and creation of hdlcOutputQueue */
  osMessageQStaticDef(hdlcOutputQueue, 3, uint32_t, hdlcOutputQueueBuffer, &hdlcOutputQueueControlBlock);
  hdlcOutputQueueHandle = osMessageCreate(osMessageQ(hdlcOutputQueue), NULL);

  /* definition and creation of dacOutputQueue */
  osMessageQStaticDef(dacOutputQueue, 128, uint8_t, dacOutputQueueBuffer, &dacOutputQueueControlBlock);
  dacOutputQueueHandle = osMessageCreate(osMessageQ(dacOutputQueue), NULL);

  /* definition and creation of adcInputQueue */
  osMessageQStaticDef(adcInputQueue, 3, uint32_t, adcInputQueueBuffer, &adcInputQueueControlBlock);
  adcInputQueueHandle = osMessageCreate(osMessageQ(adcInputQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE BEGIN RTOS_QUEUES */

  // Initialize the DC offset DAC and the PGA op amp.  Calibrate the ADC.
  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1024) != HAL_OK) Error_Handler();
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_OPAMP_SelfCalibrate(&hopamp1) != HAL_OK) Error_Handler();
  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) Error_Handler();
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();

  if (!go_back_to_sleep) {

      // Initialize the BM78 Bluetooth module and the RTC date/time the first time we boot.
      if (!bm78_initialized()) {
          bm78_initialize();
          memset(error_message, 0, sizeof(error_message));
          // init_rtc_date_time();
      }
      else bm78_wait_until_ready();
  }

  init_ioport();
  initCDC();
  initSerial();

  // Initialize option bytes.
  FLASH_OBProgramInitTypeDef obInit = {0};
  HAL_FLASHEx_OBGetConfig(&obInit);

  if ((obInit.OptionType & OPTIONBYTE_USER) == RESET) {
    printf("FAIL: option byte init\r\n");
    Error_Handler();
  }

#if 1
  // Do not erase SRAM2 during reset.
  if ((obInit.USERConfig & FLASH_OPTR_SRAM2_RST) == RESET) {
    obInit.OptionType = OPTIONBYTE_USER;
    obInit.USERType = OB_USER_SRAM2_RST;
    obInit.USERConfig = FLASH_OPTR_SRAM2_RST;
    HAL_FLASH_OB_Unlock();
    HAL_FLASHEx_OBProgram(&obInit);
    HAL_FLASH_OB_Lock();
    HAL_FLASH_OB_Launch();
  }
#endif

#if 1
  // Enable hardware parity check on SRAM2
  if ((obInit.USERConfig & FLASH_OPTR_SRAM2_PE) == RESET) {
    obInit.OptionType = OPTIONBYTE_USER;
    obInit.USERType = OB_USER_SRAM2_PE;
    obInit.USERConfig = FLASH_OPTR_SRAM2_PE;
    HAL_FLASH_OB_Unlock();
    HAL_FLASHEx_OBProgram(&obInit);
    HAL_FLASH_OB_Lock();
    HAL_FLASH_OB_Launch();
  }
#endif

  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  RCC_CRSInitTypeDef RCC_CRSInitStruct;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

#ifdef KISS_LOGGING
  HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#endif

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

    /**Enable the SYSCFG APB clock 
    */
  __HAL_RCC_CRS_CLK_ENABLE();

    /**Configures CRS 
    */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 4129;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InitValue = 0xFFFF;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20000209;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**I2C Fast mode Plus enable 
    */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* OPAMP1 init function */
static void MX_OPAMP1_Init(void)
{

  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_HIGH;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_CONNECT_NO;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RNG init function */
static void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */


  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  init_rtc_date_time();
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1817;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1817;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
     PA2   ------> RCC_LSCO
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_WAKE_GPIO_Port, BT_WAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BT_SLEEP_Pin|BAT_DIVIDER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AUDIO_ATTEN_Pin|VDD_EN_Pin|USB_CE_Pin|BT_CMD_Pin 
                          |BT_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PTT_B_Pin|PTT_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BT_WAKE_Pin */
  GPIO_InitStruct.Pin = BT_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BT_WAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_POWER_Pin SW_BOOT_Pin */
  GPIO_InitStruct.Pin = SW_POWER_Pin|SW_BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_POWER_Pin */
  GPIO_InitStruct.Pin = USB_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_SLEEP_Pin */
  GPIO_InitStruct.Pin = BT_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BT_SLEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_DIVIDER_Pin */
  GPIO_InitStruct.Pin = BAT_DIVIDER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BAT_DIVIDER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AUDIO_ATTEN_Pin BT_CMD_Pin BT_RESET_Pin USB_CE_Pin*/
  GPIO_InitStruct.Pin = AUDIO_ATTEN_Pin|BT_CMD_Pin|BT_RESET_Pin|USB_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : VDD_EN_Pin PTT_B_Pin PTT_A_Pin */
  GPIO_InitStruct.Pin = VDD_EN_Pin|PTT_B_Pin|PTT_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_STATE2_Pin BT_STATE1_Pin */
  GPIO_InitStruct.Pin = BT_STATE2_Pin|BT_STATE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void stop2()
{
  osThreadSuspendAll();

  int usb_stop_state = HAL_GPIO_ReadPin(USB_POWER_GPIO_Port, USB_POWER_Pin);

  HAL_OPAMP_DeInit(&hopamp1);
  HAL_TIM_PWM_DeInit(&htim1);
  HAL_I2C_DeInit(&hi2c1);
  HAL_ADC_DeInit(&hadc1);
  HAL_DAC_DeInit(&hdac1);
  HAL_UART_DeInit(&huart3);

  HAL_PWR_DisablePVD();
  USB->BCDR = 0;
  HAL_PWREx_DisableVddUSB();
  HAL_ADCEx_EnterADCDeepPowerDownMode(&hadc1);
  configure_gpio_for_stop();
  if (!usb_stop_state) power_down_vdd();

  HAL_RCCEx_DisableLSCO();

  configure_wakeup_gpio();

  __asm volatile ( "cpsid i" );
  __asm volatile ( "dsb" );
  __asm volatile ( "isb" );

  go_back_to_sleep = 0;
  stop_now = 0;

  HAL_PWREx_DisableLowPowerRunMode();
  HAL_DBGMCU_DisableDBGStopMode();
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFE);

  // Powered off state
  // When awakened by USB_POWER pin change:
  // If unplugged, re-init IO, disabling charging, then go back to STOP.
  // If plugged, re-init IO, do charger detection then,
  // If powerOnViaUSB(), stay awake, otherwise go back to STOP.
  usb_wake_state = HAL_GPIO_ReadPin(USB_POWER_GPIO_Port, USB_POWER_Pin);
  go_back_to_sleep = (usb_stop_state != usb_wake_state);
  if (usb_wake_state) {
      if (powerOnViaUSB()) {
          go_back_to_sleep = 0;
      }
  } else {
      charging_enabled = 0;
  }
  HAL_NVIC_SystemReset();
}

#if 1
long _write_r(struct _reent *r, int fd, const char *ptr, int len);

long _write_r(struct _reent *r, int fd, const char *ptr, int len)
{
  UNUSED(r);
  UNUSED(fd);
#ifdef KISS_LOGGING
    for (int i = 0; i != len; ++i)
      ITM_SendChar(ptr[i]);
#endif
  return len;
}

int _write(int file, char *ptr, int len);

int _write(int file, char *ptr, int len) {
    UNUSED(file);
#ifdef KISS_LOGGING
    for (int i = 0; i != len; ++i)
      ITM_SendChar(ptr[i]);
#endif
    return len;

}
#endif

void init_rtc_date_time()
{
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0) return;

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    /**Initialize RTC and set the Time and Date
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sTime.TimeFormat = RTC_HOURFORMAT_24;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void init_rtc_alarm()
{
    RTC_AlarmTypeDef sAlarm;

    /**Enable the Alarm A
    */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enable the Alarm B
    */
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_B;

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  UNUSED(argument);

  if (HAL_GPIO_ReadPin(USB_POWER_GPIO_Port, USB_POWER_Pin) == GPIO_PIN_SET)
  {
#ifdef KISS_LOGGING
    printf("VBUS detected\r\n");
#endif
    MX_USB_DEVICE_Init();
    HAL_PCD_MspInit(&hpcd_USB_FS);
    HAL_PCDEx_ActivateBCD(&hpcd_USB_FS);
    HAL_PCDEx_BCD_VBUSDetect(&hpcd_USB_FS);
  } else {
#ifdef KISS_LOGGING
    printf("VBUS not detected\r\n");
#endif
  }
/* Infinite loop */
  for(;;)
  {
    osDelay(osWaitForever);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM1) {
      HTIM1_PeriodElapsedCallback();
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
#ifdef KISS_LOGGING
  printf("Error handler called from file %s on line %d\r\n", file, line);
#endif
  snprintf(error_message, sizeof(error_message), "Error: %s:%d", file, line);

  stop_now = 0;
  go_back_to_sleep = 0;
  NVIC_SystemReset();
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
