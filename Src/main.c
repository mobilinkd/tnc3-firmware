/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_core.h"
#include "IOEventTask.h"
#include "PortInterface.h"
#include "LEDIndicator.h"
#include "bm78.h"
#include "KissHardware.h"
#include "Log.h"
#include "power.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

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

osThreadId ioEventTaskHandle;
uint32_t ioEventTaskBuffer[ 384 ];
osStaticThreadDef_t ioEventTaskControlBlock;
osThreadId audioInputTaskHandle;
uint32_t audioInputTaskBuffer[ 512 ];
osStaticThreadDef_t audioInputTaskControlBlock;
osThreadId modulatorTaskHandle;
uint32_t modulatorTaskBuffer[ 384 ];
osStaticThreadDef_t modulatorTaskControlBlock;
osMessageQId ioEventQueueHandle;
uint8_t ioEventQueueBuffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t ioEventQueueControlBlock;
osMessageQId audioInputQueueHandle;
uint8_t audioInputQueueBuffer[ 8 * sizeof( uint8_t ) ];
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
osTimerId usbShutdownTimerHandle;
osStaticTimerDef_t usbShutdownTimerControlBlock;
osTimerId powerOffTimerHandle;
osStaticTimerDef_t powerOffTimerControlBlock;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int lost_power = 0;
int reset_requested = 0;
char serial_number_64[24] = {0};
// Make sure it is not overwritten during resets (bss3).
uint8_t mac_address[6] __attribute__((section(".bss3"))) = {0};
char error_message[80] __attribute__((section(".bss3"))) = {0};
// USB power control -- need to renegotiate USB charging in STOP mode.
int go_back_to_sleep __attribute__((section(".bss3")));
int charging_enabled __attribute__((section(".bss3")));
int usb_wake_state __attribute__((section(".bss3")));
int reset_button = 0;

uint16_t mobilinkd_model;
uint16_t mobilinkd_date_code;
uint32_t mobilinkd_serial_number;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
void startIOEventTask(void const * argument);
extern void startAudioInputTask(void const * argument);
extern void startModulatorTask(void const * argument);
void shutdown(void const * argument);
void powerOffTimerCallback(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
        "%02X%02X%02X%02X%02X%02X (%04lu)",
        serial[0], serial[1], serial[2],
        serial[3], serial[4], serial[5],
        mobilinkd_serial_number
    );
}

static ResetCause getResetCause()
{
  uint32_t wakeEvent = PWR->SR1 & 0x1F; // Wake-up event.

  // Capture cause of reset/wake-up.
  ResetCause resetCause = RESET_CAUSE_UNKNOWN;
  if (RCC->CSR & RCC_CSR_SFTRSTF) {
      resetCause = RESET_CAUSE_SOFT;
  } else if (RCC->CSR & RCC_CSR_IWDGRSTF) {
      resetCause = RESET_CAUSE_IWDG;
  } else if (RCC->CSR & RCC_CSR_PINRSTF) {
      resetCause = RESET_CAUSE_HARD;
      reset_button = 1;
  } else if (RCC->CSR & RCC_CSR_BORRSTF) {
      resetCause = RESET_CAUSE_BOR;
  } else if (wakeEvent) {
      resetCause = RESET_CAUSE_WUF;
  } else if (__HAL_RTC_WAKEUPTIMER_GET_FLAG(&hrtc, RTC_FLAG_WUTF) != RESET) {
      resetCause = RESET_CAUSE_WUTF;
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

  return resetCause;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  __HAL_DBGMCU_FREEZE_IWDG();

  // Capture cause of reset/wake-up.
  ResetCause resetCause = getResetCause();

  // Read serial, model, date from OTP record. Values are big-endian.
  mobilinkd_serial_number = __builtin_bswap32(*(uint32_t*) (0x1FFF7000));
  mobilinkd_model = __builtin_bswap16(*(uint16_t*) (0x1FFF7004));
  mobilinkd_date_code = __builtin_bswap16(*(uint16_t*) (0x1FFF7006));

  if (mobilinkd_serial_number == 0xFFFFFFFF) mobilinkd_serial_number = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // SysClock starts at 16MHz and we want a 2MHz SWO.
  TPI->ACPR = 7;

  // Note that it is important that all GPIO interrupts are disabled until
  // the FreeRTOS kernel has started.  All GPIO interrupts send messages
  // to the ioEventTask thread.  Attempts to use any message queues before
  // FreeRTOS has started will lead to problems.  Because of this, GPIO
  // interrupts are not enabled until the ioEventTask thread starts.

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  GPIO_PinState power_switch_state = !!(SW_POWER_GPIO_Port->IDR & SW_POWER_Pin);
  // Log the cause of the reset.
  switch (resetCause) {
  case RESET_CAUSE_SOFT:
    INFO("software reset");
    break;
  case RESET_CAUSE_IWDG:
    INFO("watchdog reset");
    // Reset the BKUP_TNC_LOWPOWER_STATE register to ensure the TNC wakes
    // after a watchdog reset.
    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, 0x0);
    HAL_PWR_DisableBkUpAccess();
    break;
  case RESET_CAUSE_HARD:
    INFO("hardware reset");
    // Reset the BKUP_TNC_LOWPOWER_STATE register to ensure the TNC wakes
    // after a hardware reset.
    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, 0x0);
    HAL_PWR_DisableBkUpAccess();
    break;
  case RESET_CAUSE_BOR:
    INFO("brown-out reset");
    // Reset the BKUP_TNC_LOWPOWER_STATE register to ensure the TNC wakes
    // after a brown-out reset.
    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, 0x0);
    HAL_PWR_DisableBkUpAccess();
    break;
  case RESET_CAUSE_WUF:
    INFO("wake-up event: %02lx", PWR->SR1 & 0x1F);
    break;
  case RESET_CAUSE_WUTF:
    INFO("wake-up timer");
    break;
  default:
    INFO("unknown reset, RCC->CSR=0x%08lx", RCC->CSR);
    INFO("unknown reset, RTC->SR=0x%08lx", RTC->ISR);
    INFO("unknown reset, PWR->SR1=0x%08lx", PWR->SR1);
  }
  INFO("PWR->SCR=0x%08lx", PWR->SCR);
  INFO("PWR->CR1=0x%08lx", PWR->CR1);
  __HAL_RCC_CLEAR_RESET_FLAGS();
  uint32_t shutdown_reg = READ_REG(BKUP_TNC_LOWPOWER_STATE);
  // Reset the BKUP_TNC_LOWPOWER_STATE register.
  HAL_PWR_EnableBkUpAccess();
  WRITE_REG(BKUP_TNC_LOWPOWER_STATE, 0x0);
  HAL_PWR_DisableBkUpAccess();

  if (shutdown_reg & TNC_LOWPOWER_DFU) {
    // DFU leaves the system in a bad state. This starts clean.
    HAL_NVIC_SystemReset();
  }

  // TNC_LOWPOWER_RECONFIG is used to negotiate USB power when the device
  // is connected to USB while in a low-power state.
  go_back_to_sleep = !!(shutdown_reg & TNC_LOWPOWER_RECONFIG); // Need to return to sleep.
  // If shutdown because battery is low and there is no VUSB, shutdown again.
  if ((shutdown_reg & TNC_LOWPOWER_LOW_BAT) && !(USB_POWER_GPIO_Port->IDR & USB_POWER_Pin)) {
    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_LOW_BAT);
    HAL_PWR_DisableBkUpAccess();
    WARN("Battery too low to start");
    indicate_battery_low();
    HAL_Delay(3000);
    go_back_to_sleep = 1;
  }

  INFO("start");
  if ((resetCause == RESET_CAUSE_SOFT || resetCause == RESET_CAUSE_IWDG) && error_message[0] != 0) {
    error_message[79] = 0;
    WARN(error_message);
  }
  error_message[0] = 0;
  if (shutdown_reg == 0) {
    memset(mac_address, 0, 6);
    memset(error_message, 0, 80);
    charging_enabled = 0;
    usb_wake_state = 0;
  }
  
  // Needed to check battery level.
  enable_vdd();
  HAL_Delay(10);
  MX_ADC1_Init();
  MX_TIM6_Init();
  if (HAL_ADCEx_Calibration_Start(&BATTERY_ADC_HANDLE, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }

  // Don't start up at all if battery is low.
  if (!go_back_to_sleep && is_battery_low()) {
    WARN("low battery");
    indicate_battery_low();
    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_LOW_BAT);
    HAL_PWR_DisableBkUpAccess();
    HAL_Delay(3000);
    go_back_to_sleep = 1;
  }

  SCB->SHCSR |= 0x70000;    // Enable fault handlers;
  if (!go_back_to_sleep) {
      indicate_turning_on();    // LEDs on during boot.
      if (power_switch_state && reset_button) {
        reset_requested = 1;
      }
  }

  encode_serial_number();

  MX_DAC1_Init();
  MX_OPAMP1_Init();
  MX_TIM7_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();

  if (!go_back_to_sleep) {
      HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_SET); // BT module on.
      HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_RESET); // BT module out of reset.
      HAL_Delay(1);
      HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_SET); // BT module out of reset.
      bm78_wait_until_ready();

      // Initialize the BM78 Bluetooth module the first time we boot.
      if (!bm78_initialized() || reset_requested) {
          bm78_initialize();
      } else if (reset_button) {
          bm78_initialize_mac_address();
      }

      HAL_IWDG_Refresh(&hiwdg);

    // Initialize the DC offset DAC and the PGA op amp.  Calibrate the ADC.
    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048) != HAL_OK) Error_Handler();
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_OPAMP_SelfCalibrate(&hopamp1) != HAL_OK) Error_Handler();
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) Error_Handler();
  }

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

  // Disable IWDG in stop2
  HAL_FLASHEx_OBGetConfig(&obInit);
  if ((obInit.USERConfig & FLASH_OPTR_IWDG_STOP)) {
      obInit.OptionType = OPTIONBYTE_USER;
      obInit.USERType = OB_USER_IWDG_STOP;
      obInit.USERConfig = OB_IWDG_STOP_FREEZE;
      HAL_FLASH_Unlock();
      HAL_FLASH_OB_Unlock();
      HAL_FLASHEx_OBProgram(&obInit);
      HAL_FLASH_OB_Launch();
      HAL_FLASH_OB_Lock();
      HAL_FLASH_Lock();
  }

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"    // cmsis-os is not const-correct.
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of usbShutdownTimer */
  osTimerStaticDef(usbShutdownTimer, shutdown, &usbShutdownTimerControlBlock);
  usbShutdownTimerHandle = osTimerCreate(osTimer(usbShutdownTimer), osTimerOnce, NULL);

  /* definition and creation of powerOffTimer */
  osTimerStaticDef(powerOffTimer, powerOffTimerCallback, &powerOffTimerControlBlock);
  powerOffTimerHandle = osTimerCreate(osTimer(powerOffTimer), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ioEventQueue */
  osMessageQStaticDef(ioEventQueue, 16, uint32_t, ioEventQueueBuffer, &ioEventQueueControlBlock);
  ioEventQueueHandle = osMessageCreate(osMessageQ(ioEventQueue), NULL);

  /* definition and creation of audioInputQueue */
  osMessageQStaticDef(audioInputQueue, 8, uint8_t, audioInputQueueBuffer, &audioInputQueueControlBlock);
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

//  MX_IWDG_Init();

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ioEventTask */
  osThreadStaticDef(ioEventTask, startIOEventTask, osPriorityLow, 0, 384, ioEventTaskBuffer, &ioEventTaskControlBlock);
  ioEventTaskHandle = osThreadCreate(osThread(ioEventTask), NULL);

  /* definition and creation of audioInputTask */
  osThreadStaticDef(audioInputTask, startAudioInputTask, osPriorityAboveNormal, 0, 512, audioInputTaskBuffer, &audioInputTaskControlBlock);
  audioInputTaskHandle = osThreadCreate(osThread(audioInputTask), NULL);

  /* definition and creation of modulatorTask */
  osThreadStaticDef(modulatorTask, startModulatorTask, osPriorityAboveNormal, 0, 384, modulatorTaskBuffer, &modulatorTaskControlBlock);
  modulatorTaskHandle = osThreadCreate(osThread(modulatorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#pragma GCC diagnostic pop

  osThreadSuspend(audioInputTaskHandle);
  osThreadSuspend(modulatorTaskHandle);

  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_RESUMED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300617;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_HIGH;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_CONNECT_NO;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALPOWER;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
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
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  // Do not initialize RTC if the date/time has been set.
  if (!(RTC->ISR & 0x10)) { // Labelled ICSR in the reference manual.

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  }
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1817;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1817;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

  /*Configure GPIO pin : USB_POWER_Pin */
  GPIO_InitStruct.Pin = USB_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USB_POWER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_POWER_Pin SW_BOOT_Pin */
  GPIO_InitStruct.Pin = SW_POWER_Pin|SW_BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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

  /*Configure GPIO pins : AUDIO_ATTEN_Pin USB_CE_Pin BT_CMD_Pin BT_RESET_Pin */
  GPIO_InitStruct.Pin = AUDIO_ATTEN_Pin|USB_CE_Pin|BT_CMD_Pin|BT_RESET_Pin;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#if 1
_ssize_t _write_r(struct _reent *r, int fd, const void *ptr, size_t len);

_ssize_t _write_r(struct _reent *r, int fd, const void *ptr, size_t len)
{
  UNUSED(r);
  UNUSED(fd);
#ifdef KISS_LOGGING
    for (size_t i = 0; i != len; ++i)
      ITM_SendChar(((char*) ptr)[i]);
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

/**
 * @page Clock Configuration
 * 
 * The clock configurations are used to reduce power consumption to the
 * minimum required by the system components and the modem selected.
 * 
 * It is required that no modem is active (audio input is IDLE) when switching
 * clocks.
 * 
 * Note that HSI48, which consumes quite a bit of power, is never enabled in
 * any of these clock modes.
 * 
 * 16MHz Clock
 * 
 * The 16MHz clock is only used at startup. The clock tree is set as close
 * as possible to the other clock modes. This is only used when initializing
 * the peripherals. MSI set to 48MHz in PLL mode.
 * 
 * LSE, LSI, MSI and HSI are enabled.
 * 
 * LSE is used for RTC and MSI PLL.
 * LSI is used for IWDG.
 * HSI is used for SYSCLK, UART, and I2C.
 * MSI is used for RNG and USB.
 * 
 * 2MHz Clock
 * 
 * When disconnected and on battery power, the clock is set to 2MHz. In this
 * mode, almost all peripheral clocks are gated. It is required that both the
 * USB and RNG clocks are gated before switching to the 2MHz clock as they are
 * driven by the MSI directly; both must driven by a 48MHz clock.
 * 
 * The ADC is used solely for monitoring the power domain via a VREFINT
 * watchdog. The ADC is clocked from SYSCLK in this mode.
 * 
 * LSE, LSI, MSI, HSI are enabled.
 * 
 * LSE is used for RTC and MSI PLL.
 * LSI is used for IWDG.
 * HSI is used for UART and I2C.
 * MSI is used for SYSCLK (RNG and USB gated).
 * PLL is disabled and unused.
 * 
 * LCSO is only enabled for DEBUG builds.
 * 
 * In this mode, voltage scaling is reduced to the lowest possible. The system
 * is in low power run mode.
 * 
 * 48MHz Clock
 * 
 * When powered by USB, or when needed by a modem, the 48MHz clock is used.
 * In this mode, most peripherals are disabled in disconnected mode, and
 * enabled in connected mode.
 * 
 * The MSI clock is running at 48MHz in PLL mode to get the most accurate
 * clock possible (48.005Mhz, 32768Hz * 1465) in a system without an HSE.
 * 
 * The ADC is driven by PLLSAIR at 80MHz so that it can be used in injected
 * watchdog mode. At 80Mhz the ADC can do 48000 samples per second of two
 * channels at 24.5 cycles and 16x oversampling.
 * 
 * LSE, LSI, MSI, HSI are enabled.
 * 
 * LSE is used for RTC and MSI PLL.
 * LSI is used for IWDG.
 * HSI is used for UART and I2C.
 * MSI is used for SYSCLK, RNG and USB.
 * PLLSAIR is used for ADC.
 * 
 * 72MHz Clock
 * 
 * When needed by a modem, the 72MHz clock is used. Currently this is only
 * used by the 9600 baud GFSK modem. In this mode, most peripherals are
 * disabled in disconnected mode, and enabled in connected mode. This mode
 * may be used when plugged in to USB and disconnected.
 * 
 * The MSI clock is running at 48MHz in PLL mode to get the most accurate
 * clock possible (48.005Mhz, 32768Hz * 1465) in a system without an HSE.
 * 
 * The ADC is driven by PLLSAIR at 80MHz so that it can be used in injected
 * watchdog mode. At 80Mhz the ADC can do 96000 samples per second of two
 * channels at 12.5 cycles and 16x oversampling.
 * 
 * LSE, LSI, MSI, HSI are enabled.
 * 
 * LSE is used for RTC and MSI PLL.
 * LSI is used for IWDG.
 * HSI is used for UART and I2C.
 * MSI is used for RNG and USB.
 * PLL/R is used for SYSCLK.
 * PLLSAIR is used for ADC.
 */

void SysClock2()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
    HAL_StatusTypeDef status = HAL_OK;

    if (HAL_RCC_GetHCLKFreq() == 2000000) return;

    INFO("Setting 2MHz SysClock.");

    vTaskSuspendAll();

    // Disable low power run mode.
    HAL_RCCEx_DisableMSIPLLMode();
    status = HAL_PWREx_DisableLowPowerRunMode();
    if (status != HAL_OK) {
        _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    status = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
    if (status != HAL_OK)
    {
      _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }

    TPI->ACPR = 7;  // 16MHz clock, 2MHz SWO
    __HAL_TIM_SET_PRESCALER(&LED_PWM_TIMER_HANDLE, 15);

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    // PLLSAI cannot be used when disabling or modifying the PLL.
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;

    status = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
    if (status != HAL_OK)
    {
      _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5; // 2MHz
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      _Error_Handler(__FILE_NAME__, __LINE__);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;

    status = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
    if (status != HAL_OK)
    {
      _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }

    TPI->ACPR = 0;  // 2MHz clock, 2MHz SWO
    __HAL_TIM_SET_PRESCALER(&LED_PWM_TIMER_HANDLE, 1);
    __HAL_TIM_SET_PRESCALER(&htim6, 7);
    __HAL_TIM_SET_AUTORELOAD(&htim6, 249);

    // Reduce voltage scaling and set low power run mode.
    status = HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
    if (status != HAL_OK) {
        _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }
    HAL_PWREx_EnableLowPowerRunMode();

    HAL_RCCEx_EnableMSIPLLMode();

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

#ifdef KISS_LOGGING
    HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#else
    HAL_RCCEx_DisableLSCO();
#endif

    INFO("CPU core clock: %luHz", SystemCoreClock);

    HAL_Delay(10);

    xTaskResumeAll();
}

void SysClock48()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
    HAL_StatusTypeDef status;

    if (HAL_RCC_GetHCLKFreq() == 48000000) return;

    INFO("Setting 48MHz SysClock.");

    vTaskSuspendAll();

    // Disable low power run mode and set the voltage scaling to maximum.
    HAL_RCCEx_DisableMSIPLLMode();
    status = HAL_PWREx_DisableLowPowerRunMode();
    if (status != HAL_OK) {
        _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }
    status = HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    if (status != HAL_OK) {
        _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    status = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
    if (status != HAL_OK)
    {
      _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }

    TPI->ACPR = 7;  // 16MHz clock, 2MHz SWO
    __HAL_TIM_SET_PRESCALER(&LED_PWM_TIMER_HANDLE, 15);

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    // PLLSAI cannot be used when disabling or modifying the PLL.
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      _Error_Handler(__FILE_NAME__, __LINE__);
    }

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 12;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      _Error_Handler(__FILE_NAME__, __LINE__);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;

    // STM32L433 at 48MHz requires at least 2 wait states.
    status = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
    if (status != HAL_OK)
    {
      _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }

    TPI->ACPR = 23;  // 48MHz clock, 2MHz SWO
    __HAL_TIM_SET_PRESCALER(&LED_PWM_TIMER_HANDLE, 47);
    __HAL_TIM_SET_PRESCALER(&htim6, 0);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 6;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 20;
    PeriphClkInit.PLLSAI1.PLLSAI1P = 2;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = 2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = 2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      _Error_Handler(__FILE_NAME__, __LINE__);
    }

    HAL_RCCEx_EnableMSIPLLMode();

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

#ifdef KISS_LOGGING
    HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#else
    HAL_RCCEx_DisableLSCO();
#endif

    INFO("CPU core clock: %luHz", SystemCoreClock);

    HAL_Delay(10);

    xTaskResumeAll();
}

void SysClock72()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
    HAL_StatusTypeDef status;

    if (HAL_RCC_GetHCLKFreq() == 72000000) return;

    INFO("Setting 72MHz SysClock.");

    vTaskSuspendAll();

    // Disable low power run mode and set the voltage scaling to maximum.
    HAL_RCCEx_DisableMSIPLLMode();
    status = HAL_PWREx_DisableLowPowerRunMode();
    if (status != HAL_OK) {
        _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }
    status = HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    if (status != HAL_OK) {
        _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
      _Error_Handler(__FILE_NAME__, __LINE__);
    }

    TPI->ACPR = 7;  // 16MHz clock, 2MHz SWO
    __HAL_TIM_SET_PRESCALER(&LED_PWM_TIMER_HANDLE, 15);

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    // PLLSAI cannot be used when disabling or modifying the PLL.
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      _Error_Handler(__FILE_NAME__, __LINE__);
    }

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 18;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      _Error_Handler(__FILE_NAME__, __LINE__);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    // STM32L433 at 72MHz requires at least 4 wait states.
    status = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
    if (status != HAL_OK)
    {
      _Error_Handler2(__FILE_NAME__, __LINE__, status);
    }

    TPI->ACPR = 35;  // 72MHz clock, 2MHz SWO
    __HAL_TIM_SET_PRESCALER(&LED_PWM_TIMER_HANDLE, 71);
    __HAL_TIM_SET_PRESCALER(&htim6, 0);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 6;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 20;
    PeriphClkInit.PLLSAI1.PLLSAI1P = 2;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = 2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = 2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      _Error_Handler(__FILE_NAME__, __LINE__);
    }

    HAL_RCCEx_EnableMSIPLLMode();

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

#ifdef KISS_LOGGING
    HAL_RCCEx_EnableLSCO(RCC_LSCOSOURCE_LSE);
#else
    HAL_RCCEx_DisableLSCO();
#endif

    INFO("CPU core clock: %luHz", SystemCoreClock);

    HAL_Delay(10);

    xTaskResumeAll();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
#ifdef KISS_LOGGING
  printf("Error handler called from file %s on line %d\r\n", file, line);
#endif
  snprintf(error_message, sizeof(error_message), "Error: %s:%d", file, line);
  error_message[sizeof(error_message) - 1] = 0;

  go_back_to_sleep = 0;

  vTaskSuspendAll(); // Will eventually cause IWDG reset.

  error_code(MORSE_1, MORSE_1);
}

void _Error_Handler2(char *file, int line, HAL_StatusTypeDef status)
{
#ifdef KISS_LOGGING
  printf("Error handler called from file %s on line %d, status = %d\r\n", file, line, status);
#endif
  snprintf(error_message, sizeof(error_message), "Error: %s:%d, status = %d", file, line, status);
  error_message[sizeof(error_message) - 1] = 0;

  go_back_to_sleep = 0;

  vTaskSuspendAll(); // Will eventually cause IWDG reset.

  error_code(MORSE_1, MORSE_1);
}
volatile uint32_t delay_count = 0;


void delay(uint32_t ms) {
  delay_count = (SystemCoreClock >> 13) * ms;
  for (uint32_t i = 0; i != delay_count; ++i) asm volatile("nop");
}

void dit()
{
  HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_RESET);
  delay(100);
  HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_SET);
  delay(100);
}

void dah()
{
  HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_RESET);
  delay(300);
  HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_SET);
  delay(100);
}

void brk1()
{
  delay(300);
}

void brk2()
{
  delay(700);
}

/**
 * Output a two-digit error code in Morse. This outputs an error code non-stop
 * in Morse code to the TX LED (red). Once this code is hit, the TNC will need
 * to be reset.
 *
 * The code is passed as a 5-bit value, with 1 = dit, 0 = dah.
 *
 * For example:
 *  0 = 00000 / 0x00
 *  1 = 10000 / 0x10
 *  2 = 11000 / 0x18
 *  3 = 11100 / 0x1C
 *  4 = 11110 / 0x1E
 *  5 = 11111 / 0x1F
 *  6 = 01111 / 0x0F
 *  7 = 00111 / 0x07
 *  8 = 00011 / 0x03
 *  9 = 00001 / 0x01
 */
void error_code(int8_t a, int8_t b)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_TIM_PWM_DeInit(&LED_PWM_TIMER_HANDLE);  // Disable PWM.

  // Re-initialize LED GPIO.
  GPIO_InitStruct.Pin = LED_BT_Pin|LED_RX_Pin|LED_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LED_BT_GPIO_Port, LED_BT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_RX_GPIO_Port, LED_RX_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_SET);

  a &= 0x1F;
  b &= 0x1F;
  for (uint8_t j = 0; j != 3; ++j)
  {
    int8_t aa = a;
    for (int i = 0; i != 5; ++i)
    {
      if (aa & 0x10) dit(); else dah();
      aa <<= 1;
    }
    brk1();
    int8_t bb = b;
    for (int i = 0; i != 5; ++i)
    {
      if (bb & 0x10) dit(); else dah();
      bb <<= 1;
    }
    brk2();
  }
  NVIC_SystemReset();
}

void __assert_fail(const char *expr, const char *file, unsigned int line, const char *function)
{
  snprintf(error_message, sizeof(error_message), "Assertion %s failed in %s at %s:%u", expr, function, file, line);
  ERROR("Assertion %s failed in %s at %s:%u", expr, function, file, line);
  error_code(0,0);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startIOEventTask */
/**
  * @brief  Function implementing the ioEventTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startIOEventTask */
__weak void startIOEventTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(osWaitForever);
  }
  /* USER CODE END 5 */
}

/* shutdown function */
void shutdown(void const * argument)
{
  /* USER CODE BEGIN shutdown */
  UNUSED(argument);
  osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 0);
  /* USER CODE END shutdown */
}

/* powerOffTimerCallback function */
void powerOffTimerCallback(void const * argument)
{
  /* USER CODE BEGIN powerOffTimerCallback */

  UNUSED(argument);
  INFO("shutdown timer triggered");
  osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 0);

  /* USER CODE END powerOffTimerCallback */
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
      LED_TIMER_PeriodElapsedCallback();
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    snprintf(error_message, sizeof(error_message), "Wrong parameters value: file %s on line %lu", file, line);
    ERROR("Wrong parameters value: file %s on line %lu", file, line);
    error_code(0,0);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
