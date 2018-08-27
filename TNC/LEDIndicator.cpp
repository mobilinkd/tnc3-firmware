// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "LEDIndicator.h"

#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_tim.h>
#include <stm32l4xx_hal_tim_ex.h>

#include <functional>

#include <stdint.h>

extern "C" void _Error_Handler(char *, int);

extern TIM_HandleTypeDef htim1;

namespace mobilinkd { namespace tnc {

}} // mobilinkd::tnc

/**
 * No connection shows a low, slow breathing. Each breath inhale takes
 * for 500ms, is held for 500ms, and exhaled in 500ms. This is repeated
 * every 10 seconds.  Maximum brightness is 20%.
 *
 * Each interrupt occurs at 10ms intervals.
 *
 * The sequence is:
 *  - ramp up 300ms(30)
 *  - hold 400ms (40)
 *  - ramp down 300ms (30)
 *  - wait 9000ms (900)
 *
 *
 */
struct NoConnection {
  enum STATE {
    RAMP_UP_1, WAIT_1, RAMP_DN_1, WAIT_2
  };

  int count{0};
  int state{RAMP_UP_1};

  int operator()()
  {
    int result;
    switch (state) {
    case RAMP_UP_1:
      result = count * 40;
      if (count == 49) {
        count = 0;
        state = WAIT_1;
      } else {
        ++count;
      }
      break;
    case WAIT_1:
      result = 2000;
      if (count == 49) {
        state = RAMP_DN_1;
        count = 49;
      } else {
        ++count;
      }
      break;
    case RAMP_DN_1:
      result = count * 40;
      if (count == 0) {
        count = 0;
        state = WAIT_2;
      } else {
        --count;
      }
      break;
    case WAIT_2:
      result = 0;
      if (count == 849) {
        state = RAMP_UP_1;
        count = 0;
      } else {
        ++count;
      }
      break;
    }
    return result;
  }
};

/**
 * Bluetooth connection shows a double blip. Each blip lasts for 200ms
 * and is separated by 200ms, and is repeated ever 5 seconds.
 *
 * Each interrupt occurs at 10ms intervals.
 *
 * The sequence is:
 *  - ramp up 100s(10)
 *  - ramp down 100ms (10)
 *  - wait 200ms (20)
 *  - ramp up 100ms (10)
 *  - ramp down 100ms (10)
 *  - wait 4400ms (440)
 *
 *
 */
struct BluetoothConnection {
  enum STATE {
    RAMP_UP_1, RAMP_DN_1, WAIT_1, RAMP_UP_2, RAMP_DN_2, WAIT_2
  };

  int count{0};
  int pulse{0};
  int state{RAMP_UP_1};
  int ramp[10] = {1564,3090,4540,5878,7071,8090,8910,9510,9877,9999};

  int operator()()
  {
    int result;
    switch (state) {
    case RAMP_UP_1:
      result = ramp[count] / 2;
      if (count == 9) {
        state = RAMP_DN_1;
      } else {
        ++count;
      }
      break;
    case RAMP_DN_1:
      result = ramp[count] / 2;
      if (count == 0) {
        state = WAIT_1;
      } else {
        --count;
      }
      break;
    case WAIT_1:
      result = 0;
      if (count == 19) {
        state = RAMP_UP_2;
        count = 0;
      } else {
        ++count;
      }
      break;
    case RAMP_UP_2:
      result = ramp[count] / 2;
      if (count == 9) {
        state = RAMP_DN_2;
      } else {
        ++count;
      }
      break;
    case RAMP_DN_2:
      result = ramp[count] / 2;
      if (count == 0) {
        state = WAIT_2;
      } else {
        --count;
      }
      break;
    case WAIT_2:
      result = 0;
      if (count == 439) {
        state = RAMP_UP_1;
        count = 0;
      } else {
        ++count;
      }
      break;
    }
    return result;
  }
};

/**
 * USB connection shows a triple blip. Each blip lasts for 200ms. The
 * first two are separated by 400ms.  The third comes 200ms later.  This
 * is repeated ever 5 seconds.
 *
 * Each interrupt occurs at 10ms intervals.
 *
 * The sequence is:
 *  - ramp up 100s(10)
 *  - ramp down 100ms (10)
 *  - wait 200ms (20)
 *  - ramp up 100s(10)
 *  - ramp down 100ms (10)
 *  - wait 400ms (20)
 *  - ramp up 100ms (10)
 *  - ramp down 100ms (10)
 *  - wait 3800ms (440)
 *
 *
 */
struct USBConnection {
  enum STATE {
    RAMP_UP_1, RAMP_DN_1, WAIT_1, RAMP_UP_2, RAMP_DN_2, WAIT_2, RAMP_UP_3, RAMP_DN_3, WAIT_3
  };

  int count{0};
  int pulse{0};
  int state{RAMP_UP_1};
  int ramp[10] = {1564,3090,4540,5878,7071,8090,8910,9510,9877,9999};

  int operator()()
  {
    int result;
    switch (state) {
    case RAMP_UP_1:
      result = ramp[count];
      if (count == 9) {
        state = RAMP_DN_1;
      } else {
        ++count;
      }
      break;
    case RAMP_DN_1:
      result = ramp[count];
      if (count == 0) {
        state = WAIT_1;
      } else {
        --count;
      }
      break;
    case WAIT_1:
      result = 0;
      if (count == 39) {
        state = RAMP_UP_2;
        count = 0;
      } else {
        ++count;
      }
      break;
    case RAMP_UP_2:
      result = ramp[count];
      if (count == 9) {
        state = RAMP_DN_2;
      } else {
        ++count;
      }
      break;
    case RAMP_DN_2:
      result = ramp[count];
      if (count == 0) {
        state = WAIT_2;
      } else {
        --count;
      }
      break;
    case WAIT_2:
      result = 0;
      if (count == 19) {
        state = RAMP_UP_3;
        count = 0;
      } else {
        ++count;
      }
      break;
    case RAMP_UP_3:
      result = ramp[count];
      if (count == 9) {
        state = RAMP_DN_3;
      } else {
        ++count;
      }
      break;
    case RAMP_DN_3:
      result = ramp[count];
      if (count == 0) {
        state = WAIT_3;
      } else {
        --count;
      }
      break;
    case WAIT_3:
      result = 0;
      if (count == 379) {
        state = RAMP_UP_1;
        count = 0;
      } else {
        ++count;
      }
      break;
    }
    return result;
  }
};

struct Flash {
  enum STATE {
    RAMP_UP, ON, RAMP_DN, OFF
  };

  int gr_count{9};
  STATE gr_state{OFF};
  int rd_count{9};
  STATE rd_state{OFF};
  int ramp[10] = {1564,3090,4540,5878,7071,8090,8910,9510,9877,9999};

  typedef std::function<int(void)> function_type;

  NoConnection noConnection;
  BluetoothConnection btConnection;
  USBConnection usbConnection;

  function_type blue_func{noConnection};

  int blue()
  {
    return blue_func();
  }

  int green()
  {
    int result = 0;
    switch (gr_state) {
    case RAMP_UP:
      result = ramp[gr_count] / 3;
      if (gr_count == 9) {
        gr_state = ON;
      } else {
        ++gr_count;
      }
      break;
    case ON:
      result = ramp[gr_count] / 3;
      break;
    case RAMP_DN:
      result = ramp[gr_count] / 3;
      if (gr_count == 0) {
        gr_state = OFF;
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
      } else {
        --gr_count;
      }
      break;
    case OFF:
      result = 0;
      break;
    }
    return result;
  }

  int red()
  {
    int result = 0;
    switch (rd_state) {
    case RAMP_UP:
      result = ramp[rd_count] / 3;
      if (rd_count == 9) {
        rd_state = ON;
      } else {
        ++rd_count;
      }
      break;
    case ON:
      result = ramp[rd_count] / 3;
      break;
    case RAMP_DN:
      result = ramp[rd_count] / 3;
      if (rd_count == 0) {
        rd_state = OFF;
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
      } else {
        --rd_count;
      }
      break;
    case OFF:
      result = 0;
      break;
    }
    return result;
  }

  void dcd_on() {
    if (gr_state == OFF)
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    gr_state = RAMP_UP;
  }
  void dcd_off() {
    if (gr_state != OFF) gr_state = RAMP_DN;
  }

  void tx_on() {
    if (rd_state == OFF)
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    rd_state = RAMP_UP;
  }
  void tx_off() {
    if (rd_state != OFF) rd_state = RAMP_DN;
  }
  void disconnect() { blue_func = noConnection; }
  void usb() { blue_func = usbConnection; }
  void bt() { blue_func = btConnection; }
};

Flash flash;

void HTIM1_PeriodElapsedCallback()
{
  htim1.Instance->CCR1 = flash.blue();
  htim1.Instance->CCR2 = flash.red();
  htim1.Instance->CCR3 = flash.green();
}

void indicate_turning_on(void)
{
  HAL_TIM_Base_Start_IT(&htim1);
  tx_on();

  HAL_Delay(200);
}

void indicate_initializing_ble(void)
{
  rx_on();
}

void indicate_on()
{
  tx_off();
  rx_off();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void indicate_waiting_to_connect(void)
{
  flash.disconnect();
}
void indicate_connected_via_usb(void)
{
  flash.usb();
}
void indicate_connected_via_ble(void)
{
  flash.bt();
}

void tx_on(void)
{
  flash.tx_on();
}

void tx_off(void)
{
  flash.tx_off();
}

// DCD is active.
void rx_on()
{
   flash.dcd_on();
}

// DCD is active.
void rx_off()
{
  flash.dcd_off();
}
