/****************************************************************************
 * boards/arm/stm32f7/ardusimple-mapkit/src/stm32_userleds.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include <stdbool.h>
#include <debug.h>

#include "stm32_gpio.h"
#include "ardusimple-mapkit.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* LED Power Management */

#ifdef CONFIG_PM
static void led_pm_notify(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
static int led_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM
static struct pm_callback_s g_ledscb =
{
  .notify  = led_pm_notify,
  .prepare = led_pm_prepare,
};
#endif

/* This array maps an LED number to GPIO pin configuration */

static uint32_t g_ledcfg[BOARD_NLEDS] =
{
  GPIO_LD1, 
  GPIO_LD2, 
  GPIO_LD3,
  GPIO_LD4,
  GPIO_LD5,
  GPIO_LD6,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void led_pm_notify(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Restore normal LEDs operation */
        }
        break;

      case(PM_IDLE):
        {
          /* Entering IDLE mode - Turn leds off */
        }
        break;

      case(PM_STANDBY):
        {
          /* Entering STANDBY mode - Logic for PM_STANDBY goes here */
        }
        break;

      case(PM_SLEEP):
        {
          /* Entering SLEEP mode - Logic for PM_SLEEP goes here */
        }
        break;

      default:
        {
          /* Should not get here */
        }
        break;
    }
}
#endif

/****************************************************************************
 * Name: led_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int led_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate)
{
  /* No preparation to change power modes is required by the LEDs driver.
   * We always accept the state change by returning OK.
   */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the
 *   board_userled_initialize() is available to initialize the LED from user
 *   application logic.
 *
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  for (int i=0; i<BOARD_NLEDS; i++) {
    stm32_configgpio(g_ledcfg[i]);
  }
  
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *  LEDs.  If CONFIG_ARCH_LEDS is not defined, then the board_userled() is
 *  available to control the LED from user application logic.
 *
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if ((unsigned)led < BOARD_NLEDS)
    {
      stm32_gpiowrite(g_ledcfg[led], !ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *  LEDs.  If CONFIG_ARCH_LEDS is not defined, then the board_userled_all()
 *  is available to control the LED from user application logic. NOTE: since
 *  there is only a single LED on-board, this is function is not very useful.
 *
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  for (int i=0; i<BOARD_NLEDS; i++) {
    stm32_gpiowrite(g_ledcfg[i], (ledset & (1 << i)) == 0);
  }
}

#ifdef CONFIG_USERLED_LOWER_READSTATE
/****************************************************************************
 * Name: board_userled_getall
 ****************************************************************************/

void board_userled_getall(uint32_t *ledset)
{
  /* Clear the LED bits */

  *ledset = 0;

  /* Get LED state. An output of '1' illuminates the LED. */

  for (int i=0; i<BOARD_NLEDS; i++) {

    *ledset |= (stm32_gpioread(g_ledcfg[i]) << i);
  }
}
#endif /* CONFIG_USERLED_LOWER_READSTATE */

#endif /* !CONFIG_ARCH_LEDS */
