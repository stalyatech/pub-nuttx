/****************************************************************************
 * boards/arm/stm32h7/gmtcnt-glc23x/src/stm32_pwm.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_pwm.h"
#include "gmtcnt-glc23x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HAVE_PWM 1
#ifndef CONFIG_PWM
#  undef HAVE_PWM
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   This function initializes the PWM hardware and registers the PWM
 *   device for each configured timer. It supports multiple timers
 *   (TIM1, TIM2, TIM3, TIM4) for generating PWM signals. The function
 *   ensures that initialization is performed only once. If initialization
 *   is successful, it registers each PWM device at unique device paths
 *   ("/dev/pwm0", "/dev/pwm1", etc.). On any failure, it logs an error
 *   message and returns an appropriate error code.
 *
 * Return Value:
 *   Returns OK (0) on successful initialization, or an error code
 *   (e.g., -ENODEV) if the initialization fails.
 *
 ****************************************************************************/

int stm32_pwm_setup(void)
{
#ifdef HAVE_PWM
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32_pwminitialize() to get an instance of the PWM interface */

#if defined(CONFIG_STM32H7_TIM1_PWM)
      pwm = stm32_pwminitialize(1);
      if (!pwm)
        {
          tmrerr("ERROR: Failed to get the STM32H7 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          tmrerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif /* CONFIG_STM32H7_TIM1_PWM */

#if defined(CONFIG_STM32H7_TIM2_PWM)
      pwm = stm32_pwminitialize(2);
      if (!pwm)
        {
          tmrerr("ERROR: Failed to get the STM32H7 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm1", pwm);
      if (ret < 0)
        {
          tmrerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif /* CONFIG_STM32H7_TIM2_PWM */

#if defined(CONFIG_STM32H7_TIM3_PWM)
      pwm = stm32_pwminitialize(3);
      if (!pwm)
        {
          tmrerr("ERROR: Failed to get the STM32H7 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm2", pwm);
      if (ret < 0)
        {
          tmrerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif /* CONFIG_STM32H7_TIM3_PWM */

#if defined(CONFIG_STM32H7_TIM4_PWM)
      pwm = stm32_pwminitialize(4);
      if (!pwm)
        {
          tmrerr("ERROR: Failed to get the STM32H7 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm3", pwm);
      if (ret < 0)
        {
          tmrerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif /* CONFIG_STM32H7_TIM4_PWM */

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
#else
  return -ENODEV;
#endif
}
