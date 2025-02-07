/****************************************************************************
 * boards/arm/stm32h7/gmtcnt-glc23x/src/stm32_adc.c
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
#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include <debug.h>

#include "stm32_gpio.h"
#include "stm32_adc.h"
#include "gmtcnt-glc23x.h"

#ifdef CONFIG_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* ADC3 interfaces are supported */

#if defined(CONFIG_STM32H7_ADC3)

/* The number of ADC channels in the conversion list */

#define ADC3_NCHANNELS 1

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32H7_ADC3
/* Identifying number of each ADC channel */

static const uint8_t  g_adc3_chanlist[ADC3_NCHANNELS] =
{
  0
};

/* Configurations of pins used by each ADC channels */

static const uint32_t g_adc3_pinlist[ADC3_NCHANNELS] =
{
  GPIO_ADC3_INP0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
#if defined(CONFIG_STM32H7_ADC3)
  static bool initialized = false;
  struct adc_dev_s *adc;
  int ret;
  int i;
  char devname[] = "/dev/adc0";

  /* Check if we have already initialized */

  if (!initialized)
    {
#endif
#if defined(CONFIG_STM32H7_ADC3)
      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC3_NCHANNELS; i++)
        {
          if (g_adc3_pinlist[i] != 0)
            {
              stm32_configgpio(g_adc3_pinlist[i]);
            }
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      adc = stm32h7_adc_initialize(3, g_adc3_chanlist, ADC3_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC3 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0 or 1" */

      ret = adc_register(devname, adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register(%s) failed: %d\n", devname, ret);
          return ret;
        }
#endif

#if defined(CONFIG_STM32H7_ADC1) || defined(CONFIG_STM32H7_ADC3)
      /* Now we are initialized */

      initialized = true;
    }

  return OK;
#else
  return -ENOSYS;
#endif
}

#endif /* CONFIG_STM32H7_ADC3 */
#endif /* CONFIG_ADC */
