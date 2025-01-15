/****************************************************************************
 * boards/arm/stm32f7/ardusimple-mapkit/src/stm32_usb.c
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

#include <debug.h>

#include "stm32_gpio.h"
#include "ardusimple-mapkit.h"

#if defined(CONFIG_STM32F7_ETHMAC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ****************************************************************************/

void stm32_netinitialize(void)
{
  /* Configure PHY /RESET output */

  stm32_configgpio(GPIO_ETH_NRST);

  /* Configure PHY IRQ input */

  stm32_configgpio(GPIO_ETH_IRQ);
}

#endif /* CONFIG_STM32F7_ETHMAC */
