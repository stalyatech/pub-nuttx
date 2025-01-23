/****************************************************************************
 * boards/arm/stm32h7/gmtcnt-glc23x/src/stm32_netinit.c
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

#include "stm32_gpio.h"
#include "stm32_ethernet.h"
#ifdef CONFIG_STM32H7_FDCAN
#include "stm32_fdcan_sock.h"
#endif

#include "gmtcnt-glc23x.h"

#if defined(CONFIG_STM32H7_ETHMAC)

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
 * Function: board_net_initialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c. If STM32H7_NETHERNET
 *   greater than one, then board specific logic will have to supply a
 *   version of arm_netinitialize() that calls stm32_ethinitialize() with
 *   the appropriate interface number.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/
#ifdef CONFIG_NETDEV_LATEINIT
void board_net_initialize(void)
{
  /* Then initialize the driver */

  stm32_ethinitialize(0);

#ifdef CONFIG_STM32H7_FDCAN1
  stm32_fdcansockinitialize(0);
#endif

#ifdef CONFIG_STM32H7_FDCAN2
  stm32_fdcansockinitialize(1);
#endif

#ifdef CONFIG_STM32H7_FDCAN3
  stm32_fdcansockinitialize(2);
#endif
}
#endif /* CONFIG_NETDEV_LATEINIT */

/****************************************************************************
 * Name: stm32_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ****************************************************************************/

void stm32_netinitialize(void)
{
  /* Configure PHY IRQ input */

  stm32_configgpio(GPIO_ETH_IRQ);

  /* Power on the device */

	stm32_gpiowrite(GPIO_ETH_PWR, true);

  /* Release the reset pin */

  stm32_gpiowrite(GPIO_ETH_RST, true);
}

#endif /* CONFIG_STM32F7_ETHMAC */
