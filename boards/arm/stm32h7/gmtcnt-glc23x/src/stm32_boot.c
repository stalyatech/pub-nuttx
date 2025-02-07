/****************************************************************************
 * boards/arm/stm32h7/gmtcnt-glc23x/src/stm32_boot.c
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
#include "gmtcnt-glc23x.h"

/****************************************************************************
 * Name: board_power_initialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

static void board_power_initialize(void)
{
  /* OTG power */

	stm32_configgpio(GPIO_OTGFS_PWRON);

  /* FPGA power */

	stm32_configgpio(GPIO_FPGA_PWRCO);
	stm32_configgpio(GPIO_FPGA_PWRIO);

  /* Ethernet power */

	stm32_configgpio(GPIO_ETH_PWR);

  /* External ADC power */

	stm32_configgpio(GPIO_ADC_PWR);

  /* Absolute encoder power */

	stm32_configgpio(GPIO_ENC_PWR);

  /* Expansion bus */

	stm32_configgpio(GPIO_EXP_ENB);
	stm32_configgpio(GPIO_EXP_SIL);

  /* LED controller */

	stm32_configgpio(GPIO_LDC_ENB);
	stm32_configgpio(GPIO_LDC_RST);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
  /* Configure board power section. */

	board_power_initialize();

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif /* CONFIG_ARCH_LEDS */

#if defined(CONFIG_STM32H7_OTGFS) || defined(CONFIG_STM32H7_HOST)
  /* Initialize USB */

  stm32_usbinitialize();
#endif /* CONFIG_STM32H7_OTGFS || CONFIG_STM32H7_HOST */

#ifdef CONFIG_STM32H7_SPI
  /* Configure SPI chip selects */

  stm32_spidev_initialize();
#endif /* CONFIG_STM32H7_SPI */

#ifdef CONFIG_STM32H7_ETHMAC
  /* Configure GPIO pins to support Ethernet */

  stm32_netinitialize();
#endif /* CONFIG_STM32H7_ETHMAC */
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize()
 *   will be called immediately after up_initialize() is called and just
 *   before the initial application is started.  This additional
 *   initialization phase may be used, for example, to initialize board-
 *   specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
	/* Perform board-specific initialization */

  stm32_bringup();
}
#endif
