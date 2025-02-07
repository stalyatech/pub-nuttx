/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/src/stm32_usb.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_otg.h"
#include "stalya-fmu.h"

#ifdef CONFIG_STM32H7_OTGFS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
#  define HAVE_USB 1
#else
#  warning "CONFIG_STM32_OTGFS is enabled but neither CONFIG_USBDEV nor CONFIG_USBHOST"
#  undef HAVE_USB
#endif

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
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup
 *   USB-related GPIO pins for the Stalya-FMU board.
 *
 ****************************************************************************/

void stm32_usbinitialize(void)
{
  /* The OTG FS has an internal soft pull-up.
   * No GPIO configuration is required
   */

  /* Configure the OTG FS VBUS sensing GPIO,
   * Power On, and Overcurrent GPIOs
   */

  /* USB Hub power is controlled by I/O expander */

#ifdef CONFIG_STM32H7_OTGFS
  stm32_configgpio(GPIO_OTGFS_VBUS);
#endif
}

/****************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV
 *   driver is used. This function is called whenever the USB enters or
 *   leaves suspend mode. This is an opportunity for the board logic to
 *   shutdown clocks, power, etc. while the USB is suspended.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV
void stm32_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}
#endif

#endif /* CONFIG_STM32_OTGFS */
