/****************************************************************************
 * boards/arm/stm32f7/ardusimple-sbc/src/stm32_usbhub.c
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
#include <nuttx/usb/usb2517.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "ardusimple-sbc.h"

#ifdef CONFIG_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usb2517_config_s hub_config = 
{
  .mfr_str = "EPSWorks",
  .prd_str = "simpleRTK2B-SBC",
  .ser_str = "1234567890",
  .config  = 
  {
    0x24,   // Vendor ID LSB
    0x04,   // Vendor ID MSB
    0x17,   // Product ID LSB
    0x25,   // Product ID MSB
    0x11,   // Device ID LSB
    0x22,   // Device ID MSB
    0xBB,		// Configuration Data Byte 1
    0x20,		// Configuration Data Byte 2
    0x09,		// Configuration Data Byte 3
    0x00,		// Non-Removable Devices
    0x00,		// Port Disable (Self)
    0x00,		// Port Disable (Bus)
    0x32,		// Max Power (Self)
    0xFA,		// Max Power (Bus)
    0x32,		// Hub Controller Max Current (Self)
    0xFA,		// Hub Controller Max Current (Bus)
    0x64,		// Power-on Time
    0x04,		// Language ID High (Turkish:0x041F, English:0x0409)
    0x09,		// Language ID Low
    0x08,		// Manufacturer String Length
    0x0F,		// Product String Length
    0x00,		// Serial String Length
    
    // Manufacturer String (max 31 character)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 
    
    // Product String (max 31 character)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 
    
    // Serial String (max 31 character)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 
    
    // Reserved
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    
    0x00,		// Boost_Up
    0x00,		// Boost_7:5
    0x00,		// Boost_4:0
    0x00,		// Reserved
    0xC1,		// Port Swap
    0x00,		// Port Map 2-1
    0x00,		// Port Map 4-3
    0x00,		// Port Map 6-5
    0x00,		// Port Map 7
    0x00,		// Status/Command
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_usbhub_initialize
 *
 * Description:
 *
 ****************************************************************************/

int stm32_usbhub_initialize(int bus)
{
  struct i2c_master_s *i2c;
  int ret = ERROR;

  /* Configure GPIOs
   * Power On, and Reset GPIOs
   */

  stm32_configgpio(GPIO_USBHUB_PWRON);
  stm32_configgpio(GPIO_USBHUB_NRST);

  /* reset the device */

  stm32_gpiowrite(GPIO_USBHUB_NRST, false);
  up_udelay(2000);
  stm32_gpiowrite(GPIO_USBHUB_NRST, true);
  
  /* Initialize the I2C bus */

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      /* register the driver */

      ret = usb2517_register("/dev/usbhub0", i2c, 0x2c, &hub_config);
    }

  return ret;
}

#endif /* CONFIG_I2C */
