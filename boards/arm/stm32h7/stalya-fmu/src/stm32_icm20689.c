/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/src/stm32_icm20689.c
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
#include <nuttx/arch.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include "stm32.h"
#include <stalya-fmu.h>
#include <nuttx/sensors/icm20689.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32H7_I2C4
#  error "ICM20689 driver requires CONFIG_STM32H7_I2C4 to be enabled"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_icm20689_initialize
 *
 * Description:
 *   Initialize I2C-based ICM20689.
 *
 ****************************************************************************/

int stm32_icm20689_initialize(char *devpath)
{
  struct i2c_master_s *i2c;
  int ret = OK;

  sninfo("Initializing ICM20689!\n");

  /* Configure the GPIO interrupt */

#if defined(CONFIG_STM32H7_I2C4)
  i2c = stm32_i2cbus_initialize(4);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sninfo("INFO: Initializing ICM20689 accelero-gyro sensor over I2C%d\n",
         ret);

  ret = icm20689_sensor_register(devpath, i2c, ICM20689_I2CADDR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize ICM20689 accelero-gyro driver %s\n",
            devpath);
      return -ENODEV;
    }

  sninfo("INFO: ICM20689 sensor has been initialized successfully\n");
#endif

  return ret;
}
