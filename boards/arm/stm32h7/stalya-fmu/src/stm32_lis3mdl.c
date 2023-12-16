/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/src/stm32_lis3mdl.c
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
#include <nuttx/sensors/lis3mdl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32H7_I2C4
#  error "LIS3MDL driver requires CONFIG_STM32H7_I2C4 to be enabled"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_lis3mdl_initialize
 *
 * Description:
 *   Initialize I2C-based LIS3MDL.
 *
 ****************************************************************************/

int stm32_lis3mdl_initialize(char *devpath)
{
  struct i2c_master_s *i2c;
  int ret = OK;

  sninfo("INFO: Initializing LIS3MDL sensor over I2C\n");

#if defined(CONFIG_STM32H7_I2C4)
  i2c = stm32_i2cbus_initialize(4);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  ret = lis3mdl_sensor_register(devpath, i2c,
                                  LIS3MDL_I2CADDR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LIS3MDL magneto driver %s\n",
            devpath);
      return -ENODEV;
    }

  sninfo("INFO: LIS3MDL sensor has been initialized successfully\n");
#endif

  return ret;
}
