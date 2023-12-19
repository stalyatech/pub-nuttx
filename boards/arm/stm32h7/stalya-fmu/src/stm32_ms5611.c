/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/src/stm32_ms5611.c
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

#include <stdio.h>
#include <debug.h>

#include <nuttx/sensors/ms5611.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>

#include "stm32.h"
#include "stalya-fmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ms5611_i2c_initialize
 *
 * Description:
 *   Initialize and register the I2C based MS5611 Pressure Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ms5611_i2c_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  int ret = -ENODEV;

  sninfo("Initializing MS5611!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (i2c != NULL)
    {
      /* Then try to register the barometer sensor on I2C */

      ret = ms5611_register(i2c, devno, MS5611_I2CADDR);
      if (ret < 0)
        {
          snerr("ERROR: Error registering MS5611 on I2C%d\n", busno);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_ms5611_spi_initialize
 *
 * Description:
 *   Initialize and register the SPI based MS5611 Pressure Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number
 *   busno - The SPI bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ms5611_spi_initialize(int devno, int busno)
{
  return -ENODEV;
}
