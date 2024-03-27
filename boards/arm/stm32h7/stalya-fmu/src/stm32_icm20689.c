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

#include <stdio.h>
#include <debug.h>

#include <nuttx/sensors/icm20689.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>

#include "stm32.h"
#include "stalya-fmu.h"

/****************************************************************************
 * Private data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_icm20689_i2c_initialize
 *
 * Description:
 *   Initialize and register the I2C based ICM20689 Motion Tracker driver.
 *
 * Input Parameters:
 *   devno - The device number
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_icm20689_i2c_initialize(int devno, int busno)
{
  struct icm20689_config_s config;
  int ret = -ENODEV;

  sninfo("Initializing ICM20689!\n");
  memset(&config, 0, sizeof(config));

#ifdef CONFIG_ICM20689_I2C
  /* Initialize I2C */

  config.freq = CONFIG_ICM20689_I2C_FREQ;
  config.addr = ICM20689_I2CADDR;
  config.i2c = stm32_i2cbus_initialize(busno);
  if (config.i2c != NULL)
    {
      /* Then try to register the imu sensor on I2C */

      ret = icm20689_register(devno, &config);
      if (ret < 0)
        {
          snerr("ERROR: Error registering ICM20689 on I2C%d\n", busno);
        }
    }
#endif /* CONFIG_ICM20689_I2C */

  return ret;
}

/****************************************************************************
 * Name: board_icm20689_spi_initialize
 *
 * Description:
 *   Initialize and register the SPI based ICM20689 Motion Tracker driver.
 *
 * Input Parameters:
 *   devno - The device number
 *   busno - The SPI bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_icm20689_spi_initialize(int devno, int busno)
{
  struct icm20689_config_s config;
  int ret = -ENODEV;

  sninfo("Initializing ICM20689!\n");
  memset(&config, 0, sizeof(config));

#ifdef CONFIG_ICM20689_SPI
  /* Initialize SPI */

  config.freq = CONFIG_ICM20689_SPI_FREQ;
  config.spi_devid = ICM20689_SPIDEV;
  config.spi = stm32_spibus_initialize(busno);
  if (config.spi != NULL)
    {
      /* Then try to register the imu sensor on SPI */

      ret = icm20689_register(devno, &config);
      if (ret < 0)
        {
          snerr("ERROR: Error registering ICM20689 on SPI%d\n", busno);
        }
    }
#endif /* CONFIG_ICM20689_SPI */

  return ret;
}
