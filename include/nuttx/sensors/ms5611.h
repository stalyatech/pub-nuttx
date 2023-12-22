/****************************************************************************
 * include/nuttx/sensors/ms5611.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MS5611_H
#define __INCLUDE_NUTTX_SENSORS_MS5611_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#ifdef CONFIG_SENSORS_MS5611

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_I2C
 *   Enables support for I2C drivers
 * CONFIG_SENSORS_MS5611
 *   Enables support for the MS5611 driver
 */

/* I2C Address **************************************************************/

#define MS5611_ADDR0       0x77
#define MS5611_ADDR1       0x76

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These structures are defined elsewhere, and we don't need their
 * definitions here.
 */

#ifdef CONFIG_MS5611_SPI
struct spi_dev_s;
#endif 

#ifdef CONFIG_MS5611_I2C
struct i2c_master_s;
#endif

/* Specifies the initial chip configuration and location */

struct ms5611_config_s
{
#ifdef CONFIG_MS5611_SPI
  /* For users on SPI.
   *
   *  spi_devid : the SPI master's slave-select number
   *              for the chip, as used in SPI_SELECT(..., dev_id, ...)
   *  spi       : the SPI master device, as used in SPI_SELECT(spi, ..., ...)
   */

  FAR struct spi_dev_s *spi;
  int spi_devid;
#endif /* CONFIG_MS5611_SPI */

#ifdef CONFIG_MS5611_I2C
  /* For users on I2C.
   *
   *  i2c  : the I2C master device
   *  addr : the I2C address.
   */

  FAR struct i2c_master_s *i2c;
  int addr;
#endif /* CONFIG_MS5611_I2C */

  /* Bus Frequency I2C/SPI */
  
  uint32_t freq;
};

struct ms5611_measure_s
{
  int32_t temperature;  /* in Degree   x100    */
  int32_t pressure;     /* in mBar     x10     */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ms5611_register
 *
 * Description:
 *   Register the MS5611 character device as 'devpath'.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance.
 *   devno   - Number of device (i.e. baro0, baro1, ...)
 *   addr    - The I2C address of the MS5611.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ms5611_register(int devno, FAR struct ms5611_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_MS5611 */
#endif /* __INCLUDE_NUTTX_SENSORS_MS5611_H */
