/****************************************************************************
 * include/nuttx/sensors/ms56xx.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MS56XX_H
#define __INCLUDE_NUTTX_SENSORS_MS56XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* These structures are defined elsewhere, and we don't need their
 * definitions here.
 */

#if defined(CONFIG_SENSORS_MS56XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_I2C
 *   Enables support for I2C drivers
 * CONFIG_SENSORS_MS56XX
 *   Enables support for the MS56XX driver
 */

/* I2C Address **************************************************************/

#define MS56XX_ADDR0       0x77
#define MS56XX_ADDR1       0x76

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_MS56XX_SPI
struct spi_dev_s;
#endif

#ifdef CONFIG_MS56XX_I2C
struct i2c_master_s;
#endif

enum ms56xx_model_e
{
  MS56XX_MODEL_MS5607 = 0,
  MS56XX_MODEL_MS5611 = 1,
};

struct ms56xx_measure_s
{
  int32_t temperature;  /* in Degree   x100    */
  int32_t pressure;     /* in mBar     x10     */
};

/* Specifies the initial chip configuration and location */

struct ms56xx_config_s
{
#ifdef CONFIG_MS56XX_SPI
  /* For users on SPI.
   *
   *  spi_devid : the SPI master's slave-select number
   *              for the chip, as used in SPI_SELECT(..., dev_id, ...)
   *  spi       : the SPI master device, as used in SPI_SELECT(spi, ..., ...)
   */

  FAR struct spi_dev_s *spi;
  int spi_devid;
#endif /* CONFIG_ICM20689_SPI */

#ifdef CONFIG_MS56XX_I2C
  /* For users on I2C.
   *
   *  i2c  : the I2C master device
   *  addr : the I2C address.
   */

  FAR struct i2c_master_s *i2c;
  int addr;
#endif /* CONFIG_ICM20689_I2C */

  /* Bus Frequency I2C/SPI */

  uint32_t freq;

  /* Device model */

  enum ms56xx_model_e model;
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
 * Name: ms56xx_register
 *
 * Description:
 *   Register the MS56XX character device as 'devpath'.
 *
 * Input Parameters:
 *   devno   - Number of device (i.e. baro0, baro1, ...)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int ms56xx_register(int devno, struct ms56xx_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_MS56XX */
#endif /* __INCLUDE_NUTTX_SENSORS_MS56XX_H */
