/****************************************************************************
 * include/nuttx/sensors/icm20689.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_ICM20689_H
#define __INCLUDE_NUTTX_SENSORS_ICM20689_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sets bit @n */

#define BIT(n)  (1 << (n))

/* Creates a mask of @m bits, i.e. MASK(2) -> 00000011 */

#define MASK(m) (BIT(m) - 1)

/* Masks and shifts @v into bit field @m */

#define TO_BITFIELD(m,v) (((v) & MASK(m ##_WIDTH)) << (m ##_SHIFT))

/* Un-masks and un-shifts bit field @m from @v */

#define FROM_BITFIELD(m,v) (((v) >> (m ##_SHIFT)) & MASK(m ##_WIDTH))

/* SPI read/write codes */

#define ICM_REG_READ    (0x80)
#define ICM_REG_WRITE   (0)
#define CONSTANTS_ONE_G (9.8f)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These structures are defined elsewhere, and we don't need their
 * definitions here.
 */

#ifdef CONFIG_ICM20689_SPI
struct spi_dev_s;
#endif 

#ifdef CONFIG_ICM20689_I2C
struct i2c_master_s;
#endif

/* Specifies the initial chip configuration and location.
 *
 * The chip supports both SPI and I2C interfaces, but you wouldn't use
 * both at the same time on the same chip. It isn't an error to have
 * one chip of each flavor in the system, though, so it's not an
 * either-or configuration item.
 *
 * Important note :
 *
 * The driver determines which interface type to use according to
 * which of the two groups of fields is non-NULL.  Since support for
 * I2C and SPI are individually configurable, however, users should
 * let the compiler clear unused fields instead of setting unused
 * fields to NULL directly. For example, if using SPI and a
 * stack-allocated instance:
 *
 *    struct icm20689_config_s icmc;
 *    memset(&icmc, 0, sizeof(icmc)); * sets i2c to NULL, if present *
 *    icmc.spi = ...;
 *
 * Or, if using dynamic memory allocation and I2C:
 *
 *    struct icm20689_config_s* icmc;
 *    icmc = kmm_malloc(sizeof(*icmc));
 *    memset(icmc, 0, sizeof(*icmc)); * sets spi to NULL, if present *
 *    icmc.i2c = ...;
 *
 * The above examples will avoid compile-time errors unless the user
 * forgets to enable their preferred interface type, and will allow
 * them to disable or enable the unused interface type without
 * changing their code.
 *
 */

struct icm20689_config_s
{
#ifdef CONFIG_ICM20689_SPI
  /* For users on SPI.
   *
   *  spi_devid : the SPI master's slave-select number
   *              for the chip, as used in SPI_SELECT(..., dev_id, ...)
   *  spi       : the SPI master device, as used in SPI_SELECT(spi, ..., ...)
   */

  FAR struct spi_dev_s *spi;
  int spi_devid;
#endif /* CONFIG_ICM20689_SPI */

#ifdef CONFIG_ICM20689_I2C
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
};

/* Describes the icm20689 sensor register file. This structure reflects
 * the underlying hardware, so don't change it!
 */

#pragma pack(push, 1)
struct sensor_data_s
{
  uint16_t x_accel;
  uint16_t y_accel;
  uint16_t z_accel;
  uint16_t temp;
  uint16_t x_gyro;
  uint16_t y_gyro;
  uint16_t z_gyro;
};
#pragma pack(pop)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Declares the existence of an icm20689 chip, wired according to
 * config; creates an interface to it at path.
 *
 * Returns 0 on success, or negative errno.
 */

int icm20689_register(int devno, FAR struct icm20689_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_ICM20689_H */
