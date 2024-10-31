/****************************************************************************
 * drivers/sensors/ms5611_base.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MS5611_COMMOM_H
#define __INCLUDE_NUTTX_SENSORS_MS5611_COMMOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ms5611.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <math.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Specific device id*/

#define MS5611_DEVID  (0x98)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ms5611_calib_s
{
  uint16_t reversed;
  uint16_t c1;
  uint16_t c2;
  uint16_t c3;
  uint16_t c4;
  uint16_t c5;
  uint16_t c6;
  uint16_t crc;
};

struct ms5611_sensor_s
{
  struct sensor_lowerhalf_s
                lower;    /* Lower half sensor driver. */
  struct ms5611_calib_s
                calib;    /* Calib. params from ROM */
  uint64_t      last_update;
  bool          enabled;
  unsigned long interval; /* Polling interval */
  FAR void      *dev;     /* The pointer to common device data of ms5611 */
};

/* Used by the driver to manage the device */

struct ms5611_dev_s
{
  struct ms5611_sensor_s priv;
  struct ms5611_config_s
                config;   /* board-specific information */
  mutex_t       lock;     /* mutex for this structure */
  struct work_s work;     /* Interrupt handler worker. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int ms5611_read(FAR struct ms5611_dev_s *dev,
                FAR uint8_t *buf, uint8_t len);

int ms5611_write(FAR struct ms5611_dev_s *dev,
                 FAR const uint8_t *buf, uint8_t len);

int ms5611_transfer(FAR struct ms5611_dev_s *dev,
                    FAR const uint8_t *txbuf, uint8_t txlen,
                    FAR uint8_t *rxbuf, uint8_t rxlen);

#endif /* __INCLUDE_NUTTX_SENSORS_MS5611_COMMOM_H */
