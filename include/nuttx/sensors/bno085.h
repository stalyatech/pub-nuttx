/****************************************************************************
 * include/nuttx/sensors/bno085.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BNO085_H
#define __INCLUDE_NUTTX_SENSORS_BNO085_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/ioctl.h>

#include <nuttx/sensors/ceva/sh2/sh2_SensorValue.h>
#include <nuttx/sensors/ceva/sh2/euler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

#define SNIOC_RESET       _SNIOC(0x0001)    /* Arg: None          */
#define SNIOC_GETSTATUS   _SNIOC(0x0002)    /* Arg: int*  pointer */
#define SNIOC_SETCONFIG   _SNIOC(0x0003)    /* Arg: void* pointer */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the BNO085
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct bno085_config_s
{
    /* Device interface */

#ifdef CONFIG_SENSORS_BNO085_I2C
  FAR struct i2c_master_s *dev;
#else
  FAR struct spi_dev_s *dev;
#endif

  /* Since multiple BNO085 can be connected to the same bus we need
   * to use multiple device ids which are employed by NuttX to select/
   * deselect the desired BNO085 chip via their chip select inputs.
   */

  int devid;

  /* Frequency */

  int freq;

  /* The IRQ number must be provided for each BNO085 device so that
   * their interrupts can be distinguished.
   */

  int irq;

  /* Attach the BNO085 interrupt handler to the GPIO interrupt of the
   * concrete BNO085 instance.
   */

  int (*attach)(FAR struct bno085_config_s *, xcpt_t, void *);
};

/*
 * BNO085 Report configuration
 *
 */
struct bno085_reports_s
{
  /* Sensor Id */

  int sensorId;

  /* Report Configuration */

  sh2_SensorConfig_t config;
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
 * Name: bno085_register
 *
 * Description:
 *   Register the BNO085 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/sensor0"
 *   config  - Configuration of the SPI/I2C interface to use to communicate
 *             with BNO085
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bno085_register(FAR const char *devpath, 
                    FAR struct bno085_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_BNO085_H */
