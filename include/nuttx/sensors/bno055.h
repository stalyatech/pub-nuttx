/****************************************************************************
 * include/nuttx/sensors/bno055.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BNO055_H
#define __INCLUDE_NUTTX_SENSORS_BNO055_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/ioctl.h>

#include <nuttx/sensors/bosch/bno055.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sensor Power Modes *******************************************************/

#define BNO055_PWMODE_NORMAL    (BNO055_POWER_MODE_NORMAL)
#define BNO055_PWMODE_LOWPOWER  (BNO055_POWER_MODE_LOWPOWER)
#define BNO055_PWMODE_SUSPEND   (BNO055_POWER_MODE_SUSPEND)

/* Sensor Operation Modes *****************************************************/

#define BNO055_OPMODE_RAW       (BNO055_OPERATION_MODE_AMG)
#define BNO055_OPMODE_FUSION    (BNO055_OPERATION_MODE_NDOF)

/* Sensor Status ************************************************************/

#define BNO055_STATUS_READY     (1)
#define BNO055_STATUS_FAULT     (2)

/* IOCTL Commands ***********************************************************/

#define SNIOC_RESET           _SNIOC(0x0001)    /* Arg: None          */
#define SNIOC_GETSTATUS       _SNIOC(0x0002)    /* Arg: int*  pointer */
#define SNIOC_SETPOWERMODE    _SNIOC(0x0003)    /* Arg: int*  pointer */
#define SNIOC_SETOPERMODE     _SNIOC(0x0004)    /* Arg: int*  pointer */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the BNO055
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct bno055_config_s
{
    /* Device interface */

#ifdef CONFIG_SENSORS_BNO055_I2C
  FAR struct i2c_master_s *dev;
#else
  FAR struct spi_dev_s *dev;
#endif

  /* Since multiple BNO055 can be connected to the same bus we need
   * to use multiple device ids which are employed by NuttX to select/
   * deselect the desired BNO055 chip via their chip select inputs.
   */

  int devid;

  /* Frequency */

  int freq;

  /* The IRQ number must be provided for each BNO055 device so that
   * their interrupts can be distinguished.
   */

  int irq;

  /* Attach the BNO055 interrupt handler to the GPIO interrupt of the
   * concrete BNO055 instance.
   */

  int (*attach)(FAR struct bno055_config_s *, xcpt_t, void *);

  /* Sensor power mode */

  uint8_t pwr_mode;

  /* Sensor operation mode */

  uint8_t opr_mode;
};

/*
 * BNO055 Report configuration
 *
 */
struct bno055_reports_s
{
  /* Sensor operation mode */

  uint8_t opr_mode;

  /* Raw data */

  struct
  {
    struct bno055_accel_t accel;
    struct bno055_gyro_t gyro;
    struct bno055_mag_t mag;
  } raw;

  /* Quaternion data */

  struct
  {
    struct bno055_quaternion_t quat;
    struct bno055_euler_t euler;
  } fusion;
  
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
 * Name: bno055_register
 *
 * Description:
 *   Register the BNO055 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/imu0"
 *   config  - Configuration of the SPI/I2C interface to use to communicate
 *             with BNO055
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bno055_register(FAR const char *devpath,
                    FAR struct bno055_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_BNO055_H */
