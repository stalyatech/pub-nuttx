/****************************************************************************
 * drivers/ioexpander/pca9557.h
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

/* References:
 *   "8-bit I2C-bus and SMBus I/O port with interrupt product datasheet",
 *   Rev. 08 - 22 October 2009, NXP
 */

#ifndef __DRIVERS_IOEXPANDER_PCA9557_H
#define __DRIVERS_IOEXPANDER_PCA9557_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pca9557.h>

#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_PCA9557)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *   CONFIG_I2C
 *     I2C support is required
 *   CONFIG_IOEXPANDER
 *     Enables support for the PCA9557 I/O expander
 *
 * CONFIG_IOEXPANDER_PCA9557
 *   Enables support for the PCA9557 driver (Needs CONFIG_INPUT)
 * CONFIG_PCA9557_MULTIPLE
 *   Can be defined to support multiple PCA9557 devices on board.
 * 
 */

#undef CONFIG_PCA9557_REFCNT

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.
 *  It defined here
 * so that it will be used consistently in all places.
 */

/* PCA9557 Resources ********************************************************/

#define PCA9557_GPIO_NPINS  8 /* All pins can be used as GPIOs */

#ifndef CONFIG_I2C
#error "CONFIG_I2C is required by PCA9557"
#endif

#define PCA9557_MAXDEVS             8

/* I2C frequency */

#define PCA9557_I2C_MAXFREQUENCY    400000       /* 400KHz */

/* PCA9557 Registers ********************************************************/

/* Register Addresses */

#define PCA9557_REG_INPUT  0x00
#define PCA9557_REG_OUTPUT 0x01
#define PCA9557_REG_POLINV 0x02
#define PCA9557_REG_CONFIG 0x03

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents the state of the PCA9557 driver */

struct pca9557_dev_s
{
  struct ioexpander_dev_s      dev;      /* Nested structure to allow casting as public gpio
                                          * expander. */
#ifdef CONFIG_PCA9557_SHADOW_MODE
  uint8_t sreg[4];                       /* Shadowed registers of the PCA9557 */
#endif
#ifdef CONFIG_PCA9557_MULTIPLE
  FAR struct pca9557_dev_s    *flink;    /* Supports a singly linked list of drivers */
#endif
  FAR struct pca9557_config_s *config;   /* Board configuration data */
  FAR struct i2c_master_s     *i2c;      /* Saved I2C driver instance */
  mutex_t                      lock;     /* Mutual exclusion */
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_PCA9557 */
#endif /* __DRIVERS_IOEXPANDER_PCA9557_H */
