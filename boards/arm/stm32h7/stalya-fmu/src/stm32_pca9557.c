/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/src/stm32_pca9557.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/pca9557.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/gpio.h>

#include "stm32.h"
#include "stalya-fmu.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct pca9557_config_s g_pca9557_cfg =
{
  .address   = PCA9557_I2CADDR,
  .frequency = PCA9557_I2CFREQ,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pca9557_pinscfg
 ****************************************************************************/

static void stm32_pca9557_pinscfg(struct ioexpander_dev_s *ioe)
{
  /* Pin 0: XBee Sockets Power (Default: On) */

  IOEXP_SETDIRECTION(ioe, 0, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 0, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 0, true);

  /* Pin 1: RTCM source selection (Default: Off) */

  IOEXP_SETDIRECTION(ioe, 1, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 1, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 1, false);

  /* Pin 2: GPS Power (Default: On) */

  IOEXP_SETDIRECTION(ioe, 2, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 2, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 2, true);

  /* Pin 3: IMU power (Default: On) */

  IOEXP_SETDIRECTION(ioe, 3, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 3, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 3, true);

  /* Pin 4: IMU heating (Default: Off) */

  IOEXP_SETDIRECTION(ioe, 4, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 4, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 4, false);

  /* Pin 5: Peripheral powers (Default: On) */

  IOEXP_SETDIRECTION(ioe, 5, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 5, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 5, true);

  /* Pin 6: USB power (Default: On) */

  IOEXP_SETDIRECTION(ioe, 6, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 6, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 6, true);

  /* Pin 7: I/O processor power (Default: On) */

  IOEXP_SETDIRECTION(ioe, 7, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(ioe, 7, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_WRITEPIN(ioe, 7, true);

#ifdef CONFIG_GPIO_LOWER_HALF
  gpio_lower_half(ioe, 0, GPIO_OUTPUT_PIN_OPENDRAIN, 0);
  gpio_lower_half(ioe, 1, GPIO_OUTPUT_PIN, 1);
  gpio_lower_half(ioe, 2, GPIO_OUTPUT_PIN, 2);
  gpio_lower_half(ioe, 3, GPIO_OUTPUT_PIN, 3);
  gpio_lower_half(ioe, 4, GPIO_OUTPUT_PIN, 4);
  gpio_lower_half(ioe, 5, GPIO_OUTPUT_PIN, 5);
  gpio_lower_half(ioe, 6, GPIO_OUTPUT_PIN, 6);
  gpio_lower_half(ioe, 7, GPIO_OUTPUT_PIN, 7);
#endif /* CONFIG_GPIO_LOWER_HALF */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pca9557_initialize
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   I/O expander. This function will register the driver as /dev/ioe0.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_pca9557_initialize(void)
{
  struct i2c_master_s     *i2c = NULL;
  struct ioexpander_dev_s *ioe = NULL;
  int                      ret = OK;

  /* Configure pca9557 pins */

  stm32_configgpio(GPIO_PCA9557_NRST);

  /* Get the I2C driver that interfaces with the pca9557 */

  i2c = stm32_i2cbus_initialize(PCA9557_I2CBUS);
  if (!i2c)
    {
      i2cerr("ERROR: Failed to initialize I2C%d\n", PCA9557_I2CBUS);
      return -ENODEV;
    }

  /* Initialize pca9557 */

  ioe = pca9557_initialize(i2c, &g_pca9557_cfg);
  if (ioe == NULL)
    {
      i2cerr("ERROR: Failed to initialize sx1509 %d\n", ret);
      return ret;
    }

  /* Register pins */

  stm32_pca9557_pinscfg(ioe);

  return OK;
}
