/****************************************************************************
 * boards/arm/stm32f7/ardusimple-sbc/src/stm32_bno085_i2c.c
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
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/sensors/bno085.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include "stm32_i2c.h"
#include "stm32_gpio.h"

#include "ardusimple-sbc.h"

#ifdef CONFIG_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BNO085_I2C_ADDRESS    (0x4A)
#define BNO085_I2C_FREQUENCY  (100000)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bno085_attach(struct bno085_config_s *cfg, xcpt_t irq, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only one BNO085 device on board */

static struct bno085_config_s g_bno085_config =
{
  .attach = bno085_attach,
  .irq    = BOARD_IMU_IRQ,
  .devid  = BNO085_I2C_ADDRESS,
  .freq   = BNO085_I2C_FREQUENCY
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bno085_attach()
 *
 * Description: Attach the bno085 interrupt handler to the GPIO interrupt
 *
 ****************************************************************************/

static int bno085_attach(struct bno085_config_s *cfg, xcpt_t irq, void *arg)
{
  return stm32_gpiosetevent(BOARD_IMU_GPIO_INT, false, true,
                            true, irq, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_bno085_initialize(int bus)
{
  struct i2c_master_s *i2c;
  int ret = ERROR;

  /* Initialize the I2C bus */

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      /* register the driver */

      ret = bno085_register("/dev/sensor0", i2c, &g_bno085_config);
    }

  return ret;
}

#endif /* CONFIG_I2C */
