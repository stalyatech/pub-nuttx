/****************************************************************************
 * boards/arm/stm32f7/ardusimple-mapkit/src/stm32_bno055.c
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
#include <nuttx/sensors/bno055.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include "stm32_i2c.h"
#include "stm32_gpio.h"

#include "ardusimple-mapkit.h"

#ifdef CONFIG_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef BOARD_IMU_IRQ
static int bno055_attach(struct bno055_config_s *cfg, xcpt_t irq, void *arg);
#endif /* BOARD_IMU_IRQ */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only one BNO055 device on board */

static struct bno055_config_s g_bno055_config =
{
  .dev    = NULL,
  .devid  = BNO055_I2C_ADDR,
  .freq   = BNO055_I2C_FREQ,
#ifdef BOARD_IMU_IRQ
  .irq    = BOARD_IMU_IRQ,
  .attach = bno055_attach,
#endif /* BOARD_IMU_IRQ */
  .pwr_mode = BNO055_PWMODE_NORMAL,
  .opr_mode = BNO055_OPMODE_RAW,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bno055_attach()
 *
 * Description: Attach the bno055 interrupt handler to the GPIO interrupt
 *
 ****************************************************************************/
#if defined(BOARD_IMU_IRQ) && defined(BOARD_IMU_GPIO_INT)
static int bno055_attach(struct bno055_config_s *cfg, xcpt_t irq, void *arg)
{
  return stm32_gpiosetevent(BOARD_IMU_GPIO_INT, false, true,
                            true, irq, arg);
}
#endif /* BOARD_IMU_IRQ && BOARD_IMU_GPIO_INT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_bno055_initialize(void)
{
  int ret = ERROR;

  /* Initialize the I2C bus */

  g_bno055_config.dev = stm32_i2cbus_initialize(BNO055_I2C_BUS);
  if (g_bno055_config.dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", BNO055_I2C_BUS);
    }
  else
    {
      /* register the driver */

      ret = bno055_register("/dev/imu0", &g_bno055_config);
    }

  return ret;
}

#endif /* CONFIG_I2C */
