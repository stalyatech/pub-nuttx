/****************************************************************************
 * boards/arm/stm32f7/ardusimple-mapkit/src/stm32_bq2429x.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_charger.h>

#include "stm32_i2c.h"
#include "stm32_gpio.h"

#include "ardusimple-mapkit.h"


/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_BQ2429X

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bq2429x_initialize
 *
 * Description:
 *   Called to configure bq2429x
 *
 ****************************************************************************/

int stm32_bq2429x_initialize(const char *devname)
{
  FAR struct battery_charger_dev_s *bq2429x;
  FAR struct i2c_master_s *i2c;

  /* get the internal U2C bus instance */

  i2c = stm32_i2cbus_initialize(BQ2429X_I2C_BUS);

  /* initialize the driver */

  bq2429x = (FAR struct battery_charger_dev_s *)bq2429x_initialize( i2c,
                                                                    BQ2429X_I2C_ADDR,
                                                                    BQ2429X_I2C_FREQ,
                                                                    500);
  return battery_charger_register(devname, bq2429x);
}

#endif /* CONFIG_BQ2429X */
