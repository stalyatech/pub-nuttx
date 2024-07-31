/****************************************************************************
 * boards/arm/stm32f7/ardusimple-mapkit/src/stm32_gps.c
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
#include <nuttx/kthread.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/sensors/ublox_gps.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include "ardusimple-mapkit.h"

#ifdef CONFIG_SENSORS_GPS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gps_initialize
 *
 * Description:
 *   GPS driver registration
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_gps_initialize(void)
{
  int ret = ERROR;

#ifdef CONFIG_SENSORS_UBLOX_GPS
#ifdef CONFIG_SENSORS_UBLOX_GPS1_PORT
  ret = ublox_gps_register(CONFIG_SENSORS_UBLOX_GPS1_DEVPATH, 
                           CONFIG_SENSORS_UBLOX_GPS1_BAUDRATE,
                           0, CONFIG_SENSORS_UBLOX_NBUFFERS);

  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize GPS1 uORB driver: %d\n", ret);
    }
#endif /* CONFIG_SENSORS_UBLOX_GPS1_PORT */

#ifdef CONFIG_SENSORS_UBLOX_GPS2_PORT
  ret = ublox_gps_register(CONFIG_SENSORS_UBLOX_GPS2_DEVPATH, 
                           CONFIG_SENSORS_UBLOX_GPS2_BAUDRATE,
                           1, CONFIG_SENSORS_UBLOX_NBUFFERS);

  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize GPS2 uORB driver: %d\n", ret);
    }
#endif /* CONFIG_SENSORS_UBLOX_GPS2_PORT */

#ifdef CONFIG_SENSORS_UBLOX_GPS3_PORT
  ret = ublox_gps_register(CONFIG_SENSORS_UBLOX_GPS3_DEVPATH, 
                           CONFIG_SENSORS_UBLOX_GPS3_BAUDRATE,
                           2, CONFIG_SENSORS_UBLOX_NBUFFERS);

  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize GPS3 uORB driver: %d\n", ret);
    }
#endif /* CONFIG_SENSORS_UBLOX_GPS1_PORT */

#ifdef CONFIG_SENSORS_UBLOX_GPS4_PORT
  ret = ublox_gps_register(CONFIG_SENSORS_UBLOX_GPS4_DEVPATH, 
                           CONFIG_SENSORS_UBLOX_GPS4_BAUDRATE,
                           3, CONFIG_SENSORS_UBLOX_NBUFFERS);

  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize GPS4 uORB driver: %d\n", ret);
    }
#endif /* CONFIG_SENSORS_UBLOX_GPS4_PORT */

#ifdef CONFIG_SENSORS_UBLOX_GPS5_PORT
  ret = ublox_gps_register(CONFIG_SENSORS_UBLOX_GPS5_DEVPATH, 
                           CONFIG_SENSORS_UBLOX_GPS5_BAUDRATE,
                           4, CONFIG_SENSORS_UBLOX_NBUFFERS);

  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize GPS5 uORB driver: %d\n", ret);
    }
#endif /* CONFIG_SENSORS_UBLOX_GPS5_PORT */
#endif /* CONFIG_SENSORS_UBLOX_GPS */

  return ret;
}

#endif /* CONFIG_SENSORS_GPS */
