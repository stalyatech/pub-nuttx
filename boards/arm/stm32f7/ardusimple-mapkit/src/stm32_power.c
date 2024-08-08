/****************************************************************************
 * boards/arm/stm32f7/ardusimple-mapkit/src/stm32_reset.c
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

#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>

#include "ardusimple-mapkit.h"

#ifdef CONFIG_BOARDCTL_POWEROFF

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_power_off
 *
 * Description:
 *   Power off the board.
 *
 *   If this function returns, then it was not possible to power-off the
 *   board due to some other constraints.
 *
 * Input Parameters:
 *   status - Status information provided with the power off event.
 *            This status is used as the power shutdown level.
 *            0= Deep Sleep, 1= Cold Sleep
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

int board_power_off(int status)
{
  struct file fd;
  int op, ret;

  /* Open the battery charger device */

  ret = file_open(&fd, CHARGER_DEVPATH, O_RDWR);
  if (ret < 0)
    {
      pwrerr("ERROR: open %s failed: %d\n", CHARGER_DEVPATH, ret);
      return ret;
    }

  /* Set system off mode */

  op = BATIO_OPRTN_SYSOFF;
  ret = file_ioctl(&fd, BATIOC_OPERATE, (unsigned long)(uintptr_t)&op);
  if (ret < 0)
    {
      pwrerr("ERROR: ioctl(BATIOC_OPERATE) failed: %d\n", errno);
      file_close(&fd);
      return ret;
    }

  /* Close battery charger device and exit. */

  file_close(&fd);
  return ret;
}

#endif /* CONFIG_BOARDCTL_POWEROFF */
