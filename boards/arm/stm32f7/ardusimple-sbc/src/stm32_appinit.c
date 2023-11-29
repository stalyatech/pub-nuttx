/****************************************************************************
 * boards/arm/stm32f7/ardusimple-sbc/src/stm32_appinit.c
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
#include <sys/types.h>
#include <stdio.h>

#include "stm32_uid.h"
#include "ardusimple-sbc.h"

#ifdef CONFIG_BOARDCTL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_usbdev_serialstr
 *
 * Description:
 *   Use board unique serial number string to iSerialNumber field in the
 *   device descriptor. This is for determining the board when multiple
 *   boards on the same host.
 *
 * Returned Value:
 *   The board unique serial number string.
 *
 ****************************************************************************/

#if defined(CONFIG_BOARD_USBDEV_SERIALSTR)
static char g_serialstr[CONFIG_BOARDCTL_UNIQUEID_SIZE * 2 + 1];

FAR const char *board_usbdev_serialstr(void)
{
  uint8_t uid[CONFIG_BOARDCTL_UNIQUEID_SIZE];

  stm32_get_uniqueid(uid);

  snprintf(g_serialstr, sizeof(g_serialstr),
           "%02X%02X%02X%02X%02X",
           uid[0], uid[1], uid[2], uid[3], uid[4]);

  return g_serialstr;
}
#endif

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifndef CONFIG_BOARD_LATE_INITIALIZE
  /* Perform board-specific initialization */

  return stm32_bringup();
#else
  return OK;
#endif
}

#endif
