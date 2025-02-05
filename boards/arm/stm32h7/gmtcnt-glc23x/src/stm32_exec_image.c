/****************************************************************************
 * boards/arm/stm32h7/gmtcnt-glc23x/src/stm32_exec_image.c
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

#include <debug.h>
#include <stdio.h>
#include <fcntl.h>

#include <arch/board/board.h>
#include <arch/board/boardctl.h>

#include "barriers.h"

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_get_vector_table
 *
 * Description:
 *   This entry point is called by logic loader to get application 
 * 	 information.
 *
 ****************************************************************************/

int board_get_vector_table(const char *path, uint32_t hdr_size,
													 struct vector_table *vt)
{
  struct file file;
  ssize_t bytes;
  int ret;

  ret = file_open(&file, path, O_RDONLY | O_CLOEXEC);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to open %s with: %d", path, ret);
      return ret;
    }

  bytes = file_pread(&file, vt, sizeof(struct vector_table), hdr_size);
  if (bytes != sizeof(vt))
    {
      syslog(LOG_ERR, "Failed to read ARM vector table: %d", bytes);
      return bytes < 0 ? bytes : -1;
    }

  return 0;
}

/****************************************************************************
 * Name: board_exe_vector_table
 *
 * Description:
 *   This entry point is called by logic loader to jump to application image.
 *
 ****************************************************************************/
int board_exe_vector_table(const struct vector_table *vt)
{
  /* Set main and process stack pointers */

  __asm__ __volatile__("\tmsr msp, %0\n" : : "r" (vt->spr));
  setcontrol(0x00);
  ARM_ISB();
  ((void (*)(void))vt->reset)();

  return 0;
}
