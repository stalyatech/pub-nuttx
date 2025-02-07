/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/src/stm32_ramtron.c
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

#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "stalya-fmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RAMTRON_SPI_PORT  (4)

/* Configuration ************************************************************/

/* Can't support the RAMTRON device if it SPI4 or RAMTRON support is not enabled */

#define HAVE_RAMTRON  (1)
#if !defined(CONFIG_STM32H7_SPI4) || !defined(CONFIG_MTD_RAMTRON)
#  undef HAVE_RAMTRON
#endif

/* Can't support RAMTRON features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_RAMTRON
#endif

/* Can't support both FAT and SMARTFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_SMARTFS)
#  warning "Can't support both FAT and SMARTFS -- using FAT"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ramtron_initialize
 *
 * Description:
 *   Initialize and register the RAMTRON FLASH file system.
 *
 ****************************************************************************/

int board_ramtron_initialize(int minor)
{
  int ret = -ENODEV;
#ifdef HAVE_RAMTRON
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
#ifdef CONFIG_FS_NXFFS
  char devname[12];
#endif

  /* Get the SPI port */

  spi = stm32_spibus_initialize(RAMTRON_SPI_PORT);
  if (!spi)
    {
      ferr("ERROR: Failed to initialize SPI port %d\n", RAMTRON_SPI_PORT);
      return -ENODEV;
    }

  /* Now bind the SPI interface to the FM25 SPI FLASH driver */

  mtd = ramtron_initialize(spi);
  if (!mtd)
    {
      ferr("ERROR: Failed to bind SPI port %d to the FM25 FLASH driver\n", RAMTRON_SPI_PORT);
      return -ENODEV;
    }

#ifndef CONFIG_FS_NXFFS
  /* And use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(minor, mtd);
  if (ret < 0)
    {
      ferr("ERROR: Initialize the FTL layer\n");
      return ret;
    }
#else /* !CONFIG_FS_NXFFS */
  /* Initialize to provide NXFFS on the MTD interface */

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      ferr("ERROR: NXFFS initialization failed: %d\n", -ret);
      return ret;
    }

  /* Mount the file system at /fram */

  snprintf(devname, 12, "/nvm");
  ret = nx_mount(NULL, devname, "nxffs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_FS_NXFFS */
#endif /* HAVE_RAMTRON*/

  return ret;
}
