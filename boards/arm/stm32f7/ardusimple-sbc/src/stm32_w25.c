/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_w25.c
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
#ifdef CONFIG_STM32F7_SPI3
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/drivers/drivers.h>
#  include <nuttx/fs/fs.h>
#endif
#include <arch/board/board.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include "stm32_spi.h"
#include "ardusimple-sbc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the watchdog
 * timer
 */

#define W25_SPI_PORT 3

/* Configuration ************************************************************/

/* Can't support the W25 device if it SPI3 or W25 support is not enabled */

#define HAVE_W25  1
#if !defined(CONFIG_STM32F7_SPI3) || !defined(CONFIG_MTD_W25)
#  undef HAVE_W25
#endif

/* Can't support W25 features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_W25
#endif

/* Default W25 minor number */

#if defined(HAVE_W25) && !defined(CONFIG_NSH_W25MINOR)
#  define CONFIG_NSH_W25MINOR 0
#endif

/* Can't support both FAT and SMARTFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_SMARTFS)
#  warning "Can't support both FAT and SMARTFS -- using FAT"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Initialize and register the W25 FLASH file system.
 *
 ****************************************************************************/

int stm32_w25initialize(int minor)
{
#ifdef HAVE_W25
  struct mtd_dev_s *part[CONFIG_ARDUSIMPLE_SBC_SPIFLASH_NPARTITIONS];
  struct mtd_dev_s *mtd;
  struct spi_dev_s *spi;
  struct mtd_geometry_s geo;
  int ret, i;
  off_t offset;
  off_t nblocks;
  uint32_t blkpererase;
  char blockname[32];
  char charname[32];
  char mntpoint[32];

  /* Get the SPI port */

  spi = stm32_spibus_initialize(W25_SPI_PORT);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n", W25_SPI_PORT);
      return -ENODEV;
    }

  /* Now bind the SPI interface to the W25 SPI FLASH driver */

  mtd = w25_initialize(spi);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port %d to the W25 FLASH driver\n", W25_SPI_PORT);
      return -ENODEV;
    }

  /* Get the device geometry */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY,
                        (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

  /* Determine the size of each partition.  Make each partition an even
   * multiple of the erase block size (perhaps not using some space at the
   * end of the FLASH).
   */

  blkpererase = geo.erasesize / geo.blocksize;
  nblocks     = (geo.neraseblocks / CONFIG_ARDUSIMPLE_SBC_SPIFLASH_NPARTITIONS) *
                blkpererase;

  /* Now create MTD FLASH partitions */

  syslog(LOG_ERR, "INFO: Creating partitions\n");

  for (offset = 0, i = 0;
       i < CONFIG_ARDUSIMPLE_SBC_SPIFLASH_NPARTITIONS;
       offset += nblocks, i++)
    {
      syslog(LOG_INFO, "INFO: Partition %d. Block offset=%lu, size=%lu\n",
                       i, (unsigned long)offset, (unsigned long)nblocks);

      /* Create the partition */

      part[i] = mtd_partition(mtd, offset, nblocks);
      if (!part[i])
        {
          syslog(LOG_ERR, "ERROR: Failed to create first MTD partition\n");
          return -ENODEV;
        }

      /* Initialize to provide an FTL block driver on the MTD FLASH
       * interface
       */

      snprintf(blockname, sizeof(blockname), "/dev/mtdblock%d", i);
      snprintf(charname, sizeof(charname), "/dev/mtd%d", i);
      snprintf(mntpoint, sizeof(mntpoint), "/data%d", i);

      ret = ftl_initialize(i, part[i]);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: ftl_initialize %s failed: %d\n", blockname, ret);
          return ret;
        }

      /* Now create a character device on the block device */

      ret = bchdev_register(blockname, charname, false);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n", charname, ret);
          return ret;
        }

      /* Initialize to provide flash file system on the MTD partitions */

      ret = board_spiflash_init(part[i], blockname, mntpoint);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Flash file system initialization failed: %d\n", -ret);
          return ret;
        }
    }
#endif /* HAVE_W25 */

  return OK;
}
