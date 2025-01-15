/****************************************************************************
 * boards/arm/stm32f7/ardusimple-mapkit/src/stm32_progmem.c
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

#include <sys/mount.h>
#include <sys/param.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/progmem.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#ifdef CONFIG_BCH
#include <nuttx/drivers/drivers.h>
#endif

#include "chip.h"
#include "hardware/stm32_flash.h"
#include "ardusimple-mapkit.h"

#ifdef HAVE_PROGMEM_CHARDEV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PARTITION_LABEL_LEN   16

/* Configuration ************************************************************/

/* Make sure that support for MTD partitions is enabled */
#ifdef CONFIG_MTD

#ifndef CONFIG_MTD_PARTITION
#  error "CONFIG_MTD_PARTITION is required"
#endif

#endif
/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_STM32F7_PROGMEM_OTA_PARTITION)
struct ota_partition_s
{
  uint32_t    offset;          /* Partition offset from the beginning of MTD */
  uint32_t    size;            /* Partition size in bytes */
  const char *devpath;         /* Partition device path */
};
#endif

/* Progmem partition data structure */

struct progmem_part_s
{
  const struct ota_partition_s *ota;
  struct mtd_dev_s *mtd;
  struct mtd_geometry_s geo;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_STM32F7_PROGMEM_OTA_PARTITION)
static struct mtd_dev_s *progmem_alloc_mtdpart(uint32_t mtd_offset,
                                               uint32_t mtd_size);
static int init_ota_partitions(void);
#endif

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     progmem_blk_open(struct inode *inode);
static int     progmem_blk_close(struct inode *inode);
#endif
static ssize_t progmem_blk_read(struct inode *inode, unsigned char *buffer,
                 blkcnt_t start_sector, unsigned int nsectors);
static ssize_t progmem_blk_write(struct inode *inode,
                 const unsigned char *buffer, blkcnt_t start_sector,
                 unsigned int nsectors);
static int     progmem_blk_geometry(struct inode *inode,
                 struct geometry *geometry);
static int     progmem_blk_ioctl(struct inode *inode, int cmd,
                 unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     progmem_blk_unlink(struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_STM32F7_PROGMEM_OTA_PARTITION)
static const struct ota_partition_s g_ota_partition_table[] =
{
  {
    .offset  = CONFIG_STM32_OTA_PRIMARY_SLOT_OFFSET,
    .size    = CONFIG_STM32_OTA_SLOT_SIZE,
    .devpath = CONFIG_STM32_OTA_PRIMARY_SLOT_DEVPATH
  }
};
static struct progmem_part_s g_progmem_part[nitems(g_ota_partition_table)];
#else
static struct progmem_part_s g_progmem_part[1];
#endif
static struct mtd_dev_s     *g_progmem_mtd;

static const struct block_operations g_bops =
{
  progmem_blk_open,     /* open     */
  progmem_blk_close,    /* close    */
  progmem_blk_read,     /* read     */
  progmem_blk_write,    /* write    */
  progmem_blk_geometry, /* geometry */
  progmem_blk_ioctl,    /* ioctl    */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  progmem_blk_unlink    /* unlink   */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: progmem_blk_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int progmem_blk_open(struct inode *inode)
{
  struct progmem_part_s *partition;

  finfo("Entry\n");

  DEBUGASSERT(inode->i_private);
  partition = inode->i_private;
  UNUSED(partition);

  return OK;
}

/****************************************************************************
 * Name: progmem_blk_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int progmem_blk_close(struct inode *inode)
{
  struct progmem_part_s *partition;

  finfo("Entry\n");

  DEBUGASSERT(inode->i_private);
  partition = inode->i_private;
  UNUSED(partition);

  return OK;
}

/****************************************************************************
 * Name: progmem_blk_read
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t progmem_blk_read(struct inode *inode, unsigned char *buffer,
                                blkcnt_t start_sector, unsigned int nsectors)
{
  ssize_t nread;
  struct progmem_part_s *partition;

  finfo("sector: %" PRIuOFF " nsectors: %u\n", start_sector, nsectors);

  DEBUGASSERT(inode->i_private);
  partition = inode->i_private;

  nread = MTD_BREAD(partition->mtd, start_sector, nsectors, buffer);
  if (nread != nsectors)
    {
      finfo("Read %u blocks starting at block %" PRIuOFF " failed: %d\n",
            nsectors, start_sector, nread);
      return -EIO;
    }

  return nread;
}

/****************************************************************************
 * Name: progmem_blk_write
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

static ssize_t progmem_blk_write(struct inode *inode,
                                 const unsigned char *buffer,
                                 blkcnt_t start_sector, unsigned int nsectors)
{
  ssize_t nwrite;
  struct progmem_part_s *partition;

  finfo("sector: %" PRIuOFF " nsectors: %u\n", start_sector, nsectors);

  DEBUGASSERT(inode->i_private);
  partition = inode->i_private;

  nwrite = MTD_BWRITE(partition->mtd, start_sector, nsectors, buffer);
  if (nwrite != nsectors)
    {
      finfo("Write %u blocks starting at block %" PRIuOFF " failed: %d\n",
            nsectors, start_sector, nwrite);
      return -EIO;
    }

  return nwrite;
}

/****************************************************************************
 * Name: progmem_blk_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int progmem_blk_geometry(struct inode *inode, struct geometry *geometry)
{
  struct progmem_part_s *partition;

  finfo("Entry\n");

  DEBUGASSERT(inode->i_private);
  partition = inode->i_private;

  if (geometry)
    {
      memset(geometry, 0, sizeof(struct geometry));
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
      geometry->geo_writeenabled  = true;
      geometry->geo_nsectors      = (partition->geo.neraseblocks * partition->geo.erasesize) /
                                     partition->geo.blocksize;
      geometry->geo_sectorsize    = partition->geo.blocksize;
      strlcpy(geometry->geo_model, partition->geo.model, NAME_MAX);

      finfo("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");

      finfo("nsectors: %" PRIuOFF " sectorsize: %" PRIi16 "\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: progmem_blk_ioctl
 *
 * Description:
 *   Return device geometry
 *
 ****************************************************************************/

static int progmem_blk_ioctl(struct inode *inode, int cmd, unsigned long arg)
{
  struct progmem_part_s *partition;
  int ret;

  finfo("Entry\n");

  DEBUGASSERT(inode->i_private);
  partition = inode->i_private;

  ret = MTD_IOCTL(partition->mtd, cmd, arg);
  if (ret < 0)
    {
      finfo("ERROR: MTD ioctl(%04x) failed: %d\n", cmd, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: progmem_blk_unlink
 *
 * Description:
 *   The block driver has been unlinked.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int progmem_blk_unlink(struct inode *inode)
{
  struct progmem_part_s *partition;

  DEBUGASSERT(inode->i_private);
  partition = inode->i_private;
  UNUSED(partition);

  return OK;
}
#endif

#if defined(CONFIG_STM32F7_PROGMEM_OTA_PARTITION)

/****************************************************************************
 * Name: sam_progmem_alloc_mtdpart
 *
 * Description:
 *   Allocate an MTD partition from FLASH.
 *
 * Input Parameters:
 *   mtd_offset - MTD Partition offset from the base address in FLASH.
 *   mtd_size   - Size for the MTD partition.
 *
 * Returned Value:
 *   MTD partition data pointer on success, NULL on failure.
 *
 ****************************************************************************/

static struct mtd_dev_s *progmem_alloc_mtdpart(uint32_t mtd_offset,
                                               uint32_t mtd_size)
{
  uint32_t blocks;
  ssize_t startblock;

  ASSERT((mtd_offset % up_progmem_pagesize(0)) == 0);
  ASSERT((mtd_size % up_progmem_pagesize(0)) == 0);

  finfo("\tMTD offset = 0x%"PRIx32"\n", mtd_offset);
  finfo("\tMTD size = 0x%"PRIx32"\n", mtd_size);

  startblock = up_progmem_getpage(mtd_offset + up_progmem_getaddress(0));
  if (startblock < 0)
    {
      return NULL;
    }

  blocks = mtd_size / up_progmem_pagesize(0);

  return mtd_partition(g_progmem_mtd, startblock, blocks);
}

/****************************************************************************
 * Name: init_ota_partitions
 *
 * Description:
 *   Initialize partitions that are dedicated to firmware OTA update.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int init_ota_partitions(void)
{
  char path[PARTITION_LABEL_LEN + 1];
  int i, ret = 0;

  for (i = 0; i < nitems(g_ota_partition_table); ++i)
    {
      const struct ota_partition_s *ota = &g_ota_partition_table[i];
      g_progmem_part[i].ota = ota;
      g_progmem_part[i].mtd = progmem_alloc_mtdpart(ota->offset, ota->size);

      strlcpy(path, (char *)ota->devpath, PARTITION_LABEL_LEN);

      finfo("INFO: [label]:   %s\n", path);
      finfo("INFO: [offset]:  0x%08" PRIx32 "\n", ota->offset);
      finfo("INFO: [size]:    0x%08" PRIx32 "\n", ota->size);

      if (!g_progmem_part[i].mtd)
        {
          ferr("ERROR: Failed to create MTD partition\n");
          ret = -1;
        }

      /* Get the device geometry info */

      ret = MTD_IOCTL(g_progmem_part[i].mtd,
                      MTDIOC_GEOMETRY,
                      (unsigned long)((uintptr_t)&g_progmem_part[i].geo));
      if (ret < 0)
        {
          ferr("ERROR: Failed to read geometry MTD partition");
          ret = -1;
        }

      /* Register the MTD partitions as a block device */

      ret = register_blockdriver(path, &g_bops, 0777, &g_progmem_part[i]);
      if (ret < 0)
        {
          ferr("ERROR: Failed to register MTD @ %s\n", path);
          ret = -1;
        }
    }

  return ret;
}
#endif /* CONFIG_STM32F7_PROGMEM_OTA_PARTITION */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_progmem_init
 *
 *   Initialize Progmem partition. Read partition information, and use
 *   these data for creating MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int stm32_progmem_init(void)
{
  int ret = 0;

#ifdef CONFIG_STM32F7_PROGMEM_OTA_PARTITION
  const struct ota_partition_s *ota = &g_ota_partition_table[0];

  g_progmem_mtd = progmem_initialize(ota->offset);
  if (g_progmem_mtd == NULL)
    {
      ferr("ERROR: Failed to get progmem flash MTD\n");
      ret = -EIO;
    }
  g_progmem_mtd->name = "progmem";

  ret = init_ota_partitions();
  if (ret < 0)
    {
      ferr("ERROR: Failed to create OTA partition from MTD\n");
      ret = -EIO;
    }
#endif

  return ret;
}

#endif /* HAVE_PROGMEM_CHARDEV */
