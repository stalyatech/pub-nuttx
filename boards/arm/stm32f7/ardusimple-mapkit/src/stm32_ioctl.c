/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_ioctl.c
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
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>

#include <arch/board/board.h>
#include <arch/board/boardctl.h>

#include "chip.h"
#include "ardusimple-mapkit.h"


#ifdef CONFIG_BOARDCTL_IOCTL

#ifdef CONFIG_STM32F7_PROGMEM_OTA_PARTITION

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMAGE_HEADER_MAGIC      0x534f584e
#define IMAGE_HEADER_MAGIC_INV  0xaca0abb1
#define IMAGE_PRERELEASE_MAXLEN 110

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Versioning is according to Semantic Versioning 2.0.0
 * refer to (https://semver.org/spec/v2.0.0.html)
 */

struct img_version
{
  uint16_t major;             /* MAJOR version */
  uint16_t minor;             /* MINOR version */
  uint16_t patch;             /* PATCH version */

  char pre_release[IMAGE_PRERELEASE_MAXLEN];  /* Additional pre-release version */
};

struct img_header
{
  uint32_t magic;             /* Header magic */
  uint32_t size;              /* Image size (excluding the header) */
  uint32_t crc;               /* CRC32 of image (excluding the header). */
  struct img_version version; /* Image version */
};
#endif /* CONFIG_STM32F7_PROGMEM_OTA_PARTITION */

/****************************************************************************
 * Private data
 ****************************************************************************/

static char g_version[18] = "X.X.X";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_STM32F7_PROGMEM_OTA_PARTITION

/****************************************************************************
 * Name: flash_partition_open
 *
 * Description:
 *   Opens the partition based on a given name and returns the file
 *   descriptor to it.
 *
 * Input parameters:
 *   path: Path to the device.
 *
 * Returned Value:
 *   Valid file descriptor on success, -1 on failure.
 *
 ****************************************************************************/

static int flash_partition_open(const char *path)
{
  int fd;

  fd = open(path, O_RDWR);
  if (fd < 0)
    {
      return ERROR;
    }

  return fd;
}

/****************************************************************************
 * Name: flash_partition_close
 *
 * Description:
 *   Closes opened partition.
 *
 * Input parameters:
 *   fd: Valid file descriptor.
 *
 * Returned Value:
 *   0 on success, -1 on failure.
 *
 ****************************************************************************/

static int flash_partition_close(int fd)
{
  return close(fd);
}

/****************************************************************************
 * Name: flash_partition_read
 *
 * Description:
 *   Read count data to buffer buf at offset off from a partition
 *   referenced by file descriptor fd.
 *
 * Input parameters:
 *   fd: Valid file descriptor.
 *   buf: The pointer where read data are stored.
 *   count: Number of bytes to be read.
 *   off: Read offset in bytes.
 *
 * Returned Value:
 *   0 on success, -1 on failure.
 *
 ****************************************************************************/

static int flash_partition_read(int fd, void *buf, size_t count, off_t off)
{
  int ret;
  off_t pos;
  size_t size;
  ssize_t nbytes;
  struct mtd_geometry_s geometry;

  ret = ioctl(fd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geometry));
  if (ret < 0)
    {
      return ERROR;
    }

  size = geometry.erasesize * geometry.neraseblocks;
  if (count + off > size)
    {
      return ERROR;
    }

  pos = lseek(fd, off, SEEK_SET);
  if (pos != off)
    {
      return ERROR;
    }

  nbytes = read(fd, buf, count);
  if (nbytes != count)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: get_image_ver
 *
 * Description:
 *   Gets the current application version.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   0 on success, -1 and sets errno on failure.
 *
 ****************************************************************************/

static int get_image_ver(struct img_header *header)
{
  int fd, ret;

  /* Open the partition */

  fd = flash_partition_open(CONFIG_STM32_OTA_PRIMARY_SLOT_DEVPATH);
  if (fd < 0)
    {
      return ERROR;
    }

  /* Read the partition */

  ret = flash_partition_read(fd, header, sizeof(struct img_header), 0);
  if (ret < 0)
    {
      /* Something went wrong, treat the partition as empty. */

      memset(header, 0, sizeof(struct img_header));
    }

  /* Check the image header */

  if ((header->magic != IMAGE_HEADER_MAGIC) &&
      (header->magic != IMAGE_HEADER_MAGIC_INV))
    {
      /* Something went wrong, treat the partition as empty. */

      memset(header, 0, sizeof(struct img_header));
    }

  /* Close the partition */

  return flash_partition_close(fd);
}
#endif /* CONFIG_STM32F7_PROGMEM_OTA_PARTITION */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ioctl
 *
 * Description:
 *   The "landing site" for much of the boardctl() interface. Generic board-
 *   control functions invoked via ioctl() get routed through here.
 *
 *   Since we don't do anything unusual at the moment, this function
 *   accomplishes nothing except avoid a missing-function linker error if
 *   CONFIG_BOARDCTL_IOCTL is selected.
 *
 * Input Parameters:
 *   cmd - IOCTL command being requested.
 *   arg - Arguments for the IOCTL.
 *
 * Returned Value:
 *   we don't yet support any boardctl IOCTLs.  This function always returns
 *  -ENOTTY which is the standard IOCTL return value when a command is not
 *  supported
 *
 ****************************************************************************/

int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  int ret = -ENOTTY;

  switch (cmd)
    {
      case BOARDIOC_OTA_GETVERSION:
      {
#ifdef CONFIG_STM32F7_PROGMEM_OTA_PARTITION
        /* Get the image version */

        struct img_header header;
        FAR const char **version;

        ret = get_image_ver(&header);
        if (ret >= 0)
          {
            snprintf(g_version, sizeof(g_version), "%d.%d.%d", header.version.major,
                                                               header.version.minor,
                                                               header.version.patch);
          }
#endif /* CONFIG_STM32F7_PROGMEM_OTA_PARTITION */

        /* Inform the application layer */

        version = (FAR const char **)((uintptr_t)arg);
        if (version != 0)
          {
            *version = (FAR const char *)((uintptr_t)&g_version);
            ret = OK;
          }
        break;
      }

      default:
        break;
    }

  return ret;
}

#endif /* CONFIG_BOARDCTL_IOCTL */
