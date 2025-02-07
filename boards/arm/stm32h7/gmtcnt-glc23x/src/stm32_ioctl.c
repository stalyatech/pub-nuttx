/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/src/stm32_ioctrl.c
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

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <arch/board/board.h>
#include <arch/board/boardctl.h>

#ifdef CONFIG_LIBFLASH
#include <flash/partition.h>
#endif

#include "gmtcnt-glc23x.h"

#ifdef CONFIG_BOARDCTL_IOCTL

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

int board_get_vector_table(const char *path, uint32_t hdr_size,
													 struct vector_table *vt);
int board_exe_vector_table(struct vector_table *vt);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_STM32H7_PROGMEM_OTA_PARTITION

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
#endif /* CONFIG_STM32H7_PROGMEM_OTA_PARTITION */

/****************************************************************************
 * Private data
 ****************************************************************************/

static char g_version[18] = "X.X.X";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_STM32H7_PROGMEM_OTA_PARTITION) && \
    defined(CONFIG_LIBFLASH)

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
#endif /* CONFIG_STM32H7_PROGMEM_OTA_PARTITION && CONFIG_LIBFLASH */

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
#if defined(CONFIG_STM32H7_PROGMEM_OTA_PARTITION) && \
    defined(CONFIG_LIBFLASH)
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
#else
#warning "Version info trusts the Flash library support!"
#endif /* CONFIG_STM32H7_PROGMEM_OTA_PARTITION && CONFIG_LIBFLASH */

        /* Inform the application layer */

        version = (FAR const char **)((uintptr_t)arg);
        if (version != 0)
          {
            *version = (FAR const char *)((uintptr_t)&g_version);
            ret = OK;
          }
        break;
      }

			/* Get the image vector table */

      case BOARDIOC_OTA_GETVECTOR:
      {
          FAR struct boardioc_image_info_s *info =
            (FAR struct boardioc_image_info_s *)arg;

          DEBUGASSERT(info != NULL);

          ret = board_get_vector_table(info->path, info->header_size, &info->vector_tabl);
				break;
			}

			/* Execute the image vector table */

      case BOARDIOC_OTA_EXEVECTOR:
      {
          FAR const struct boardioc_image_info_s *info =
            (FAR const struct boardioc_image_info_s *)arg;

          DEBUGASSERT(info != NULL);

          ret = board_exe_vector_table(&info->vector_tabl);
				break;
			}

      default:
        break;
    }

  return ret;
}

#endif /* CONFIG_BOARDCTL_IOCTL */
