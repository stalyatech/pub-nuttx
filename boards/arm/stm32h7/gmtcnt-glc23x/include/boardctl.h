/****************************************************************************
 * boards/arm/stm32h7/gmtcnt-glc23x/include/boardctl.h
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

#ifndef __BOARDS_ARM_STM32H7_GMTCNT_GLC23X_INCLUDE_BOARDCTL_H
#define __BOARDS_ARM_STM32H7_GMTCNT_GLC23X_INCLUDE_BOARDCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/boardctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARDIOC_OTA_GETVERSION   (BOARDIOC_USER+(1))
#define BOARDIOC_OTA_GETVECTOR   	(BOARDIOC_USER+(2))
#define BOARDIOC_OTA_EXEVECTOR   	(BOARDIOC_USER+(3))

/****************************************************************************
 * Public types
 ****************************************************************************/

/* This structure represents the first two entries on NVIC vector table */

typedef struct vector_table
{
  uint32_t spr;  								/* Stack pointer on reset */
  uint32_t reset; 							/* Pointer to reset exception handler */
} vector_table_s;

/* Structure containing the arguments to the BOARDIOC_OTA_BOOTIMAGE command */

struct boardioc_image_info_s
{
  FAR const char *path;       	/* Path to application firmware image */
  uint32_t        header_size;  /* Size of the image header in bytes */
	vector_table_s 	vector_tabl;	/* ARM NVIC vector table */
};

#endif /* __BOARDS_ARM_STM32L4_STM32L4R9AI_DISCO_INCLUDE_BOARDCTL_H */
