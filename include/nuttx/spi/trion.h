/****************************************************************************
 * include/nuttx/spi/trion.h
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

#ifndef __INCLUDE_NUTTX_SPI_TRION_H
#define __INCLUDE_NUTTX_SPI_TRION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if(defined(CONFIG_SPI) && defined(CONFIG_SPI_TRION))

#define TRION_SPI_FINAL_CLK_CYCLES	200
#define TRION_SPI_MAX_XFER 					4096

#define FPGAIOC_WRITE_INIT 					_FPGACFGIOC(0x0001)
#define FPGAIOC_WRITE 							_FPGACFGIOC(0x0002)
#define FPGAIOC_WRITE_COMPLETE 			_FPGACFGIOC(0x0003)

/****************************************************************************
 * Public Function Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: trion_register
 *
 * Description:
 *   Register the Trion character device as 'devpath'.
 *
 ****************************************************************************/

struct trion_dev_s;

struct trion_ops_s
{
  CODE void(*power)(FAR struct trion_dev_s *dev, FAR bool power);
  CODE void(*reset)(FAR struct trion_dev_s *dev, FAR bool reset);
  CODE void(*select)(FAR struct trion_dev_s *dev, FAR bool select);
  CODE bool(*get_done)(FAR struct trion_dev_s *dev);
  CODE bool(*get_stat)(FAR struct trion_dev_s *dev);
};

struct trion_dev_s
{
  FAR const struct trion_ops_s *ops;
  FAR struct spi_dev_s *spi;
  bool is_open;
  bool in_progress;
};

int trion_register(FAR const char *path, FAR struct trion_dev_s *dev);

#endif /* CONFIG_SPI && CONFIG_SPI_TRION */

#endif /* __INCLUDE_NUTTX_SPI_TRION_H */
