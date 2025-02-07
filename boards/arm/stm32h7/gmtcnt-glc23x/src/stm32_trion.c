/****************************************************************************
 * boards/arm/stm32h7/gmtcnt-glc23x/src/stm32_trion.c
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
#include <nuttx/nuttx.h>

#include <sys/types.h>
#include <errno.h>

#include <arch/board/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/trion.h>
#include <nuttx/spi/spi.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "gmtcnt-glc23x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_TRION_PORT	(4)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void trion_power		(struct trion_dev_s *dev, bool power);
static void trion_reset		(struct trion_dev_s *dev, bool reset);
static void trion_select	(struct trion_dev_s *dev, bool select);
static bool trion_get_done(struct trion_dev_s *dev);
static bool trion_get_stat(struct trion_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct trion_ops_s trion_ops =
{
  .power 		= trion_power,
  .reset 		= trion_reset,
  .select 	= trion_select,
  .get_done = trion_get_done,
  .get_stat = trion_get_stat,
};

struct stm32_trion_dev_s
{
  struct trion_dev_s base;
  uint32_t gpio_cdone;
  uint32_t gpio_crst;
  uint32_t gpio_csn;
  uint32_t gpio_stat;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trion_power
 *
 * Description:
 *  Power Trion T20 FPGA
 *
 * Input Parameters:
 *  power - true to power on, false to power off
 *
 ****************************************************************************/

static void trion_power(struct trion_dev_s *dev, bool power)
{
	UNUSED(dev);

	if (power)
		{
			/* Power on the Trion */

			stm32_gpiowrite(GPIO_FPGA_PWRCO, true);
			up_udelay(1);
			stm32_gpiowrite(GPIO_FPGA_PWRIO, true);
			up_udelay(1);
		}
	else
		{
			/* Power off the Trion */

			stm32_gpiowrite(GPIO_FPGA_PWRIO, false);
			up_udelay(1);
			stm32_gpiowrite(GPIO_FPGA_PWRCO, false);
			up_udelay(1);
		}
}

/****************************************************************************
 * Name: trion_reset
 *
 * Description:
 * 	Reset Trion T20 FPGA
 *
 * Input Parameters:
 *  dev - address of the Trion driver structure
 *  reset - true to reset, false to release reset (inverse logic, active low)
 *
 ****************************************************************************/
static void trion_reset(struct trion_dev_s *dev, bool reset)
{
  struct stm32_trion_dev_s *priv
      = container_of (dev, struct stm32_trion_dev_s, base);

	/* Reset the Trion */

  stm32_gpiowrite(priv->gpio_crst, !reset);
}

/****************************************************************************
 * Name: trion_select
 *
 * Description:
 *  Select Trion T20 FPGA
 *
 * Input Parameters:
 *  dev - address of the Trion driver structure
 *  select - true to select, false to deselect (inverse logic, active low)
 *
 ****************************************************************************/

static void trion_select(struct trion_dev_s *dev, bool select)
{
  struct stm32_trion_dev_s *priv
      = container_of (dev, struct stm32_trion_dev_s, base);

	/* Select the Trion */

  stm32_gpiowrite(priv->gpio_csn, !select);
}

/****************************************************************************
 * Name: trion_get_done
 *
 * Description:
 *  Get Trion FPGA status via CDONE pin. Important to know if the FPGA is
 *  programmed and ready to use.
 *
 * Returned Value:
 *  true if the FPGA is programmed and ready to use, false otherwise.
 *
 ****************************************************************************/

static bool trion_get_done(struct trion_dev_s *dev)
{
  struct stm32_trion_dev_s *priv
      = container_of(dev, struct stm32_trion_dev_s, base);

	/* Read the CDONE pin */

  return stm32_gpioread(priv->gpio_cdone);
}

/****************************************************************************
 * Name: trion_get_stat
 *
 * Description:
 *  Get Trion FPGA status via NSTATUS pin. Important to know if the FPGA is
 *  programmed and ready to use.
 *
 * Returned Value:
 *  true if the FPGA is programmed and ready to use, false otherwise.
 *
 ****************************************************************************/

static bool trion_get_stat(struct trion_dev_s *dev)
{
  struct stm32_trion_dev_s *priv
      = container_of(dev, struct stm32_trion_dev_s, base);

	/* Read the NSTATUS pin */

  return stm32_gpioread(priv->gpio_stat);
}

/****************************************************************************
 * Name: stm32_trion_initialize
 *
 * Description:
 *  Initialize ICE40 FPGA GPIOs and SPI.
 *
 * Input Parameters:
 *
 *
 ****************************************************************************/

struct trion_dev_s *stm32_trion_initialize(const uint32_t gpio_cdone,
						 const uint32_t gpio_crst,
						 const uint32_t gpio_csn,
																					 const uint32_t gpio_stat,
						 const uint16_t spi_port)
{
  struct stm32_trion_dev_s *trion_ptr;

	/* Allocate the driver structure */

  trion_ptr = kmm_malloc(sizeof(struct stm32_trion_dev_s));
  if (trion_ptr == NULL)
    {
      spierr("ERROR: Failed to allocate memory for ICE40 driver\n");
      return NULL;
    }

  memset(trion_ptr, 0, sizeof (struct stm32_trion_dev_s));

  trion_ptr->base.ops = &trion_ops;

  /* Configure GPIO pins */

  stm32_configgpio(gpio_cdone);
  stm32_configgpio(gpio_crst);
  stm32_configgpio(gpio_csn);
  stm32_configgpio(gpio_stat);

	/* Initialize the driver structure */

  trion_ptr->gpio_cdone = gpio_cdone;
  trion_ptr->gpio_crst	= gpio_crst;
  trion_ptr->gpio_csn		= gpio_csn;
  trion_ptr->gpio_stat	= gpio_stat;

  /* Configure SPI */

  trion_ptr->base.spi = stm32_spibus_initialize(spi_port);
  if (trion_ptr->base.spi == NULL)
    {
      spierr("ERROR: Failed to initialize SPI port %d\n",
              spi_port);
      return NULL;
    }

  return &trion_ptr->base;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_trion_initialize
 *
 * Initialize Trion FPGA GPIOs and SPI interface.  Then register the Trion
 * FPGA device at "/dev/trion0".
 *
 * Returns:
 *  OK on success, a negated errno on failure.
 *
 ****************************************************************************/
int board_trion_initialize(void)
{
  struct trion_dev_s *trion;

  /* Initialize Trion FPGA GPIOs and SPI interface */

  trion = stm32_trion_initialize(GPIO_FPGA_CDONE,
                                 GPIO_FPGA_CRST,
                                 GPIO_FPGA_CSN,
																 GPIO_FPGA_STAT,
                                 SPI_TRION_PORT);
  if (trion <= 0)
    {
      spierr("ERROR: Failed to initialize Efinix/Trion driver\n");
      return -ENODEV;
    }

  /* Register the Trion FPGA device at "/dev/trion-0" */

  int ret = trion_register("/dev/trion0", trion);
  if (ret < 0)
    {
      spierr("ERROR: Failed to register Efinix/Trion driver: %d\n", ret);
      return ret;
    }

  /* Release the CRESET_N */

	//stm32_configgpio(GPIO_FPGA_CRSTREL);

  return OK;
}
