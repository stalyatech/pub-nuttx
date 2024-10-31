/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/src/stm32_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "stalya-fmu.h"
#include <arch/board/board.h>

#ifdef CONFIG_STM32H7_SPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Stalya-FMU board.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
  /* NOTE: Clocking for SPI1 and/or SPI3 was already provided in stm32_rcc.c.
   *       Configurations of SPI pins is performed in stm32_spi.c.
   *       Here, we only initialize chip select pins unique to the board
   *       architecture.
   */

  /* All bus shared sensor VS must be defined */

  /* Configure the SPI-based BMI088 sensor selects GPIO */

  /* Accel part */

  stm32_configgpio(GPIO_BMI088A_CSN);
  stm32_gpiowrite(GPIO_BMI088A_CSN, true);

  /* Gyro part */

  stm32_configgpio(GPIO_BMI088G_CSN);
  stm32_gpiowrite(GPIO_BMI088G_CSN, true);

  /* Interrupt GPIO */

  stm32_configgpio(GPIO_BMI088_INT);

  /* Configure the SPI-based ICM20689 sensor select GPIO */

  stm32_configgpio(GPIO_ICM20689_CSN);
  stm32_gpiowrite(GPIO_ICM20689_CSN, true);

  /* Interrupt GPIO */

  stm32_configgpio(GPIO_ICM20689_INT);

  /* Configure the SPI-based LIS3MDL sensor select GPIO */

  stm32_configgpio(GPIO_LIS3MDL_CSN);
  stm32_gpiowrite(GPIO_LIS3MDL_CSN, true);

  /* Configure the SPI-based MS56XX sensor select GPIO */

  stm32_configgpio(GPIO_MS56XX_CSN);
  stm32_gpiowrite(GPIO_MS56XX_CSN, true);

#if defined(CONFIG_MTD_RAMTRON)
  /* Configure the FRAM select GPIO */

  stm32_configgpio(GPIO_FM25V_CSN);
  stm32_gpiowrite(GPIO_FM25V_CSN, true);
#endif
}

/****************************************************************************
 * Name:  stm32_spi1/2/3/4/5select and stm32_spi1/2/3/4/5status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   (including stm32_spibus_initialize()) are provided by common STM32
 *   logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI1
void stm32_spi1select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  switch (devid)
    {
#if defined(CONFIG_SENSORS_BMI088) && defined(CONFIG_BMI088_SPI)
      case SPIDEV_IMU(0):
        spiinfo("BMI088 accel device %s\n",
                selected ? "asserted" : "de-asserted");

        /* Set the GPIO low to select and high to de-select */

        stm32_gpiowrite(GPIO_BMI088A_CSN, !selected);
        break;

      case SPIDEV_IMU(1):
        spiinfo("BMI088 gyro device %s\n",
                selected ? "asserted" : "de-asserted");

        /* Set the GPIO low to select and high to de-select */

        stm32_gpiowrite(GPIO_BMI088G_CSN, !selected);
        break;
#endif /* CONFIG_SENSORS_BMI088 && CONFIG_BMI088_SPI */

#if defined(CONFIG_SENSORS_ICM20689) && defined(CONFIG_ICM20689_SPI)
      case SPIDEV_IMU(2):
        spiinfo("ICM20689 accel/gyro device %s\n",
                selected ? "asserted" : "de-asserted");

        /* Set the GPIO low to select and high to de-select */

        stm32_gpiowrite(GPIO_ICM20689_CSN, !selected);
        break;
#endif /* CONFIG_SENSORS_ICM20689 && CONFIG_ICM20689_SPI */

#if defined(CONFIG_SENSORS_LIS3MDL) && defined(CONFIG_LIS3MDL_SPI)
      case SPIDEV_IMU(3):
        spiinfo("LIS3MDL megneto device %s\n",
                selected ? "asserted" : "de-asserted");

        /* Set the GPIO low to select and high to de-select */

        stm32_gpiowrite(GPIO_LIS3MDL_CSN, !selected);
        break;
#endif /* CONFIG_SENSORS_LIS3MDL && CONFIG_LIS3MDL_SPI */

#if defined(CONFIG_SENSORS_MS56XX) && defined(CONFIG_MS56XX_SPI)
      case SPIDEV_BAROMETER(0):
        spiinfo("MS56XX baro device %s\n",
                selected ? "asserted" : "de-asserted");

        /* Set the GPIO low to select and high to de-select */

        stm32_gpiowrite(GPIO_MS56XX_CSN, !selected);
        break;
#endif /* CONFIG_SENSORS_MS56XX && CONFIG_MS56XX_SPI */
    }
}

uint8_t stm32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32H7_SPI2
void stm32_spi2select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32H7_SPI3
void stm32_spi3select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32H7_SPI4
void stm32_spi4select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
#if defined(CONFIG_MTD_RAMTRON)
  spiinfo("FM25V mtd device %s\n",
          selected ? "asserted" : "de-asserted");

  /* Set the GPIO low to select and high to de-select */

  stm32_gpiowrite(GPIO_FM25V_CSN, !selected);
#endif
}

uint8_t stm32_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32H7_SPI5
void stm32_spi5select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi5status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32H7_SPI6
void stm32_spi6select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi6status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_spi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32H7_SPI1
int stm32_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32H7_SPI2
int stm32_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32H7_SPI3
int stm32_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32H7_SPI4
int stm32_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32H7_SPI5
int stm32_spi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32H7_SPI6
int stm32_spi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#endif /* CONFIG_SPI_CMDDATA */
#endif /* CONFIG_STM32H7_SPI */
