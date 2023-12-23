/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/src/stm32_bringup.c
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

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>

#include <arch/board/board.h>

#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_STM32H7_OTGFS
#  include "stm32_usbhost.h"
#endif

#ifdef CONFIG_STM32H7_FDCAN
#include "stm32_fdcan_sock.h"
#endif

#include "stalya-fmu.h"

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

#ifdef CONFIG_STM32_ROMFS
#  include "stm32_romfs.h"
#endif

#ifdef CONFIG_STM32H7_IWDG
#  include "stm32_wdg.h"
#endif

#ifdef CONFIG_I2C
#  include "stm32_i2c.h"
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#include "stm32_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2ctool(void)
{
#ifdef CONFIG_STM32H7_I2C1
  stm32_i2c_register(1);
#endif
#ifdef CONFIG_STM32H7_I2C2
  stm32_i2c_register(2);
#endif
#ifdef CONFIG_STM32H7_I2C3
  stm32_i2c_register(3);
#endif
#ifdef CONFIG_STM32H7_I2C4
  stm32_i2c_register(4);
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;
  int devno;
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *lower;
#endif
  UNUSED(devno);
  UNUSED(ret);

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  stm32_i2ctool();
#endif /* CONFIG_I2C && CONFIG_SYSTEM_I2CTOOL */

#ifdef CONFIG_IOEXPANDER_PCA9557
  /* Initialize the PCA9557 chip */

  ret = board_pca9557_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize PCA9557 driver: %d\n", ret);
    }
#endif /* CONFIG_IOEXPANDER_PCA9557 */

#ifdef CONFIG_SENSORS_ICM20689
  /* Initialize the ICM20689 motion tracker sensor(s). */

  devno = 0;
  ret = board_icm20689_i2c_initialize(devno, ICM20689_I2CBUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize ICM20689 over I2C driver: %d\n", ret);
    } 
  else
    {
      devno++;
    } 

  ret = board_icm20689_spi_initialize(devno, ICM20689_SPIBUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize ICM20689 over SPI driver: %d\n", ret);
    }
#endif /* CONFIG_SENSORS_ICM20689 */

#ifdef CONFIG_SENSORS_MS5611
  /* Initialize the MS5611 pressure sensor(s). */

  devno = 0;
  ret = board_ms5611_i2c_initialize(devno, MS5611_I2CBUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize MS5611 over I2C driver: %d\n", ret);
    }
  else
    {
      devno++;
    } 

  ret = board_ms5611_spi_initialize(devno, MS5611_SPIBUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize MS5611 over SPI driver: %d\n", ret);
    }
#endif /* CONFIG_SENSORS_MS5611 */

#ifdef CONFIG_SENSORS_LIS3MDL
  /* Initialize the LIS3MDL e-compass sensor(s). */

  devno = 0;
  ret = board_lis3mdl_i2c_initialize(devno, LIS3MDL_I2CBUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize LIS3MDL over I2C driver: %d\n", ret);
    }
  else
    {
      devno++;
    } 

  ret = board_lis3mdl_spi_initialize(devno, LIS3MDL_SPIBUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize LIS3MDL over SPI driver: %d\n", ret);
    }
#endif /* CONFIG_SENSORS_LIS3MDL */

#ifdef CONFIG_USB251X
  /* Register the usb hub driver */
  
  ret = board_usb251x_initialize(USB251X_I2CBUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize USB251X driver: %d\n", ret);
    }
#endif /* CONFIG_USB251X */

#ifdef CONFIG_STM32_ROMFS
  /* Mount the romfs partition */

  ret = stm32_romfs_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to mount romfs at %s: %d\n", 
             CONFIG_STM32_ROMFS_MOUNTPOINT, ret);
    }
#endif /* CONFIG_STM32_ROMFS */

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32 lower-half RTC driver */

  lower = stm32_rtc_lowerhalf();
  if (!lower)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to instantiate the RTC lower-half driver\n");
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind/register the RTC driver: %d\n", ret);
        }
    }
#endif /* HAVE_RTC_DRIVER */

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize button driver: %d\n", ret);
    }
#endif /* CONFIG_INPUT_BUTTONS */

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize user led driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize ADC driver: %d\n", ret);
    }
#endif /* CONFIG_ADC */

#ifdef CONFIG_DEV_GPIO
  /* Register the GPIO driver */

  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, 
            "ERROR: Failed to initialize GPIO driver: %d\n", ret);
    }
#endif /* CONFIG_DEV_GPIO */

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize PWM driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_MTD
#ifdef HAVE_PROGMEM_CHARDEV
  ret = stm32_progmem_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize MTD progmem: %d\n", ret);
    }
#endif /* HAVE_PROGMEM_CHARDEV */

#ifdef CONFIG_MTD_RAMTRON
  ret = board_ramtron_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize FM25VXX driver: %d\n", ret);
    }
#endif /* CONFIG_MTD_RAMTRON */
#endif /* CONFIG_MTD */

#ifdef CONFIG_NETDEV_LATEINIT
  /* Initialize SocketCAN device. */

#ifdef CONFIG_STM32H7_FDCAN1
  stm32_fdcansockinitialize(0);
#endif

#ifdef CONFIG_STM32H7_FDCAN2
  stm32_fdcansockinitialize(1);
#endif
#endif /* CONFIG_NETDEV_LATEINIT */

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize()
   * starts a thread will monitor for USB connection and
   * disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize USB host: %d\n", ret);
    }
#endif /* HAVE_USBHOST */

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif /* HAVE_USBMONITOR */

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE) && \
    !defined(CONFIG_CDCACM_COMPOSITE)
  /* Initialize CDCACM */

  syslog(LOG_INFO, "Initialize CDCACM device\n");

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize USB CDC/ACM device: %d\n", ret);
    }
#endif /* CONFIG_CDCACM & !CONFIG_CDCACM_CONSOLE */

#if defined(CONFIG_RNDIS) && !defined(CONFIG_RNDIS_COMPOSITE)
  uint8_t mac[6];
  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif /* CONFIG_RNDIS && !CONFIG_RNDIS_COMPOSITE */

#if defined(CONFIG_FAT_DMAMEMORY)
  /* Initialize the FAT DMA memory allocator */

  ret = stm32_dma_alloc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize FAT DMA memory allocator: %d\n", ret);
    }
#endif /* CONFIG_FAT_DMAMEMORY */

#ifdef CONFIG_MMCSD
  /* Initialize the SDIO block driver */

  ret = stm32_mmcsd_initialize(CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, 
             "ERROR: Failed to initialize SD slot %d: %d\n",
             CONFIG_NSH_MMCSDMINOR, ret);
    }
#endif /* CONFIG_MMCSD */

#ifdef CONFIG_STM32H7_IWDG
  /* Initialize the watchdog timer */

  stm32_iwdginitialize("/dev/watchdog0", STM32_LSI_FREQUENCY);
#endif /* CONFIG_STM32H7_IWDG */

  return OK;
}
