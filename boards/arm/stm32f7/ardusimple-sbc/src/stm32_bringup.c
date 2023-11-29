/****************************************************************************
 * boards/arm/stm32f7/ardusimple-sbc/src/stm32_bringup.c
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
#include <nuttx/fs/fs.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/leds/userled.h>

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_STM32_ROMFS
#include "stm32_romfs.h"
#endif
#include "stm32_i2c.h"
#include "ardusimple-sbc.h"


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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;
#ifdef CONFIG_I2C
  int i2c_bus;
  struct i2c_master_s *i2c;
#endif /* CONFIG_I2C */
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *lower;
#endif /* HAVE_RTC_DRIVER */


#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_STM32_ROMFS
  /* Mount the romfs partition */

  ret = stm32_romfs_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount romfs at %s: %d\n",
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
      return -ENOMEM;
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
          return ret;
        }
    }
#endif /* HAVE_RTC_DRIVER */

#ifdef CONFIG_DEV_GPIO
  /* Register the GPIO driver */

  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_DEV_GPIO */

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif /* !CONFIG_ARCH_LEDS && CONFIG_USERLED_LOWER */

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif /* CONFIG_ADC */

#ifdef CONFIG_STM32F7_BBSRAM
  /* Initialize battery-backed RAM */

  stm32_bbsram_int();
#endif /* CONFIG_STM32F7_BBSRAM */

#ifdef CONFIG_FAT_DMAMEMORY
  if (stm32_dma_alloc_init() < 0)
    {
      syslog(LOG_ERR, "DMA alloc FAILED");
    }
#endif /* CONFIG_FAT_DMAMEMORY */

#ifdef CONFIG_MTD_W25
  ret = stm32_w25initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_w25initialize failed: %d\n", ret);
    }
#endif /* CONFIG_MTD_W25 */

#ifdef CONFIG_MMCSD
  /* Initialize the SDIO block driver */

  ret = stm32_sdio_initialize();
  if (ret != OK)
    {
      ferr("ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_MMCSD */

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif /* CONFIG_PWM */

#ifdef CONFIG_STM32F7_CAN_CHARDRIVER
  ret = stm32_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32f7_can_setup failed: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_STM32F7_CAN_CHARDRIVER */

#ifdef CONFIG_STM32F7_CAN_SOCKET
  ret = stm32_cansock_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_cansock_setup failed: %d\n", ret);
    }
#endif /* CONFIG_STM32F7_CAN_SOCKET */

#ifdef CONFIG_I2C
#ifdef CONFIG_USB2517
  /* Register the usb hub driver */
  
#ifdef CONFIG_STM32F7_I2C3
  ret = stm32_usbhub_initialize(3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize USB2517 Driver: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_STM32F7_I2C3 */
#endif /* CONFIG_USB2517 */

#ifdef CONFIG_SENSORS_BNO085
  /* Register the smart sensor driver */

#ifdef CONFIG_STM32F7_I2C3
  ret = stm32_bno085_initialize(3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BNO085 Driver: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_STM32F7_I2C3 */
#endif /* CONFIG_SENSORS_BNO085 */

#ifdef CONFIG_STM32F7_I2C1
  i2c_bus = 1;
  i2c = stm32_i2cbus_initialize(i2c_bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", i2c_bus);
    }
  else
    {
#ifdef CONFIG_SYSTEM_I2CTOOL
      ret = i2c_register(i2c, i2c_bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 i2c_bus, ret);
        }
#endif /* CONFIG_SYSTEM_I2CTOOL */
    }
#endif /* CONFIG_STM32F7_I2C1 */
#endif /* CONFIG_I2C */

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE) && \
    !defined(CONFIG_CDCACM_COMPOSITE)
  /* Initialize CDCACM */

  syslog(LOG_INFO, "Initialize CDCACM device\n");

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize failed: %d\n", ret);
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
#endif

  UNUSED(ret);  /* May not be used */
  return OK;
}
