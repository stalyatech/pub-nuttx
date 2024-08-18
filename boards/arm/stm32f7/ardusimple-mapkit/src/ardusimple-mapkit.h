/****************************************************************************
 * boards/arm/stm32f7/ardusimple-mapkit/src/ardusimple-mapkit.h
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

#ifndef __BOARDS_ARM_STM32F7_ARDUSIMPLE_MAPKIT_SRC_ARDUSIMPLE_MAPKIT_H
#define __BOARDS_ARM_STM32F7_ARDUSIMPLE_MAPKIT_SRC_ARDUSIMPLE_MAPKIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PROC            1
#define HAVE_USBDEV          1
#define HAVE_USBHOST         1
#define HAVE_USBMONITOR      1
#define HAVE_MTDCONFIG       1
#define HAVE_PROGMEM_CHARDEV 1
#define HAVE_HCIUART         1

/* Can't support HCI UART features if BCM4343X Bluetooth is not enabled */

#ifndef CONFIG_BLUETOOTH_BCM4343X
#  undef HAVE_HCIUART
#endif

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32F7_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#endif

/* Can't support USB device if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

#if !defined(CONFIG_STM32F7_PROGMEM) || !defined(CONFIG_MTD_PROGMEM)
#  undef HAVE_PROGMEM_CHARDEV
#endif

/* This is the on-chip progmem memory driver minor number */

#define PROGMEM_MTD_MINOR 0


/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

#if defined(CONFIG_STM32F7_SDMMC1) || defined(CONFIG_STM32F7_SDMMC2)
#  define HAVE_SDIO
#endif

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_SDIO
#endif

#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO
#  if defined(CONFIG_STM32F7_SDMMC1)
#    define GPIO_SDMMC1_NCD (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTD | GPIO_PIN3)
#  endif

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && (CONFIG_NSH_MMCSDSLOTNO != 0)
#    warning "Only one MMC/SD slot, slot 0"
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif
#endif

/* ArduSimple-MapKit GPIOs */

/* The ArduSimple-MapKit board has 6 LEDs.
 *
 * All of them can be controlled by software.
 *
 */

#define GPIO_LD1          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN2)
#define GPIO_LD2          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN3)
#define GPIO_LD3          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN5)
#define GPIO_LD4          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN6)
#define GPIO_LD5          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN7)
#define GPIO_LD6          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN0)

#define LEDS_DEVPATH      "/dev/userleds"

/* BUTTONS
 *
 */

#define GPIO_BTN_USER     (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | GPIO_PORTA | GPIO_PIN4)

#define BUTTONS_DEVPATH   "/dev/buttons"

/* SPI3 CS
 *
 * PD7  SPI3 CS
 */

#define GPIO_SPI3_CS      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN7)

/* USB OTG FS
 *
 * PA9  VBUS sensing
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN9)

/* USB hub power control
 *
 * PA10  USBHUB_PWRON
 * PE12  USBHUB_NRST
 */

#define GPIO_USB251X_PWR  (GPIO_OUTPUT | GPIO_PUSHPULL  | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN10)
#define GPIO_USB251X_NRST (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN12)

#define USB251X_I2CBUS    2
#define USB251X_I2CADDR   0x2C

/* Peripheral power control
 *
 * PD10  PER_PWRON
 */

#define GPIO_PER_PWRON    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN10)

/* Radio Interface
 *
 * PE2   WL_REG_ON    (OUT)
 * PD11  WL_HOST_WAKE (IN)
 *
 * PD0   BT_REG_ON    (OUT)
 * PE5   BL_DEV_WAKE  (OUT)
 * PE4   BL_HOST_WAKE (IN)
 */

#define BT_UART_DEVPATH   "/dev/ttyS1"

#define GPIO_WL_REG_ON    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN2)
#define GPIO_WL_HOST_WAKE (GPIO_INPUT | GPIO_PULLDOWN | GPIO_SPEED_50MHz | \
                           GPIO_EXTI | GPIO_PORTE | GPIO_PIN3)

#define GPIO_BT_REG_ON    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN0)
#define GPIO_BL_DEV_WAKE  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN5)
#define GPIO_BL_HOST_WAKE (GPIO_INPUT | GPIO_PULLDOWN | GPIO_SPEED_50MHz | \
                           GPIO_EXTI | GPIO_PORTE | GPIO_PIN4)

/* WLAN chip */

#define SDIO_WLAN0_SLOTNO 0 /* MapKit has only one sdio device */
#define SDIO_WLAN0_MINOR  0 /* Register "wlan0" device */

/* Battery charger control pins
 *
 * PE9  CHG_ENB   (OUT)
 * PB0  CHG_INT   (IN)
 * PB0  CHG_STA   (IN)
 */

#define CHARGER_DEVPATH   "/dev/batt0"

#define GPIO_CHG_ENB      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN15)

#define GPIO_CHG_STA      (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | GPIO_PORTE | GPIO_PIN13)
#define GPIO_CHG_INT      (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
                           GPIO_EXTI | GPIO_PORTE | GPIO_PIN14)

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1   /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    11  /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    3   /* Amount of GPIO Input w/ Interruption pins */

/* IMU Sensor */

#define BNO055_I2C_BUS    (4)
#define BNO055_I2C_ADDR   (0x28)
#define BNO055_I2C_FREQ   (400000)

#define BNO085_I2C_BUS    (4)
#define BNO085_I2C_ADDR   (0x4a)
#define BNO085_I2C_FREQ   (400000)

/*
 * IMU Sensor Interrupt
 *
 * PD14  IMU_INT
 */

#define GPIO_BNO085_INT   (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | GPIO_PORTD | GPIO_PIN14)
#define BNO085_IRQ        (STM32_IRQ_EXTI1510)

#define BOARD_IMU_GPIO_INT  GPIO_BNO085_INT
#define BOARD_IMU_IRQ       BNO085_IRQ

/* BQ2429X Battery charger */

#define BQ2429X_I2C_BUS   (2)
#define BQ2429X_I2C_ADDR  (0x6b)
#define BQ2429X_I2C_FREQ  (100000)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-144 board.
 *
 ****************************************************************************/

#if defined(CONFIG_SPI)
void stm32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_dma_alloc_init
 *
 * Description:
 *   Called to create a FAT DMA allocator
 *
 * Returned Value:
 *   0 on success or -ENOMEM
 *
 ****************************************************************************/

#if defined (CONFIG_FAT_DMAMEMORY)
int stm32_dma_alloc_init(void);
#endif

#ifdef CONFIG_MTD
/****************************************************************************
 * Name: stm32_mtd_initialize
 *
 * Description:
 *   Initialize MTD drivers.
 *
 ****************************************************************************/
#ifdef HAVE_PROGMEM_CHARDEV
int stm32_progmem_init(void);
#endif  /* HAVE_PROGMEM_CHARDEV */

/****************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Initialize and register the W25 FLASH file system.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_W25
int stm32_w25initialize(int minor);
#endif /* CONFIG_MTD_W25 */
#endif /* CONFIG_MTD */

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Called at application startup time to initialize the SCMMC
 *   functionality.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD
int stm32_sdio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the ardusimple-mapkit board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_OTGFS
void stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_netinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the
 *   ardusimple-mapkit board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_ETHMAC
void stm32_netinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_bbsram_int
 ****************************************************************************/

#ifdef CONFIG_STM32F7_BBSRAM
int stm32_bbsram_int(void);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 ****************************************************************************/

#ifdef CONFIG_STM32F7_CAN_CHARDRIVER
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_cansock_setup
 ****************************************************************************/

#ifdef CONFIG_STM32F7_CAN_SOCKET
int stm32_cansock_setup(void);
#endif

/****************************************************************************
 * Name: stm32_gpio_initialize
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int stm32_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_bno055_initialize
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BNO055
int stm32_bno055_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_bno085_initialize
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BNO085
int stm32_bno085_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_bq2429x_initialize
 ****************************************************************************/

#ifdef CONFIG_BQ2429X
int stm32_bq2429x_initialize(const char *devname);
#endif

/****************************************************************************
 * Name: stm32_wlan_initialize
 *
 * Description:
 *   Initialize SDIO-based BCM WLAN support
 *
 ****************************************************************************/
#ifdef CONFIG_IEEE80211_INFINEON_CYW43439
int stm32_wlan_initialize(void);
#endif

/****************************************************************************
 * Name: board_gps_initialize
 ****************************************************************************/

#ifdef CONFIG_SENSORS_GPS
int board_gps_initialize(void);
#endif

/****************************************************************************
 * Name: board_usb251x_initialize
 *
 * Description:
 *   Initialize USB251X hub driver.
 *
 ****************************************************************************/
#ifdef CONFIG_USB251X
int board_usb251x_initialize(int bus);
#endif

/****************************************************************************
 * Name: hciuart_dev_initialize
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   Bluetooth HCI UART driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_HCIUART
int hciuart_dev_initialize(void);
#endif

/****************************************************************************
 * Name: board_spiflash_init
 *
 * Description:
 *   Initialize the SPI Flash and register the MTD device.
 *
 * Input Parameters:
 *   mtd - The MTD device that supports the FLASH interface.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/
#ifdef CONFIG_MTD
struct mtd_dev_s;
int board_spiflash_init(struct mtd_dev_s *mtd, const char *path, const char *mnt_pt);
#endif

/****************************************************************************
 * Name:  board_composite_initialize
 *
 * Description:
 *   Initialize the USB device <usbdev> on the specified USB device port.
 *
 * Input Parameters:
 *   port- The USB device port.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#if !defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)
int board_composite_initialize(int port);
#endif

/****************************************************************************
 * Name:  board_composite_connect
 *
 * Description:
 *   Connect the USB composite device on the specified USB device port using
 *   the specified configuration.  The interpretation of the configid is
 *   board specific.
 *
 * Input Parameters:
 *   port     - The USB device port.
 *   configid - The USB composite configuration
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  NULL is returned on
 *   any failure.
 *
 ****************************************************************************/

#if !defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)
FAR void *board_composite_connect(int port, int configid);
#endif

#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM_STM32F7_ARDUSIMPLE_MAPKIT_SRC_ARDUSIMPLE_MAPKIT_H */
