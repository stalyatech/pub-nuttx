/****************************************************************************
 * boards/arm/stm32f7/ardusimple-sbc/src/ardusimple-sbc.h
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

#ifndef __BOARDS_ARM_STM32F7_ARDUSIMPLE_SBC_SRC_ARDUSIMPLE_SBC_H
#define __BOARDS_ARM_STM32F7_ARDUSIMPLE_SBC_SRC_ARDUSIMPLE_SBC_H

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

/* ArduSimple-SBC GPIOs */

/* The ArduSimple-SBC board has 3 LEDs.
 *
 * All of them can be controlled by software.
 *
 */

#define GPIO_LD1          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN1)
#define GPIO_LD2          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN1)
#define GPIO_LD3          (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN7)

#define LED_DRIVER_PATH   "/dev/userleds"

/* BUTTONS
 *
*/

/* SPI3 CS
 *
 * PD7  SPI3 CS
 */

#define GPIO_SPI3_CS     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN7)

/* USB OTG FS
 *
 * PA10  VBUS sensing
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN10)

/* USB hub power control
 *
 * PB10  USBHUB_PWRON
 */

#define GPIO_USB251X_PWR  (GPIO_OUTPUT | GPIO_PUSHPULL  | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN10)
#define GPIO_USB251X_NRST (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN9)

#define USB251X_I2CBUS    3
#define USB251X_I2CADDR   0x2C


/* GNSS Sub system power control
 *
 * PD4  GNSS_PWRON
 */

#define GPIO_GNSS_PWRON   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN4)
#define GPIO_GNSS_RTKIN   (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | GPIO_PORTE | GPIO_PIN2)
#define GPIO_GNSS_GEOIN   (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | GPIO_PORTE | GPIO_PIN3)

/* XBee Sockets power control
 *
 * PE9  XBA_PWRON
 * PB0  XBB_PWRON
 */

#define GPIO_XBA_PWRON    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN9)
#define GPIO_XBB_PWRON    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN0)
#define GPIO_XBA_RTKSEL   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN3)

/* Ethernet reset/irq control
 *
 * PB14 ETH_NRST
 * PB15 ETH_IRQ
 * 
 */

#define GPIO_ETH_NRST     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN14)
#define GPIO_ETH_IRQ      (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN15)

/* CAN Silent
 *
 * PD10  CAN SIL
 */

#define GPIO_CAN_SIL      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN10)

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     5   /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    11  /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1   /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_QERR         (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN15)

#define GPIO_DIN1         (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN4)
#define GPIO_DIN2         (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN5)
#define GPIO_DIN3         (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN6)

#define GPIO_DOUT1        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN15)
#define GPIO_DOUT2        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN14)
#define GPIO_DOUT3        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN13)
#define GPIO_DOUT4        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN12)

/* IMU Sensor Interrupt
 *
 * PD11  AGM_INT
 */

#define GPIO_BNO085_INT   (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | GPIO_PORTD | GPIO_PIN11)
#define BNO085_IRQ        (STM32_IRQ_EXTI1510)

#define BOARD_IMU_GPIO_INT  GPIO_BNO085_INT
#define BOARD_IMU_IRQ       BNO085_IRQ


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
 *   USB-related GPIO pins for the ardusimple-sbc board.
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
 *   ardusimple-sbc board.
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
 * Name: stm32_usbhub_initialize
 ****************************************************************************/

#ifdef CONFIG_USB2517
int stm32_usbhub_initialize(int bus);
#endif

/****************************************************************************
 * Name: stm32_bno085_initialize
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BNO085
int stm32_bno085_initialize(int bus);
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

#endif /* __BOARDS_ARM_STM32F7_ARDUSIMPLE_SBC_SRC_ARDUSIMPLE_SBC_H */
