/****************************************************************************
 * boards/arm/stm32h7/stalya-fmu/src/stalya-fmu.h
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

#ifndef __BOARDS_ARM_STM32H7_STALYA_FMU_SRC_STALYA_FMU_H
#define __BOARDS_ARM_STM32H7_STALYA_FMU_SRC_STALYA_FMU_H

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

#ifndef CONFIG_STM32H7_OTGFS
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

#if !defined(CONFIG_STM32H7_PROGMEM) || !defined(CONFIG_MTD_PROGMEM)
#  undef HAVE_PROGMEM_CHARDEV
#endif

/* This is the on-chip progmem memory driver minor number */

#define PROGMEM_MTD_MINOR 0

/* flash  */
#if defined(CONFIG_MMCSD)
#  define FLASH_BASED_PARAMS
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Check if we can support the RTC driver */

#define HAVE_RTC_DRIVER 1
#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* LED
 *
 * The Stalya-FMU board has numerous LEDs but only three, LD1 a Red LED,
 * LD2 a Green LED and LD3 a Blue LED, that can be controlled by software. 
 * The following definitions assume the default Solder Bridges are installed.
 */

#define GPIO_LD1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN5)
#define GPIO_LD2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN3)
#define GPIO_LD3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN4)

#define GPIO_LED_RED   GPIO_LD1
#define GPIO_LED_GREEN GPIO_LD2
#define GPIO_LED_BLUE  GPIO_LD3

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTONS
 *
 * The Blue pushbutton B1, labeled "User", is connected to GPIO PC13.
 * A high value will be sensed when the button is depressed.
 * Note:
 *    1) That the EXTI is included in the definition to enable an interrupt
 *       on this IO.
 *    2) The following definitions assume the default Solder Bridges are
 *       installed.
 */

#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PG7  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz| \
                           GPIO_OPENDRAIN|GPIO_PORTD|GPIO_PIN10)

/* USB hub reset control */

#define GPIO_USB251X_NRST (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | \
                          GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN15)

#define USB251X_I2CBUS    2
#define USB251X_I2CADDR   0x2C

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     5 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    0 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

/* Example, used free Ports on the board */

#define GPIO_IN1          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN4)    /* DIN1 */
#define GPIO_IN2          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTD | GPIO_PIN7)    /* DIN2 */
#define GPIO_IN3          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN14)   /* PWR_IN1_VALID */
#define GPIO_IN4          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN15)   /* PWR_IN2_VALID */
#define GPIO_IN5          (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTD | GPIO_PIN10)   /* PWR_USB_VALID */

#define GPIO_INT1         (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTD | GPIO_PIN6)    /* SAFETY */

/* IMU Board */

#define GPIO_IMU1_INT0    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN12)
#define GPIO_IMU1_INT2    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN13)

/* ADC pins */

#define GPIO_ADC1_INP4    GPIO_ADC12_INP4_0
#define GPIO_ADC1_INP5    GPIO_ADC12_INP5_0
#define GPIO_ADC1_INP8    GPIO_ADC12_INP8_0
#define GPIO_ADC1_INP9    GPIO_ADC12_INP9_0
#define GPIO_ADC1_INP10   GPIO_ADC123_INP10_0
#define GPIO_ADC1_INP11   GPIO_ADC123_INP11_0
#define GPIO_ADC1_INP18   GPIO_ADC12_INP18_0

/* MMC/SD
 * NCD - PC9
 */

#define GPIO_MMCSD_NCD    (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI |  \
                           GPIO_PORTC | GPIO_PIN9)

/* ICM20689 configuration */

#define ICM20689_I2CBUS     4
#define ICM20689_I2CADDR    0x68

/* LIS3MDL configuration */

#define LIS3MDL_I2CBUS      4
#define LIS3MDL_I2CADDR     0x1c

/* MS56110 configuration */

#define MS56110_I2CBUS      4
#define MS56110_I2CADDR     0x76

/* PCA9635 configuration */

#define PCA9557_I2CBUS      2
#define PCA9557_I2CADDR     0x18
#define PCA9557_I2CFREQ     100000

#define GPIO_PCA9557_NRST   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | \
                             GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN15)

/* PWM */

#define STALYAFMU_PWMTIMER  1

/* SD/TF Card'detected pin */

#if defined(CONFIG_STM32H7_SDMMC1)
#  define HAVE_SDIO
#endif

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_SDIO
#endif

#define GPIO_SDIO_NCD      (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN9)

#define SDIO_SLOTNO        0
#define SDIO_MINOR         0

/****************************************************************************
 * Public Function Prototypes
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
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the
 *   Stalya-FMU board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI
void stm32_spidev_initialize(void);
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
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO-Driver.
 *
 ****************************************************************************/

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
int stm32_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup
 *   USB-related GPIO pins for the STALYA-FMU board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32H7_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_icm20689_initialize
 *
 * Description:
 *   Initialize I2C-based ICM20689.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ICM20689_I2C
int stm32_icm20689_initialize(char *devpath);
#endif

/****************************************************************************
 * Name: stm32_lis3mdl_initialize
 *
 * Description:
 *   Initialize I2C-based LIS3MDL.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LIS3MDL_I2C
int stm32_lis3mdl_initialize(char *devpath);
#endif

/****************************************************************************
 * Name: stm32_ms56110_initialize
 *
 * Description:
 *   Initialize I2C-based MS56110.
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MS56110_I2C
int stm32_ms56110_initialize(char *devpath);
#endif

/****************************************************************************
 * Name: stm32_pca9557_initialize
 *
 * Description:
 *   Initialize I2C-based PCA9557 I/O expander.
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_PCA9557
int stm32_pca9557_initialize(void);
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
 * Name: stm32_mtd_initialize
 *
 * Description:
 *   Initialize MTD drivers.
 *
 ****************************************************************************/
#ifdef CONFIG_MTD

#ifdef HAVE_PROGMEM_CHARDEV
int stm32_progmem_init(void);
#endif  /* HAVE_PROGMEM_CHARDEV */
#endif

/****************************************************************************
 * Name: stm32_usbhub_initialize
 *
 * Description:
 *   Initialize USB hub drivers.
 *
 ****************************************************************************/
#ifdef CONFIG_USB251X
int stm32_usbhub_initialize(int bus);
#endif

/****************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD
int stm32_mmcsd_initialize(int minor);
#endif

#endif /* __BOARDS_ARM_STM32H7_STALYA_FMU_SRC_STALYA_FMU_H */
