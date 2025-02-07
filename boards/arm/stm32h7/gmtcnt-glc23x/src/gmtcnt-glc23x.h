/****************************************************************************
 * boards/arm/stm32h7/gmtcnt-glc23x/src/gmtcnt-glc23x.h
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

#ifndef __BOARDS_ARM_STM32H7_GMTCNT_GLC23X_SRC_GMTCNT_GLC23X_H
#define __BOARDS_ARM_STM32H7_GMTCNT_GLC23X_SRC_GMTCNT_GLC23X_H

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
#define HAVE_NETMONITOR 		 1

/* Can't support HCI UART features if BCM4343X Bluetooth is not enabled */

#ifndef CONFIG_BLUETOOTH_BCM4343X
#  undef HAVE_HCIUART
#endif

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

/* NSH Network monitor  */

#if !defined(CONFIG_NET) || !defined(CONFIG_STM32H7_ETHMAC)
#  undef HAVE_NETMONITOR
#endif

#if !defined(CONFIG_NETINIT_THREAD) || !defined(CONFIG_ARCH_PHY_INTERRUPT) || \
    !defined(CONFIG_NETDEV_PHY_IOCTL) || !defined(CONFIG_NET_UDP)
#  undef HAVE_NETMONITOR
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

#if defined(CONFIG_STM32H7_SDMMC1) || defined(CONFIG_STM32H7_SDMMC2)
#  define HAVE_SDIO
#endif

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_SDIO
#endif

#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO
#  if defined(CONFIG_STM32H7_SDMMC1)
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

/* LED(s) */

#define GPIO_LD1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN9)
#define GPIO_LD2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN10)
#define GPIO_LD3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN11)
#define GPIO_LD4       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN12)

#define GPIO_LED_CAN 		GPIO_LD1
#define GPIO_LED_USB  	GPIO_LD2
#define GPIO_LED_485   	GPIO_LD3
#define GPIO_LED_232   	GPIO_LD4

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTON(s) */

#define GPIO_BTN_USER  	(GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
												 GPIO_EXTI | GPIO_PORTA | GPIO_PIN3)

#define BUTTONS_DEVPATH	"/dev/buttons"

/* USB OTG FS
 *
 * Pxx  OTG_FS_VBUS VBUS sensing
 * PD12 OTG_FS_PowerSwitchOn
 * PD11 OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
                           GPIO_PORTA | GPIO_PIN0)

#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
													 GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN12)
#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
                           GPIO_EXTI | GPIO_PORTD | GPIO_PIN11)
#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
                           GPIO_PORTD | GPIO_PIN11)
#endif

/* Ethernet dedicated pins */

#define GPIO_ETH_PWR		(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN7)

#define GPIO_ETH_RST		(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN15)

#define GPIO_ETH_IRQ   	(GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
                         GPIO_EXTI | GPIO_PORTE | GPIO_PIN13)

/* FPGA dedicated pins */

#define GPIO_FPGA_CRST 	(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN3)

#define GPIO_FPGA_CRSTREL (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
				 GPIO_PORTE | GPIO_PIN3)

#define GPIO_FPGA_CONF  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN13)

#define GPIO_FPGA_CDONE	(GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
                         GPIO_PORTB | GPIO_PIN8)

#define GPIO_FPGA_STAT  (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
                         GPIO_PORTE | GPIO_PIN1)

#define GPIO_FPGA_CSN   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN4)

#define GPIO_FPGA_PWRCO	(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN4)

#define GPIO_FPGA_PWRIO	(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN2)

/* External ADC dedicated pins */

#define GPIO_ADC_PWR		(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN1)

/* Absolute Encoder dedicated pins */

#define GPIO_ENC_PWR		(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN9)

#define GPIO_ENC_DIR		(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN14)

/* Expansion bus dedicated pins */

#define GPIO_EXP_ENB		(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN2)

#define GPIO_EXP_SIL		(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN3)

/* LED controller dedicated pins */

#define GPIO_LDC_ENB		(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN3)

#define GPIO_LDC_RST		(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN9)

/* ADC pins */

#define GPIO_ADC3_INP0	(GPIO_ANALOG | GPIO_PORTC | GPIO_PIN2)   	/* VSENS */

/* SPI FLASH chip select */

#define GPIO_W25_CSN   	(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN15)

/* LED I2C configuration */

#define LED_I2CBUS     	(3)
#define LED_I2CADDR    	(0x00)

/* PWM */

#define GLC23X_PWMTIMER	(3)

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT   	0 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    0 /* Amount of GPIO Input w/ Interruption pins */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_early_bringup
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

int stm32_early_bringup(void);

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
 *   Called to configure SPI chip select GPIO pins for the GLC23X PLC.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI
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

#ifdef CONFIG_STM32H7_OTGFS
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

#ifdef CONFIG_STM32H7_ETHMAC
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

#ifdef CONFIG_STM32H7_BBSRAM
int stm32_bbsram_int(void);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 ****************************************************************************/

#ifdef CONFIG_STM32H7_CAN_CHARDRIVER
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_cansock_setup
 ****************************************************************************/

#ifdef CONFIG_STM32H7_CAN_SOCKET
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
 * Name: board_gnss_initialize
 ****************************************************************************/

#ifdef CONFIG_SENSORS_GNSS
int board_gnss_initialize(void);
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
int board_spiflash_init(struct mtd_dev_s *mtd, const char *path,
                        const char *mnt_pt);
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

/****************************************************************************
 * Function: board_net_initialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c. If STM32H7_NETHERNET
 *   greater than one, then board specific logic will have to supply a
 *   version of arm_netinitialize() that calls stm32_ethinitialize() with
 *   the appropriate interface number.
 *
 * Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/
#ifdef CONFIG_NETDEV_LATEINIT
void board_net_initialize(void);
#endif

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
#if defined(CONFIG_SPI) && defined(CONFIG_SPI_TRION)
int board_trion_initialize(void);
#endif

#endif /* __BOARDS_ARM_STM32H7_GMTCNT_GLC23X_SRC_GMTCNT_GLC23X_H */
