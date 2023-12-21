############################################################################
# boards/arm/stm32h7/stalya-fmu/src/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = stm32_boot.c stm32_bringup.c

ifeq ($(CONFIG_ADC),y)
CSRCS += stm32_adc.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
else
CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += stm32_buttons.c
endif

ifeq ($(CONFIG_STM32_ROMFS),y)
CSRCS += stm32_romfs_initialize.c
endif

ifeq ($(CONFIG_STM32H7_SPI),y)
CSRCS += stm32_spi.c
endif

ifeq ($(CONFIG_STM32H7_OTGFS),y)
CSRCS += stm32_usb.c
endif

ifeq ($(CONFIG_BOARDCTL_UNIQUEID),y)
CSRCS += stm32_uid.c
endif

ifeq ($(CONFIG_SENSORS_ICM20689),y)
CSRCS += stm32_icm20689.c
endif

ifeq ($(CONFIG_SENSORS_LIS3MDL),y)
CSRCS += stm32_lis3mdl.c
endif

ifeq ($(CONFIG_SENSORS_MS5611),y)
CSRCS += stm32_ms5611.c
endif

ifeq ($(CONFIG_IOEXPANDER_PCA9557),y)
CSRCS += stm32_pca9557.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += stm32_appinitialize.c
ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += stm32_reset.c
endif
ifeq ($(CONFIG_BOARDCTL_IOCTL),y)
CSRCS += stm32_ioctl.c
endif
endif

ifeq ($(CONFIG_STM32H7_PROGMEM),y)
CSRCS += stm32_progmem.c
endif

ifeq ($(CONFIG_USB251X),y)
CSRCS += stm32_usbhub.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += stm32_gpio.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += stm32_pwm.c
endif

ifeq ($(CONFIG_BOARDCTL_BOOT_IMAGE),y)
CSRCS += stm32_boot_image.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += stm32_usbmsc.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS += stm32_composite.c
endif

ifeq ($(CONFIG_MMCSD),y)
CSRCS += stm32_mmcsd.c
endif

ifeq ($(CONFIG_FAT_DMAMEMORY),y)
CSRCS += stm32_dma_alloc.c
endif

ifeq ($(CONFIG_MTD_RAMTRON),y)
CSRCS += stm32_ramtron.c
endif

include $(TOPDIR)/boards/Board.mk
