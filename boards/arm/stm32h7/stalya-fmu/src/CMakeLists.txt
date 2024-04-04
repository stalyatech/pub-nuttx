# ##############################################################################
# boards/arm/stm32h7/stalya-fmu/src/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS stm32_boot.c stm32_bringup.c stm32_platform.c)

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_STM32_ROMFS)
  list(APPEND SRCS stm32_romfs_initialize.c)
endif()

if(CONFIG_STM32H7_SPI)
  list(APPEND SRCS stm32_spi.c)
endif()

if(CONFIG_STM32H7_OTGFS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS stm32_uid.c)
endif()

if(CONFIG_SENSORS_ICM20689)
  list(APPEND SRCS stm32_icm20689.c)
endif()

if(CONFIG_SENSORS_LIS3MDL)
  list(APPEND SRCS stm32_lis3mdl.c)
endif()

if(CONFIG_SENSORS_MS5611)
  list(APPEND SRCS stm32_ms5611.c)
endif()

if(CONFIG_IOEXPANDER_PCA9557)
  list(APPEND SRCS stm32_pca9557.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinitialize.c)
  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS stm32_reset.c)
  endif()
  if(CONFIG_BOARDCTL_IOCTL)
    list(APPEND SRCS stm32_ioctl.c)
  endif()
endif()

if(CONFIG_STM32H7_PROGMEM)
  list(APPEND SRCS stm32_progmem.c)
endif()

if(CONFIG_USB251X)
  list(APPEND SRCS stm32_usbhub.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS stm32_gpio.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_BOARDCTL_BOOT_IMAGE)
  list(APPEND SRCS stm32_boot_image.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS stm32_usbmsc.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS stm32_composite.c)
endif()

if(CONFIG_MMCSD)
  list(APPEND SRCS stm32_mmcsd.c)
endif()

if(CONFIG_FAT_DMAMEMORY)
  list(APPEND SRCS stm32_dma_alloc.c)
endif()

if(CONFIG_MTD_RAMTRON)
  list(APPEND SRCS stm32_ramtron.c)
endif()

if(CONFIG_ETC_ROMFS)
  list(APPEND SRCS etc_romfs.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_STM32_APP_FORMAT_MCUBOOT)
  if(CONFIG_MCUBOOT_BOOTLOADER)
    set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash-mcuboot-loader.ld")
  else()
    set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash-mcuboot-app.ld")
  endif()
else()
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
endif()

if(NOT CONFIG_BUILD_FLAT)
  add_subdirectory(${NUTTX_BOARD_DIR}/kernel)
  set_property(
    GLOBAL PROPERTY LD_SCRIPT_USER ${NUTTX_BOARD_DIR}/scripts/memory.ld
    ${NUTTX_BOARD_DIR}/scripts/user-space.ld)
endif()
