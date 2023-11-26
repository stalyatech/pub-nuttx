/****************************************************************************
 * include/nuttx/usb/usb2517.h
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

#ifndef __INCLUDE_NUTTX_USB_USB2517_H
#define __INCLUDE_NUTTX_USB_USB2517_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

#define USBCIOC_READ_VID      _USBCIOC(0x0001)  /* Arg: uint8_t* pointer */
#define USBCIOC_READ_PID      _USBCIOC(0x0002)  /* Arg: uint8_t* pointer */
#define USBCIOC_READ_DID      _USBCIOC(0x0003)  /* Arg: uint8_t* pointer */
#define USBCIOC_CONFIG        _USBCIOC(0x0004)  /* Arg: uint8_t* pointer */
#define USBCIOC_ATTACH        _USBCIOC(0x0005)  /* Arg: None */
#define USBCIOC_DETACH        _USBCIOC(0x0006)  /* Arg: None */
#define USBCIOC_RESET         _USBCIOC(0x0007)  /* Arg: None */

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum usb2517_reg_address_e
{
  USB2517_VIDL_REG   = 0x00,
  USB2517_VIDM_REG,
  USB2517_PIDL_REG,
  USB2517_PIDM_REG,
  USB2517_DIDL_REG,
  USB2517_DIDM_REG,
  USB2517_CFG1_REG,
  USB2517_CFG2_REG,
  USB2517_CFG3_REG,
  USB2517_PRTR12_REG = 0xfb,
  USB2517_PRTR34_REG = 0xfc,
  USB2517_PRTR56_REG = 0xfd,
  USB2517_PRTR7_REG  = 0xfe,
  USB2517_STCD_REG   = 0xff
};

/* USB hub port number */
enum usb2517_ports_e
{
    USB2517_UPS_PORT,
    USB2517_XBEE_PORT,
    USB2517_RSV_PORT,
    USB2517_MCU_PORT,
    USB2517_F9H1_PORT,
    USB2517_F9P_PORT,
    USB2517_M2M_PORT,
    USB2517_F9H2_PORT,
};

/* Status/Command - 0xff */

enum usb2517_stcd_e
{
  STCD_DETACH  = (0 << 0),
  STCD_ATTACH  = (1 << 0),
  STCD_RESET   = (1 << 1),
};

struct usb2517_config_s
{
  const char *mfr_str;
  const char *prd_str;
  const char *ser_str;
  uint8_t config[256];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: usb2517_register
 *
 * Description:
 *   Register the USB2517 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/usbhub0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             USB2517
 *   addr    - The I2C address of the USB2517.
 *             The I2C address of the USB2517 is 0x58.
 *   config  - Pointer to USB2517 configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int usb2517_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, FAR struct usb2517_config_s *config);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_USB2517_H */
