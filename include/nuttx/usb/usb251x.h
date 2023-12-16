/****************************************************************************
 * include/nuttx/usb/usb251x.h
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

#ifndef __INCLUDE_NUTTX_USB_USB251X_H
#define __INCLUDE_NUTTX_USB_USB251X_H

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

enum usb251x_reg_address_e
{
  USB251X_VIDL_REG   = 0x00,
  USB251X_VIDM_REG,
  USB251X_PIDL_REG,
  USB251X_PIDM_REG,
  USB251X_DIDL_REG,
  USB251X_DIDM_REG,
  USB251X_CFG1_REG,
  USB251X_CFG2_REG,
  USB251X_CFG3_REG,
  USB251X_PRTR12_REG = 0xfb,
  USB251X_PRTR34_REG = 0xfc,
  USB251X_PRTR56_REG = 0xfd,
  USB251X_PRTR7_REG  = 0xfe,
  USB251X_STCD_REG   = 0xff
};

/* USB hub port number */
enum usb251x_ports_e
{
    USB251X_UPS_PORT,
    USB251X_DN1_PORT,
    USB251X_DN2_PORT,
    USB251X_DN3_PORT,
    USB251X_DN4_PORT,
    USB251X_DN5_PORT,
    USB251X_DN6_PORT,
    USB251X_DN7_PORT,
};

/* Status/Command - 0xff */

enum usb251x_stcd_e
{
  STCD_DETACH  = (0 << 0),
  STCD_ATTACH  = (1 << 0),
  STCD_RESET   = (1 << 1),
};

struct usb251x_config_s
{
  const char *mfr_str;
  const char *prd_str;
  const char *ser_str;
  int portnum;
  int (*portmap)(uint8_t);
  uint8_t config[256];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: usb251x_register
 *
 * Description:
 *   Register the USB251X character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/usbhub0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             USB251X
 *   addr    - The I2C address of the USB251X.
 *             The I2C address of the USB251X is 0x58.
 *   config  - Pointer to USB251X configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int usb251x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, FAR struct usb251x_config_s *config);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_USB251X_H */
