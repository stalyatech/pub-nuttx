/****************************************************************************
 * drivers/sensors/icm20689_base.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_ICM20689_COMMOM_H
#define __INCLUDE_NUTTX_SENSORS_ICM20689_COMMOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/icm20689.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <math.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Specific device id*/

#define ICM20689_DEVID  (0x98)

/* GYRO_CONFIG */

enum gyro_config_bit
{
  /* GYRO_FS_SEL [4:3] */

  GYRO_FS_SEL_250_DPS   = 0, /* 0b00000 */
  GYRO_FS_SEL_500_DPS   = 1, /* 0b01000 */
  GYRO_FS_SEL_1000_DPS  = 2, /* 0b10000 */
  GYRO_FS_SEL_2000_DPS  = 3, /* 0b11000 */

  /* FCHOICE_B [1:0] */

  FCHOICE_B_BYPASS_DLPF  = BIT(1) | BIT(0), /* 0b00 - 3-dB BW: 3281 Noise BW (Hz): 3451.0   8 kHz */
};

/* ACCEL_CONFIG */

enum accel_config_bit
{
  /* ACCEL_FS_SEL [4:3] */

  ACCEL_FS_SEL_2G  = 0, /* 0b00000 */
  ACCEL_FS_SEL_4G  = 1, /* 0b01000 */
  ACCEL_FS_SEL_8G  = 2, /* 0b10000 */
  ACCEL_FS_SEL_16G = 3  /* 0b11000 */
};

/* Sensor indexes */

enum icm20689_idx_e
{
  ACCEL_IDX,
  GYRO_IDX,
  MAX_IDX
};

/* Register addresses */

enum icm20689_regaddr_e
{
  SELF_TEST_G_X = 0x00,
  SELF_TEST_G_Y = 0x01,
  SELF_TEST_G_Z = 0x02,

  SELF_TEST_A_X = 0x0d,
  SELF_TEST_A_Y = 0x0e,
  SELF_TEST_A_Z = 0x0f,

  XG_OFFSETH = 0x13,
  XG_OFFSETL = 0x14,
  YG_OFFSETH = 0x15,
  YG_OFFSETL = 0x16,
  ZG_OFFSETH = 0x17,
  ZG_OFFSETL = 0x18,

  SMPLRT_DIV = 0x19,

	CONFIG = 0x1a,
  CONFIG_EXT_SYNC_SET_SHIFT = 3,
  CONFIG_EXT_SYNC_SET_WIDTH = 3,
  CONFIG_DLPF_CFG_SHIFT = 0,
  CONFIG_DLPF_CFG_WIDTH = 3,

	GYRO_CONFIG = 0x1b,
  GYRO_CONFIG_XG_ST = BIT(7),
  GYRO_CONFIG_YG_ST = BIT(6),
  GYRO_CONFIG_ZG_ST = BIT(5),
  GYRO_CONFIG_FS_SEL_SHIFT = 3,
  GYRO_CONFIG_FS_SEL_WIDTH = 2,

	ACCEL_CONFIG = 0x1c,
  ACCEL_CONFIG_XA_ST = BIT(7),
  ACCEL_CONFIG_YA_ST = BIT(6),
  ACCEL_CONFIG_ZA_ST = BIT(5),
  ACCEL_CONFIG_AFS_SEL_SHIFT = 3,
  ACCEL_CONFIG_AFS_SEL_WIDTH = 2,

	ACCEL_CONFIG2 = 0x1d,
  ACCEL_CONFIG2_FCHOICE_B = BIT(3),
  CONFIG2_A_DLPF_CFG_SHIFT = 0,
  CONFIG2_A_DLPF_CFG_WIDTH = 3,
  LPACCEL_ODR = 0x1e,
  WOM_THR = 0x1f,

	FIFO_EN = 0x23,
  BITS_FIFO_ENABLE_TEMP_OUT = BIT(7),
  BITS_FIFO_ENABLE_GYRO_XOUT = BIT(6),
  BITS_FIFO_ENABLE_GYRO_YOUT = BIT(5),
  BITS_FIFO_ENABLE_GYRO_ZOUT = BIT(4),
  BITS_FIFO_ENABLE_ACCEL = BIT(3),

	INT_PIN_CFG = 0x37,
  INT_PIN_CFG_INT_ACTL = BIT(7),
  INT_PIN_CFG_INT_OPEN = BIT(6),
  INT_PIN_CFG_LATCH_INT_EN = BIT(5),
  INT_PIN_CFG_INT_RD_CLEAR = BIT(4),
  INT_PIN_CFG_FSYNC_INT_LEVEL = BIT(3),
  INT_PIN_CFG_FSYNC_INT_EN = BIT(2),

	INT_ENABLE    = 0x38,
  INT_STATUS    = 0x3a,         /* RO */

  ACCEL_XOUT_H  = 0x3b,         /* RO */
  ACCEL_XOUT_L  = 0x3c,         /* RO */
  ACCEL_YOUT_H  = 0x3d,         /* RO */
  ACCEL_YOUT_L  = 0x3e,         /* RO */
  ACCEL_ZOUT_H  = 0x3f,         /* RO */
  ACCEL_ZOUT_L  = 0x40,         /* RO */
  TEMP_OUT_H    = 0x41,         /* RO */
  TEMP_OUT_L    = 0x42,         /* RO */
  GYRO_XOUT_H   = 0x43,         /* RO */
  GYRO_XOUT_L   = 0x44,         /* RO */
  GYRO_YOUT_H   = 0x45,         /* RO */
  GYRO_YOUT_L   = 0x46,         /* RO */
  GYRO_ZOUT_H   = 0x47,         /* RO */
  GYRO_ZOUT_L   = 0x48,         /* RO */

	SIGNAL_PATH_RESET = 0x68,
  SIGNAL_PATH_RESET_GYRO_RESET = BIT(2),
  SIGNAL_PATH_RESET_ACCEL_RESET = BIT(1),
  SIGNAL_PATH_RESET_TEMP_RESET = BIT(0),
  SIGNAL_PATH_RESET_ALL_RESET = BIT(3) - 1,

	USER_CTRL = 0x6a,
  USER_CTRL_DMP_EN = BIT(7),
  USER_CTRL_FIFO_EN = BIT(6),
  USER_CTRL_I2C_IF_DIS = BIT(4),
  USER_CTRL_DMP_RST = BIT(3),
  USER_CTRL_FIFO_RST = BIT(2),
  USER_CTRL_SIG_COND_RST = BIT(0),

	PWR_MGMT_1 = 0x6b,
  PWR_MGMT_1_DEVICE_RESET = BIT(7),
  PWR_MGMT_1_SLEEP = BIT(6),
  PWR_MGMT_1_CYCLE = BIT(5),
  PWR_MGMT_1_GYRO_STANDBY = BIT(4),
  PWR_MGMT_1_TEMP_DIS = BIT(3),
  PWR_MGMT_1_CLK_SEL_SHIFT = 0,
  PWR_MGMT_1_CLK_SEL_WIDTH = 3,

  PWR_MGMT_2  = 0x6c,

	FIFO_COUNTH = 0x72,
	FIFO_COUNTL = 0x73,
	FIFO_R_W    = 0x74,
	WHO_AM_I    = 0x75,

	XA_OFFSET_H = 0x77,
	XA_OFFSET_L = 0x78,
	YA_OFFSET_H = 0x7a,
	YA_OFFSET_L = 0x7b,
	ZA_OFFSET_H = 0x7d,
	ZA_OFFSET_L = 0x7e,
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct icm20689_sensor_s
{
  struct sensor_lowerhalf_s 
                lower;        /* Lower half sensor driver. */
  uint64_t      last_update;
  bool          enabled;
  float         scale;
  unsigned long interval;
  FAR void      *dev;         /* The pointer to common device data of icm20689 */
};

/* Used by the driver to manage the device */

struct icm20689_dev_s
{
  struct icm20689_sensor_s priv[MAX_IDX];
  struct icm20689_config_s 
                config;  /* board-specific information */
  sem_t         run;     /* Locks sensor thread */
  mutex_t       lock;    /* mutex for this structure */
  struct work_s work;    /* Interrupt handler worker. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int icm20689_read_reg(FAR struct icm20689_dev_s *dev,
                      enum icm20689_regaddr_e regaddr, FAR uint8_t *buf, uint8_t len);

int icm20689_write_reg(FAR struct icm20689_dev_s *dev,
                       enum icm20689_regaddr_e regaddr, FAR const uint8_t *buf, uint8_t len);

int icm20689_modify_reg(FAR struct icm20689_dev_s *dev,
                        enum icm20689_regaddr_e regaddr, uint8_t clearbits, uint8_t setbits);

int icm20689_checkid(FAR struct icm20689_dev_s *dev);

/****************************************************************************
 * Name: icm20689_configspi
 *
 * Description:
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ICM20689_SPI
inline void icm20689_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the ICM20689 */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, ICM20689_SPI_MAXFREQUENCY);
}
#endif
#endif /* __INCLUDE_NUTTX_SENSORS_ICM20689_COMMOM_H */
