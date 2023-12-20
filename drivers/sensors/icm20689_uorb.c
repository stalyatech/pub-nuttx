/****************************************************************************
 * drivers/sensors/icm20689_uorb.c
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

#include "icm20689_base.h"
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>

#if defined(CONFIG_SENSORS_ICM20689_UORB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  MIN(x, y)      (x) > (y) ? (y) : (x)

/****************************************************************************
 * Private Function Function Prototypes
 ****************************************************************************/

/* Sensor methods */

static int icm20689_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR unsigned long *period_us);
static int icm20689_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable);
static int icm20689_control(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            int cmd, unsigned long arg);

/* Sensor handle functions */

static inline int icm20689_write_gyro_range(FAR struct icm20689_dev_s *dev,
                                           uint8_t fs_sel);
static void icm20689_gyro_scale(FAR struct icm20689_sensor_s *priv,
                               enum gyro_config_bit scale);
static inline int icm20689_write_accel_range(FAR struct icm20689_dev_s *dev,
                                            uint8_t afs_sel);
static void icm20689_accel_scale(FAR struct icm20689_sensor_s *priv,
                                enum accel_config_bit scale);

/* Sensor poll functions */

static void icm20689_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_icm20689_ops =
{
  NULL,                   /* open */
  NULL,                   /* close */
  icm20689_activate,      /* activate */
  icm20689_set_interval,  /* set_interval */
  NULL,                   /* batch */
  NULL,                   /* fetch */
  NULL,                   /* selftest */
  NULL,                   /* set_calibvalue */
  NULL,                   /* calibrate */
  icm20689_control        /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm20689_activate
 *
 * Description: Activate the sensor.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int icm20689_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct icm20689_sensor_s *priv = (FAR struct icm20689_sensor_s *)lower;
  FAR struct icm20689_dev_s *dev = (FAR struct icm20689_dev_s *)(priv->dev);
  int ret;

  if (enable)
    {
      if (!priv->enabled)
        {
          /* Enable the sensor */

          priv->enabled = true;
          priv->last_update = sensor_get_timestamp();
        }

      /* Schedule the worker */

      if (work_available(&dev->work))
        {
          ret = work_queue(HPWORK, &dev->work, 
                          icm20689_worker, dev, 
                          priv->interval / USEC_PER_TICK);
          if (ret < 0)
            {
              snerr("ERROR: Failed to queue work: %d\n", ret);
            }
        }
    }
  else
    {
      /* Set suspend mode to sensors. */

      priv->enabled = false;
      work_cancel(HPWORK, &dev->work);
    }

  return OK;
}

/****************************************************************************
 * Name: icm20689_set_interval
 *
 * Description: Set data output interval of sensor.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int icm20689_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *interval)
{
  FAR struct icm20689_sensor_s *priv = (FAR struct icm20689_sensor_s *)lower;

  priv->interval = *interval;

  return OK;
}

/****************************************************************************
 * Name: icm20689_control
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int icm20689_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg)
{
  FAR struct icm20689_sensor_s *priv = (FAR struct icm20689_sensor_s *)lower;
  FAR struct icm20689_dev_s *dev = (FAR struct icm20689_dev_s *)(priv->dev);
  int ret;

  switch (cmd)
    {
      /* Set full scale command */

      case SNIOC_SET_SCALE_XL:
        {
          switch (priv->lower.type)
            {
              /* Set gyroscope full scale */

              case SENSOR_TYPE_GYROSCOPE:
                {
                  ret = icm20689_write_gyro_range(dev, (uint8_t)arg);
                  icm20689_gyro_scale(priv, (enum gyro_config_bit)arg);
                }
                break;

              /* Set accelerometer full scale */

              case SENSOR_TYPE_ACCELEROMETER:
                {
                  ret = icm20689_write_accel_range(dev, (uint8_t)arg);
                  icm20689_accel_scale(priv, (enum accel_config_bit)arg);
                }
                break;

              default:
                snerr("ERROR: Unrecognized type: %d\n", priv->lower.type);
                ret = -ENOTTY;
                break;
            }
        }
        break;

      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: icm20689_accel_scale
 *
 * Description: Set scale of accelerometer.
 *
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 16384 LSB/mg
 * 1       | +/- 4g           | 8192 LSB/mg
 * 2       | +/- 8g           | 4096 LSB/mg
 * 3       | +/- 16g          | 2048 LSB/mg
 ****************************************************************************/

static void icm20689_accel_scale(FAR struct icm20689_sensor_s *priv,
                                enum accel_config_bit scale)
{
  switch (scale)
    {
      case ACCEL_FS_SEL_2G:
        priv->scale = CONSTANTS_ONE_G / 16384.f;
        break;

      case ACCEL_FS_SEL_4G:
        priv->scale = CONSTANTS_ONE_G / 8192.f;
        break;

      case ACCEL_FS_SEL_8G:
        priv->scale = CONSTANTS_ONE_G / 4096.f;
        break;

      case ACCEL_FS_SEL_16G:
        priv->scale = CONSTANTS_ONE_G / 2048.f;
        break;
      default:
        break;
    }
}

/****************************************************************************
 * Name: icm20689_gyro_scale
 *
 * Description: Set scale of accelerometer.
 *
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 ****************************************************************************/

static void icm20689_gyro_scale(FAR struct icm20689_sensor_s *priv,
                               enum gyro_config_bit scale)
{
  switch (scale)
    {
      case GYRO_FS_SEL_250_DPS:
        priv->scale = (M_PI / 180.0f) * 250.f / 32768.f;
        break;

      case GYRO_FS_SEL_500_DPS:
        priv->scale = (M_PI / 180.0f) * 500.f / 32768.f;
        break;

      case GYRO_FS_SEL_1000_DPS:
        priv->scale = (M_PI / 180.0f) * 1000.f / 32768.f;
        break;

      case GYRO_FS_SEL_2000_DPS:
        priv->scale = (M_PI / 180.0f) * 2000.f / 32768.f;
        break;
      default:
        break;
    }
}

/****************************************************************************
 * icm20689_read_imu()
 *
 * Reads the whole IMU data file from @dev in one uninterrupted pass,
 * placing the sampled values into @buf. This function is the only way
 * to guarantee that the measured values are sampled as closely-spaced
 * in time as the hardware permits, which is almost always what you
 * want.
 ****************************************************************************/

static inline int icm20689_read_imu(FAR struct icm20689_dev_s *dev,
                                   FAR struct sensor_data_s *buf)
{
  return icm20689_read_reg(dev, ACCEL_XOUT_H, (uint8_t *) buf, sizeof(*buf));
}

/* icm20689_read_pwr_mgmt_1()
 *
 * Returns the value of the PWR_MGMT_1 register from @dev.
 */

static inline uint8_t icm20689_read_pwr_mgmt_1(FAR struct icm20689_dev_s *dev)
{
  uint8_t buf = 0xff;

  icm20689_read_reg(dev, PWR_MGMT_1, &buf, sizeof(buf));
  return buf;
}

static inline int icm20689_write_signal_reset(FAR struct icm20689_dev_s *dev,
                                                  uint8_t val)
{
  return icm20689_write_reg(dev, SIGNAL_PATH_RESET, &val, sizeof(val));
}

static inline int icm20689_write_int_pin_cfg(FAR struct icm20689_dev_s *dev,
                                            uint8_t val)
{
  return icm20689_write_reg(dev, INT_PIN_CFG, &val, sizeof(val));
}

static inline int icm20689_write_pwr_mgmt_1(FAR struct icm20689_dev_s *dev,
                                           uint8_t val)
{
  return icm20689_write_reg(dev, PWR_MGMT_1, &val, sizeof(val));
}

static inline int icm20689_write_pwr_mgmt_2(FAR struct icm20689_dev_s *dev,
                                           uint8_t val)
{
  return icm20689_write_reg(dev, PWR_MGMT_2, &val, sizeof(val));
}

static inline int icm20689_write_user_ctrl(FAR struct icm20689_dev_s *dev,
                                          uint8_t val)
{
  return icm20689_write_reg(dev, USER_CTRL, &val, sizeof(val));
}

static inline int icm20689_write_fifo_en(FAR struct icm20689_dev_s *dev,
                                        uint8_t val)
{
  return icm20689_write_reg(dev, FIFO_EN, &val, sizeof(val));
}

/****************************************************************************
 * icm20689_write_gyro_range() :
 *
 * Sets the @fs_sel bit in GYRO_CONFIG to the value provided. Per the
 * datasheet, the meaning of @fs_sel is as follows:
 *
 * GYRO_CONFIG(0x1b) :   XG_ST YG_ST ZG_ST FS_SEL1 FS_SEL0 x  x  x
 *
 *    XG_ST, YG_ST, ZG_ST  :  self-test (unsupported in this driver)
 *         1 -> activate self-test on X, Y, and/or Z gyros
 *
 *    FS_SEL[10] : full-scale range select
 *         0 -> ±  250 deg/sec
 *         1 -> ±  500 deg/sec
 *         2 -> ± 1000 deg/sec
 *         3 -> ± 2000 deg/sec
 ****************************************************************************/

static inline int icm20689_write_gyro_range(FAR struct icm20689_dev_s *dev,
                                           uint8_t fs_sel)
{
  uint8_t val = TO_BITFIELD(GYRO_CONFIG_FS_SEL, fs_sel);
  return icm20689_write_reg(dev, GYRO_CONFIG, &val, sizeof(val));
}

/****************************************************************************
 * icm20689_write_accel_range() :
 *
 * Sets the @afs_sel bit in ACCEL_CONFIG to the value provided. Per
 * the datasheet, the meaning of @afs_sel is as follows:
 *
 * ACCEL_CONFIG(0x1c) :   XA_ST YA_ST ZA_ST AFS_SEL1 AFS_SEL0 x  x  x
 *
 *    XA_ST, YA_ST, ZA_ST  :  self-test (unsupported in this driver)
 *         1 -> activate self-test on X, Y, and/or Z accelerometers
 *
 *    AFS_SEL[10] : full-scale range select
 *         0 -> ±  2 g
 *         1 -> ±  4 g
 *         2 -> ±  8 g
 *         3 -> ± 16 g
 ****************************************************************************/

static inline int icm20689_write_accel_range(FAR struct icm20689_dev_s *dev,
                                            uint8_t afs_sel)
{
  uint8_t val = TO_BITFIELD(ACCEL_CONFIG_AFS_SEL, afs_sel);
  return icm20689_write_reg(dev, ACCEL_CONFIG, &val, sizeof(val));
}

/****************************************************************************
 * CONFIG (0x1a) :   x   x   EXT_SYNC_SET[2..0] DLPF_CFG[2..0]
 *
 *    EXT_SYNC_SET  : frame sync bit position
 *    DLPF_CFG      : digital low-pass filter bandwidth
 ****************************************************************************/

static inline int icm20689_write_config(FAR struct icm20689_dev_s *dev,
                                     uint8_t ext_sync_set, uint8_t dlpf_cfg)
{
  uint8_t val = TO_BITFIELD(CONFIG_EXT_SYNC_SET, ext_sync_set) |
                TO_BITFIELD(CONFIG_DLPF_CFG, dlpf_cfg);
  return icm20689_write_reg(dev, CONFIG, &val, sizeof(val));
}

/****************************************************************************
 * CONFIG2 (0x1d) :   x   x   accel_fchoice_b  A_DLPF_CFG[2..0]
 *
 *    accel_fchoice_b  : he inverted version of accel_fchoice
 *    A_DLPF_CFG      : Accelerometer low pass filter setting
 ****************************************************************************/

static inline int icm20689_write_config2(FAR struct icm20689_dev_s *dev,
                                        uint8_t acce_fchoice_b,
                                        uint8_t a_dlpf_cfg)
{
  uint8_t val = acce_fchoice_b | TO_BITFIELD(CONFIG2_A_DLPF_CFG, a_dlpf_cfg);
  return icm20689_write_reg(dev, ACCEL_CONFIG2, &val, sizeof(val));
}

/****************************************************************************
 * Name: icm20689_initialize
 *
 * Description: Initialize the ICM20689 motion tracker
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int icm20689_initialize(FAR struct icm20689_dev_s *dev)
{
  int ret = OK;

#ifdef CONFIG_ICM20689_SPI
  if (dev->config.spi == NULL)
    {
      return -EINVAL;
    }
#endif /* CONFIG_ICM20689_SPI */ 

#ifdef CONFIG_ICM20689_I2C 
  if (dev->config.i2c == NULL)
    {
      return -EINVAL;
    }
#endif /* CONFIG_ICM20689_I2C */

  nxmutex_lock(&dev->lock);

  /* Awaken chip, issue hardware reset */

  ret = icm20689_write_pwr_mgmt_1(dev, PWR_MGMT_1_DEVICE_RESET);
  if (ret < 0)
    {
      snerr("icm20689 write_pwr_mgmt_1 error!\n");
      goto errout;
    }

  /* Wait for reset cycle to finish (note: per the datasheet, we don't need
   * to hold NSS for this)
   */

  do
    {
      nxsig_usleep(50000);            /* usecs (arbitrary) */
    }
  while (icm20689_read_pwr_mgmt_1(dev) & PWR_MGMT_1_DEVICE_RESET);

  /* Reset signal paths */

  ret = icm20689_write_signal_reset(dev, SIGNAL_PATH_RESET_ALL_RESET);
  if (ret < 0)
    {
      snerr("icm20689 write_signal_path_reset error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Disable SLEEP, use PLL with z-axis clock source */

  ret = icm20689_write_pwr_mgmt_1(dev, 3);
  if (ret < 0)
    {
      snerr("icm20689 write_pwr_mgmt_1 error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Disable low-power mode, enable all gyros and accelerometers */

  ret = icm20689_write_pwr_mgmt_2(dev, 0);
  if (ret < 0)
    {
      snerr("icm20689 write_pwr_mgmt_2 error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* clear first, and separate *_RST from *_EN */

  ret = icm20689_write_user_ctrl(dev, 0);
  if (ret < 0)
    {
      snerr("icm20689 write_user_ctrl error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  ret = icm20689_write_fifo_en(dev, 0);
  if (ret < 0)
    {
      snerr("icm20689 write_fifo_en error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Reset I2C Master module. */

  ret = icm20689_write_user_ctrl(dev, USER_CTRL_FIFO_RST | 
                                      USER_CTRL_SIG_COND_RST);
  if (ret < 0)
    {
      snerr("icm20689 write_user_ctrl error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Disable i2c if we're on spi. */

#ifdef CONFIG_ICM20689_SPI
  if (dev->config.spi)
    {
      ret = icm20689_write_user_ctrl(dev, USER_CTRL_I2C_IF_DIS);
    }
#endif /* CONFIG_ICM20689_SPI */

#ifdef CONFIG_ICM20689_I2C
  if (dev->config.i2c)
    {
      ret = icm20689_write_user_ctrl(dev, 0);
    }
#endif /* CONFIG_ICM20689_I2C */

  if (ret < 0)
    {
      snerr("icm20689 write_user_ctrl error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* default No FSYNC, set accel LPF at 184 Hz, gyro LPF at 188 Hz in
   * menuconfig
   */

  ret = icm20689_write_config(dev, CONFIG_ICM20689_EXT_SYNC_SET,
                             CONFIG_ICM20689_DLPF_CFG);
  if (ret < 0)
    {
      snerr("icm20689 write_config error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* default ± 1000 deg/sec in menuconfig */

  ret = icm20689_write_gyro_range(dev, CONFIG_ICM20689_GYRO_FS_SEL);
  if (ret < 0)
    {
      snerr("icm20689 write_gyro_range error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* default ± 8g in menuconfig */

  ret = icm20689_write_accel_range(dev, CONFIG_ICM20689_ACCEL_AFS_SEL);
  if (ret < 0)
    {
      snerr("icm20689 write_accel_range error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* Accelerometer low pass filter setting */

  ret = icm20689_write_config2(dev, CONFIG_ICM20689_ACCEL_FCHOICE_B,
                              CONFIG_ICM20689_A_DLPF_CFG);
  if (ret < 0)
    {
      snerr("icm20689 write_config2 error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

  /* clear INT on any read (we aren't using that pin right now) */

  ret = icm20689_write_int_pin_cfg(dev, INT_PIN_CFG_INT_RD_CLEAR);
  if (ret < 0)
    {
      snerr("icm20689 write int pin cfg error!\n");
      goto errout;
    }

  nxsig_usleep(1000);

errout:
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: swap16
 *
 * Description: swap H and L byte of a 16bit Data
 *
 * Parameter:
 *   val  - Big endian Data
 *
 * Return:
 *   do nothing if Big endian, swap H and L byte if little endian
 ****************************************************************************/

static uint16_t swap16(uint16_t val)
{
#ifdef CONFIG_ENDIAN_BIG
  return val;
#else
  return (val >> 8) | (val << 8);
#endif
}

/****************************************************************************
 * Name: icm20689_accel_data
 *
 * Description: get and push accel data from struct sensor_data_s
 *
 * Parameter:
 *   priv  - Internal private lower half driver instance
 *   buf  - Point to struct sensor_data_s
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static void icm20689_accel_data(FAR struct icm20689_sensor_s *priv,
                               FAR struct sensor_data_s *buf)
{
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  uint64_t now = sensor_get_timestamp();
  struct sensor_accel accel;

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }

  priv->last_update = now;

  accel.timestamp = now;
  accel.x = (int16_t)swap16(buf->x_accel) * priv->scale;
  accel.y = (int16_t)swap16(buf->y_accel) * priv->scale;
  accel.z = (int16_t)swap16(buf->z_accel) * priv->scale;
  accel.temperature = swap16(buf->temp) / 333.87f + 21.0f;

  lower->push_event(lower->priv, &accel, sizeof(accel));
  sninfo("Accel: %.3fm/s^2 %.3fm/s^2 %.3fm/s^2, t:%.1f\n",
         accel.x, accel.y, accel.z, accel.temperature);
}

/****************************************************************************
 * Name: icm20689_gyro_data
 *
 * Description: get and push gyro data from struct sensor_data_s
 *
 * Parameter:
 *   priv  - Internal private lower half driver instance
 *   buf  - Point to struct sensor_data_s
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static void icm20689_gyro_data(FAR struct icm20689_sensor_s *priv,
                              FAR struct sensor_data_s *buf)
{
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  uint64_t now = sensor_get_timestamp();
  struct sensor_gyro gyro;

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }

  priv->last_update = now;

  gyro.timestamp = now;
  gyro.x = (int16_t)swap16(buf->x_gyro) * priv->scale;
  gyro.y = (int16_t)swap16(buf->y_gyro) * priv->scale;
  gyro.z = (int16_t)swap16(buf->z_gyro) * priv->scale;
  gyro.temperature = swap16(buf->temp) / 333.87f + 21.0f;

  lower->push_event(lower->priv, &gyro, sizeof(gyro));
  sninfo("Gyro: %.3frad/s %.3frad/s %.3frad/s, t:%.1f\n",
          gyro.x, gyro.y, gyro.z, gyro.temperature);
}

/* Sensor poll functions */

/****************************************************************************
 * Name: icm20689_worker
 *
 * Description:
 *   Task the worker with retrieving the latest sensor data. We should not do
 *   this in a interrupt since it might take too long. Also we cannot lock
 *   the bus from within an interrupt.
 *
 * Input Parameters:
 *   arg    - Device struct.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void icm20689_worker(FAR void *arg)
{
  FAR struct icm20689_dev_s *dev = (FAR struct icm20689_dev_s *)arg;
  FAR struct icm20689_sensor_s *accel = &dev->priv[ACCEL_IDX];
  FAR struct icm20689_sensor_s *gyro = &dev->priv[GYRO_IDX];
  struct sensor_data_s buf;   /* temporary buffer (for read(), etc.) */
  unsigned long  min_interval;
  int ret;

  /* Sleeping thread before fetching the next sensor data */

  min_interval = MIN(accel->interval, gyro->interval);

  /* Re-schedule the worker */

  work_queue(HPWORK, &dev->work,
             icm20689_worker, dev, 
             min_interval / USEC_PER_TICK);

  /* Returns a snapshot of the accelerometer, temperature, and gyro
    * registers.
    *
    * Note: the chip uses traditional, twos-complement notation, i.e. "0"
    * is encoded as 0, and full-scale-negative is 0x8000, and
    * full-scale-positive is 0x7fff. If we read the registers
    * sequentially and directly into memory (as we do), the measurements
    * from each sensor are captured as big endian words.
    */

  ret = icm20689_read_imu(dev, &buf);
  if (ret == OK)
    {
      icm20689_accel_data(accel, &buf);
      icm20689_gyro_data(gyro, &buf); 
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm20689_register
 *
 * Description:
 *   Registers the icm20689 character device
 *
 * Input Parameters:
 *   devno   - Instance number for driver
 *   config  - Configuration information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int icm20689_register(int devno, FAR struct icm20689_config_s *config)
{
  FAR struct icm20689_dev_s *dev;
  FAR struct icm20689_sensor_s *tmp;
  int ret;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the device structure. */

  dev = kmm_malloc(sizeof(struct icm20689_dev_s));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate icm20689 device instance\n");
      return -ENOMEM;
    }

  memset(dev, 0, sizeof(*dev));
  nxmutex_init(&dev->lock);

  /* Keep a copy of the config structure, in case the caller discards
   * theirs.
   */

  dev->config = *config;

  /* Check the device id */

  ret = icm20689_checkid(dev);
  if (ret < 0)
    {
      goto error;
    }

  /* Accelerometer register */

  tmp = &dev->priv[ACCEL_IDX];
  tmp->lower.ops = &g_icm20689_ops;
  tmp->lower.type = SENSOR_TYPE_ACCELEROMETER;
  tmp->lower.nbuffer = 1;
  tmp->dev = dev;
  tmp->interval = 1000000 / CONFIG_ICM20689_MEASURE_FREQ;
  tmp->enabled = false;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto error;
    }

  icm20689_accel_scale(tmp, CONFIG_ICM20689_ACCEL_AFS_SEL);

  /* Gyroscope register */

  tmp = &dev->priv[GYRO_IDX];
  tmp->lower.ops = &g_icm20689_ops;
  tmp->lower.type = SENSOR_TYPE_GYROSCOPE;
  tmp->lower.nbuffer = 1;
  tmp->dev = dev;
  tmp->interval = 1000000 / CONFIG_ICM20689_MEASURE_FREQ;
  tmp->enabled = false;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto error;
    }

  icm20689_gyro_scale(tmp, CONFIG_ICM20689_GYRO_FS_SEL);

  /* Reset the chip, to give it an initial configuration. */

  ret = icm20689_initialize(dev);
  if (ret < 0)
    {
      goto error;
    }

  return ret;

error:
  sensor_unregister(&dev->priv[GYRO_IDX].lower, devno);
  sensor_unregister(&dev->priv[ACCEL_IDX].lower, devno);
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);

  return ret;
}

#endif /* CONFIG_SENSORS_ICM20689_UORB */
