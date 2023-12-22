/****************************************************************************
 * drivers/sensors/lis3mdl_uorb.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/lis3mdl_uorb.h>

#if defined(CONFIG_SENSORS_LIS3MDL)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sets bit @n */

#define BIT(n)  (1 << (n))

/* Creates a mask of @m bits, i.e. MASK(2) -> 00000011 */

#define MASK(m) (BIT(m) - 1)

/* Masks and shifts @v into bit field @m */

#define TO_BITFIELD(m,v) (((v) & MASK(m ##_WIDTH)) << (m ##_SHIFT))

/* Un-masks and un-shifts bit field @m from @v */

#define FROM_BITFIELD(m,v) (((v) >> (m ##_SHIFT)) & MASK(m ##_WIDTH))

/****************************************************************************
 * Private
 ****************************************************************************/

#define LIS3MDL_DEVID           (0x3d)

#define MAG_CTRL_REG2_FS_SHIFT  (5)
#define MAG_CTRL_REG2_FS_WIDTH  (2)

struct lis3mdl_sensor_data_s
{
  int16_t x_mag;              /* Measurement result for x axis */
  int16_t y_mag;              /* Measurement result for y axis */
  int16_t z_mag;              /* Measurement result for z axis */
  int16_t temperature;        /* Measurement result for temperature sensor */
};

struct lis3mdl_sensor_s
{
  struct sensor_lowerhalf_s 
                lower;        /* Lower half sensor driver. */
  uint64_t      last_update;
  bool          enabled;
  float         scale;
  unsigned long interval;
  FAR void      *dev;         /* The pointer to common device data of lis3mdl */
};

/* Used by the driver to manage the device */

struct lis3mdl_dev_s
{
  struct lis3mdl_sensor_s priv;
  struct lis3mdl_config_s config;     /* Board-specific information */
  struct lis3mdl_sensor_data_s data;  /* The data as measured by the sensor */                
  sem_t         run;                  /* Locks sensor thread */
  mutex_t       lock;                 /* Manages exclusive access to this structure */
  struct work_s work;                 /* Interrupt handler worker. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor methods */

static int lis3mdl_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR unsigned long *period_us);
static int lis3mdl_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable);
static int lis3mdl_control(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            int cmd, unsigned long arg);

/* Sensor handle functions */

static int lis3mdl_read_register(FAR struct lis3mdl_dev_s *dev,
                                  uint8_t const reg_addr,
                                  uint8_t *buf, uint8_t len);
static int lis3mdl_write_register(FAR struct lis3mdl_dev_s *dev,
                                   uint8_t const reg_addr,
                                   uint8_t const *buf, uint8_t len);
static void lis3mdl_reset(FAR struct lis3mdl_dev_s *dev);
static void lis3mdl_read_measurement_data(FAR struct lis3mdl_dev_s *dev);
static void lis3mdl_read_magnetic_data(FAR struct lis3mdl_dev_s *dev,
                                       uint16_t *x_mag, uint16_t *y_mag,
                                       uint16_t *z_mag);
static void lis3mdl_read_temperature(FAR struct lis3mdl_dev_s *dev,
                                     uint16_t *temperature);

static int lis3mdl_write_range(FAR struct lis3mdl_dev_s *dev, 
                               uint8_t fs_sel);
static void lis3mdl_scale(FAR struct lis3mdl_sensor_s *priv,
                          enum mag_fs_config scale);

/* Sensor poll functions */

static void lis3mdl_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_lis3mdl_ops =
{
  NULL,                   /* open */
  NULL,                   /* close */
  lis3mdl_activate,       /* activate */
  lis3mdl_set_interval,   /* set_interval */
  NULL,                   /* batch */
  NULL,                   /* fetch */
  NULL,                   /* selftest */
  NULL,                   /* set_calibvalue */
  NULL,                   /* calibrate */
  lis3mdl_control         /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3mdl_activate
 *
 * Description: Activate the sensor.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int lis3mdl_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct lis3mdl_sensor_s *priv = (FAR struct lis3mdl_sensor_s *)lower;
  FAR struct lis3mdl_dev_s *dev = (FAR struct lis3mdl_dev_s *)(priv->dev);
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
          ret = work_queue(LPWORK, &dev->work, 
                          lis3mdl_worker, dev, 
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
      work_cancel(LPWORK, &dev->work);
    }

  return OK;
}

/****************************************************************************
 * Name: lis3mdl_set_interval
 *
 * Description: Set data output interval of sensor.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int lis3mdl_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *interval)
{
  FAR struct lis3mdl_sensor_s *priv = (FAR struct lis3mdl_sensor_s *)lower;

  priv->interval = *interval;

  return 0;
}

/****************************************************************************
 * Name: lis3mdl_control
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int lis3mdl_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg)
{
  FAR struct lis3mdl_sensor_s *priv = (FAR struct lis3mdl_sensor_s *)lower;
  FAR struct lis3mdl_dev_s *dev = (FAR struct lis3mdl_dev_s *)(priv->dev);
  int ret;

  switch (cmd)
    {
      /* Set full scale command */

      case SNIOC_SET_SCALE_XL:
        {
          switch (priv->lower.type)
            {
              /* Set magnetometer full scale */

              case SENSOR_TYPE_MAGNETIC_FIELD:
                {
                  ret = lis3mdl_write_range(dev, (uint8_t)arg);
                  lis3mdl_scale(priv, (enum mag_fs_config)arg);
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
 * Name: lis3mdl_scale
 *
 * Description: Set scale of magnetometer.
 *
 * FS_SEL | Full Scale Range  | LSB Sensitivity
 * -------+-------------------+----------------
 * 0      | +/- 4  gauss      | 6842  LSB/gauss
 * 1      | +/- 8  gauss      | 3421  LSB/gauss
 * 2      | +/- 12 gauss      | 2281  LSB/gauss
 * 3      | +/- 16 gauss      | 1711  LSB/gauss
 ****************************************************************************/

static void lis3mdl_scale(FAR struct lis3mdl_sensor_s *priv,
                          enum mag_fs_config scale)
{
  switch (scale)
    {
      case MAG_FS_4GAUSS:
        priv->scale = (1.0f / 6842);
        break;

      case MAG_FS_8GAUSS:
        priv->scale = (1.0f / 3421);
        break;

      case MAG_FS_12GAUSS:
        priv->scale = (1.0f / 2281);
        break;

      case MAG_FS_16GAUSS:
        priv->scale = (1.0f / 1711);
        break;
      default:
        break;
    }
}

#ifdef CONFIG_LIS3MDL_SPI

/****************************************************************************
 * Name: lis3mdl_read_reg_spi
 ****************************************************************************/

static int lis3mdl_read_reg_spi(FAR struct lis3mdl_dev_s *dev,
                                uint8_t const reg_addr, uint8_t *buf, uint8_t len)
{
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;
  uint8_t reg;

  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(spi, true);
  SPI_SETMODE(dev->config.spi, LIS3MDL_SPI_MODE); 
  SPI_SETFREQUENCY(dev->config.spi, dev->config.freq);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(spi, id, true);

  /* Transmit the register address from where we want to read - the MSB
   * needs to be set to indicate the read indication.
   */

  reg = reg_addr | 0x80;

  /* If multiple read is requested than set the address increment bit */

  if (len > 1)
    {
      reg |= 0x40;      
    }

  SPI_SEND(spi, reg);

  /* Write an idle byte while receiving the required data */

  while (0 != len--)
    {
      *buf++ = (uint8_t) (SPI_SEND(spi, 0));
    }

  /* Set CS to high which deselects the LIS3MDL */

  SPI_SELECT(spi, id, false);

  /* Unlock the SPI bus */

  SPI_LOCK(spi, false);

  return OK;
}

/****************************************************************************
 * Name: lis3mdl_write_reg_spi
 ****************************************************************************/

static int lis3mdl_write_reg_spi(FAR struct lis3mdl_dev_s *dev,
                                 uint8_t const reg_addr,
                                 uint8_t const *buf, uint8_t len)
{
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;
  uint8_t reg = reg_addr;

  /* Lock the SPI bus so that only one device can access it at the same
   * time
   */

  SPI_LOCK(spi, true);
  SPI_SETMODE(dev->config.spi, LIS3MDL_SPI_MODE); 
  SPI_SETFREQUENCY(dev->config.spi, dev->config.freq);

  /* Set CS to low which selects the LIS3MDL */

  SPI_SELECT(spi, id, true);

  /* If multiple write is requested than set the address increment bit */

  if (len > 1)
    {
      reg |= 0x40;      
    }

  /* Transmit the register address from where we want to read */

  SPI_SEND(spi, reg);

  /* Transmit the content which should be written in the register */

  while (0 != len--)
    {
      SPI_SEND(spi, *buf++);
    }

  /* Set CS to high which deselects the LIS3MDL */

  SPI_SELECT(spi, id, false);

  /* Unlock the SPI bus */

  SPI_LOCK(spi, false);

  return OK;
}
#endif /* CONFIG_LIS3MDL_SPI */

#ifdef CONFIG_LIS3MDL_I2C

/****************************************************************************
 * Name: lis3mdl_read_reg_i2c
 ****************************************************************************/

static int lis3mdl_read_reg_i2c(FAR struct lis3mdl_dev_s *dev,
                                uint8_t const reg_addr, uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_LIS3MDL_I2C_FREQ;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = (FAR uint8_t *)&reg_addr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_LIS3MDL_I2C_FREQ;
  msg[1].addr      = dev->config.addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = (FAR uint8_t *)buf;
  msg[1].length    = len;

  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(read) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: lis3mdl_write_reg_i2c
 ****************************************************************************/

static int lis3mdl_write_reg_i2c(FAR struct lis3mdl_dev_s *dev,
                                 uint8_t const reg_addr,
                                 uint8_t const *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_LIS3MDL_I2C_FREQ;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = (FAR uint8_t *)&reg_addr;
  msg[0].length    = 1;
  msg[1].frequency = CONFIG_LIS3MDL_I2C_FREQ;
  msg[1].addr      = dev->config.addr;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = (FAR uint8_t *)buf;
  msg[1].length    = len;
  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(write) failed: %d\n", ret);
      return ret;
    }

  return OK;
}
#endif /* CONFIG_LIS3MDL_I2C */

/****************************************************************************
 * Name: lis3mdl_read_register
 ****************************************************************************/

static int lis3mdl_read_register(FAR struct lis3mdl_dev_s *dev,
                                uint8_t const reg_addr, 
                                uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_LIS3MDL_SPI
  /* If we're wired to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return lis3mdl_read_reg_spi(dev, reg_addr, buf, len);
    }
#endif /* CONFIG_LIS3MDL_SPI */

#ifdef CONFIG_LIS3MDL_I2C
  /* If we're wired to I2C, use that function. */

  if (dev->config.i2c != NULL)
    {
      return lis3mdl_read_reg_i2c(dev, reg_addr, buf, len);
    }
#endif /* CONFIG_LIS3MDL_I2C */

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/****************************************************************************
 * Name: lis3mdl_write_register
 ****************************************************************************/

static int lis3mdl_write_register(FAR struct lis3mdl_dev_s *dev,
                                  uint8_t const reg_addr,
                                  uint8_t const *buf, uint8_t len)
{
#ifdef CONFIG_LIS3MDL_SPI
  /* If we're wired to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return lis3mdl_write_reg_spi(dev, reg_addr, buf, len);
    }
#endif /* CONFIG_LIS3MDL_SPI */

#ifdef CONFIG_LIS3MDL_I2C
  /* If we're wired to I2C, use that function. */

  if (dev->config.i2c != NULL)
    {
      return lis3mdl_write_reg_i2c(dev, reg_addr, buf, len);
    }
#endif /* CONFIG_LIS3MDL_I2C */

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/****************************************************************************
 * Name: lis3mdl_reset
 ****************************************************************************/

static void lis3mdl_reset(FAR struct lis3mdl_dev_s *dev)
{
  uint8_t val = LIS3MDL_CTRL_REG_2_SOFT_RST_BM;
  lis3mdl_write_register(dev,
                         LIS3MDL_CTRL_REG_2,
                         &val, 1);

  up_mdelay(100);
}

/****************************************************************************
 * Name: lis3mdl_read_measurement_data
 ****************************************************************************/

static void lis3mdl_read_measurement_data(FAR struct lis3mdl_dev_s *dev)
{
  /* Magnetic data */

  uint16_t x_mag = 0;
  uint16_t y_mag = 0;
  uint16_t z_mag = 0;

  lis3mdl_read_magnetic_data(dev, &x_mag, &y_mag, &z_mag);

  /* Temperature */

  uint16_t temperature = 0;

  lis3mdl_read_temperature(dev, &temperature);

  /* Copy retrieve data to internal data structure */

  dev->data.x_mag = (int16_t) (x_mag);
  dev->data.y_mag = (int16_t) (y_mag);
  dev->data.z_mag = (int16_t) (z_mag);
  dev->data.temperature = (int16_t) (temperature);

  /* Feed sensor data to entropy pool */

  add_sensor_randomness((x_mag << 16) ^ (y_mag << 10) ^ (z_mag << 2) ^
                        temperature);
}

/****************************************************************************
 * Name: lis3mdl_read_magnetic_data
 ****************************************************************************/

static void lis3mdl_read_magnetic_data(FAR struct lis3mdl_dev_s *dev,
                                       uint16_t * x_mag, uint16_t * y_mag,
                                       uint16_t * z_mag)
{
  uint8_t mag_data[6];
  int ret;

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set -> Read Indication 0x40 -> MSB-1 (MS-Bit) is
   * set -> auto increment of address when reading multiple bytes.
   */

  ret = lis3mdl_read_register(dev, LIS3MDL_OUT_X_L_REG, mag_data, sizeof(mag_data));
  if (ret < 0)
    {
      return;
    }

  *x_mag  = ((uint16_t) (mag_data[0]) << 0);     /* LSB */
  *x_mag |= ((uint16_t) (mag_data[1]) << 8);     /* MSB */

  *y_mag  = ((uint16_t) (mag_data[2]) << 0);     /* LSB */
  *y_mag |= ((uint16_t) (mag_data[3]) << 8);     /* MSB */

  *z_mag  = ((uint16_t) (mag_data[4]) << 0);     /* LSB */
  *z_mag |= ((uint16_t) (mag_data[5]) << 8);     /* MSB */
}

/****************************************************************************
 * Name: lis3mdl_read_temperature
 ****************************************************************************/

static void lis3mdl_read_temperature(FAR struct lis3mdl_dev_s *dev,
                                     uint16_t * temperature)
{
  uint8_t temp_data[2];
  int ret;

  /* Transmit the register address from where we want to start reading
   * 0x80 -> MSB is set -> Read Indication 0x40 -> MSB-1 (MS-Bit) is
   * set -> auto increment of address when reading multiple bytes.
   */

  ret = lis3mdl_read_register(dev, LIS3MDL_TEMP_OUT_L_REG, temp_data, sizeof(temp_data));
  if (ret < 0)
    {
      return;
    }

  /* RX */

  *temperature  = ((uint16_t) (temp_data[0]) << 0);    /* LSB */
  *temperature |= ((uint16_t) (temp_data[1]) << 8);    /* MSB */
}

/****************************************************************************
 * Name: lis3mdl_write_range
 ****************************************************************************/

static int lis3mdl_write_range(FAR struct lis3mdl_dev_s *dev, uint8_t fs_sel)
{
  uint8_t val = TO_BITFIELD(MAG_CTRL_REG2_FS, fs_sel);  
  return lis3mdl_write_register(dev, LIS3MDL_CTRL_REG_2, &val, 1);
}

/****************************************************************************
 * Name: lis3mdl_checkid
 *
 * Description:
 *   Read and verify the LIS3MDL chip ID
 *
 ****************************************************************************/

static int lis3mdl_checkid(FAR struct lis3mdl_dev_s *dev)
{
  uint8_t devid;
  int ret;

  /* Read device ID */

  ret = lis3mdl_read_register(dev, LIS3MDL_WHO_AM_I_REG, &devid, sizeof(devid));
  if (ret < 0)
    {
      return ret;
    }

  sninfo("devid: %04x\n", devid);
  if (devid != LIS3MDL_DEVID)
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lis3mdl_mag_data
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

static void lis3mdl_mag_data(FAR struct lis3mdl_sensor_s *priv,
                             FAR struct lis3mdl_sensor_data_s *buf)
{
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  uint64_t now = sensor_get_timestamp();
  struct sensor_mag mag;

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }

  priv->last_update = now;

  mag.timestamp = now;
  mag.status = 0;
  mag.x = buf->x_mag * priv->scale;
  mag.y = buf->y_mag * priv->scale;
  mag.z = buf->z_mag * priv->scale;
  mag.temperature = buf->temperature;

  lower->push_event(lower->priv, &mag, sizeof(mag));
  sninfo("Mag: %.3fgauss %.3fgauss %.3fgauss, t:%.1f\n",
         mag.x, mag.y, mag.z, mag.temperature);
}

/****************************************************************************
 * Name: lis3mdl_worker
 ****************************************************************************/

static void lis3mdl_worker(FAR void *arg)
{
  FAR struct lis3mdl_dev_s *dev = (FAR struct lis3mdl_dev_s *)arg;
  FAR struct lis3mdl_sensor_s *priv = &dev->priv;

  /* Re-schedule the worker */

  work_queue(LPWORK, &dev->work,
             lis3mdl_worker, dev, 
             priv->interval / USEC_PER_TICK);

  /* Read out the latest sensor data */

  lis3mdl_read_measurement_data(dev);

  /* Publish the topic */

  lis3mdl_mag_data(priv, &dev->data);
}

/****************************************************************************
 * Name: lis3mdl_initialize
 *
 * Description: Initialize the LIS3MDL motion tracker
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int lis3mdl_initialize(FAR struct lis3mdl_dev_s *dev)
{
  uint8_t reg_content;
  uint8_t reg_addr;

  /* Lock the device */

  nxmutex_lock(&dev->lock);

  /* Perform a reset */

  lis3mdl_reset(dev);

  /* Enable * - the maximum full scale mode. * Full scale = +/- 1.6 mT (16
   * Gauss).
   */

  reg_content = LIS3MDL_CTRL_REG_2_FS_1_BM |
                LIS3MDL_CTRL_REG_2_FS_0_BM;
  lis3mdl_write_register(dev,
                         LIS3MDL_CTRL_REG_2,
                         &reg_content, 1);

  /* Enable - temperature sensor - ultra high performance mode (UMP) for X
   * and Y - fast output data rates This results in a output data rate of
   * 155 Hz for X and Y.
   */

  reg_content = LIS3MDL_CTRL_REG_1_TEMP_EN_BM |
                LIS3MDL_CTRL_REG_1_OM_1_BM |
                LIS3MDL_CTRL_REG_1_OM_0_BM |
                LIS3MDL_CTRL_REG_1_FAST_ODR_BM;
  lis3mdl_write_register(dev,
                         LIS3MDL_CTRL_REG_1,
                         &reg_content, 1);

  /* Enable * - ultra high performance mode (UMP) for Z * This should result
   * to the same output data rate as for X and Y.
   */

  reg_content = LIS3MDL_CTRL_REG_4_OMZ_1_BM |
                LIS3MDL_CTRL_REG_4_OMZ_0_BM;
  lis3mdl_write_register(dev,
                         LIS3MDL_CTRL_REG_4,
                         &reg_content, 1);

  /* Enable * - block data update for magnetic sensor data * This should
   * prevent race conditions when reading sensor data.
   */

  reg_content = LIS3MDL_CTRL_REG_5_BDU_BM;
  lis3mdl_write_register(dev,
                         LIS3MDL_CTRL_REG_5,
                         &reg_content, 1);

  /* Enable continuous conversion mode - the device starts measuring now. */

  reg_content = 0;
  lis3mdl_write_register(dev, LIS3MDL_CTRL_REG_3, &reg_content, 1);

  /* Read measurement data to ensure DRDY is low */

  lis3mdl_read_measurement_data(dev);

  /* Read back the content of all control registers for debug purposes */

  reg_content = 0;
  for (reg_addr = LIS3MDL_CTRL_REG_1;
       reg_addr <= LIS3MDL_CTRL_REG_5;
       reg_addr++)
    {
      lis3mdl_read_register(dev, reg_addr, &reg_content, 1);
      sninfo("R#%04x = %04x\n", reg_addr, reg_content);
    }

  lis3mdl_read_register(dev, LIS3MDL_STATUS_REG, &reg_content, 1);
  sninfo("STATUS_REG = %04x\n", reg_content);

  /* Unlock the device */

  nxmutex_unlock(&dev->lock);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lis3mdl_uorb_register
 *
 * Description:
 *   Register the LIS3MDL character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/mag0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             LIS3MDL
 *   config  - configuration for the LIS3MDL driver. For details see
 *             description above.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis3mdl_uorb_register(int devno, FAR struct lis3mdl_config_s const *config)
{
  FAR struct lis3mdl_dev_s *dev;
  FAR struct lis3mdl_sensor_s *tmp;
  int ret;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the LIS3MDL device structure */

  dev = kmm_malloc(sizeof(struct lis3mdl_dev_s));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }
  memset(dev, 0, sizeof(*dev));

  /* Keep a copy of the config structure, in case the caller discards
   * theirs.
   */

  dev->config = *config;

  /* Initialize sensor data access mutex */

  nxmutex_init(&dev->lock);


  /* Check the device id */

  ret = lis3mdl_checkid(dev);
  if (ret < 0)
    {
      goto error_id;
    }

  /* Magnetic field register */

  tmp = &dev->priv;
  tmp->lower.ops = &g_lis3mdl_ops;
  tmp->lower.type = SENSOR_TYPE_MAGNETIC_FIELD;
  tmp->lower.nbuffer = 1;
  tmp->dev = dev;
  tmp->interval = 1000000 / CONFIG_LIS3MDL_MEASURE_FREQ;
  ret = sensor_register(&tmp->lower, devno);
  tmp->enabled = false;
  tmp->scale = 1;
  if (ret < 0)
    {
      goto error_id;
    }

  /* Reset the chip, to give it an initial configuration. */

  ret = lis3mdl_initialize(dev);
  if (ret < 0)
    {
      goto error;
    }

  /* Set the scale value */

  lis3mdl_scale(tmp, MAG_FS_16GAUSS);

  return ret;

error:
  sensor_unregister(&dev->priv.lower, devno);
error_id:
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);

  return ret;
}

#endif /* CONFIG_SENSORS_LIS3MDL_UORB */
