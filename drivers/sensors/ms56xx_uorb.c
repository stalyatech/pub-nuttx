/****************************************************************************
 * drivers/sensors/ms56xx_uorb.c
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

#include "ms56xx_base.h"
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>

#if defined(CONFIG_SENSORS_MS56XX) && \
    (defined(CONFIG_I2C) || defined(CONFIG_SPI))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MS56XX_CMD_RESET              0x1e
#define MS56XX_CMD_START_ADC_READ     0x00
#define MS56XX_CMD_CONV_D1_OSR_256    0x40 /* D1 = uncompensated pressure */
#define MS56XX_CMD_CONV_D1_OSR_512    0x42
#define MS56XX_CMD_CONV_D1_OSR_1024   0x44
#define MS56XX_CMD_CONV_D1_OSR_2048   0x46
#define MS56XX_CMD_CONV_D1_OSR_4096   0x48
#define MS56XX_CMD_CONV_D2_OSR_256    0x50 /* D2 = uncompensated pressure */
#define MS56XX_CMD_CONV_D2_OSR_512    0x52
#define MS56XX_CMD_CONV_D2_OSR_1024   0x54
#define MS56XX_CMD_CONV_D2_OSR_2048   0x56
#define MS56XX_CMD_CONV_D2_OSR_4096   0x58
#define MS56XX_CMD_ADC_READ           0x00
#define MS56XX_CMD_ADC_PROM_READ(i)   (0xa0 + (i)*2) /* 0xA0 - 0xAE */

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor methods */

static int ms56xx_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us);
static int ms56xx_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);
static int ms56xx_control(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            int cmd, unsigned long arg);

static int ms56xx_sendcmd(FAR struct ms56xx_dev_s *dev,
                          uint8_t cmd);
static int ms56xx_read16(FAR struct ms56xx_dev_s *dev,
                         FAR uint8_t *regval);
static int ms56xx_read24(FAR struct ms56xx_dev_s *dev,
                         FAR uint8_t *regval);

static int32_t ms56xx_compensate_temp(FAR struct ms56xx_dev_s *dev,
                                      uint32_t temp_raw, int32_t *deltat);
static uint32_t ms56xx_compensate_press(FAR struct ms56xx_dev_s *dev,
                                        uint32_t press, uint32_t dt,
                                        int32_t *temp);

static unsigned long ms56xx_curtime(void);

/* Sensor poll functions */

static void ms56xx_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_ms56xx_ops =
{
  NULL,                   /* open */
  NULL,                   /* close */
  ms56xx_activate,        /* activate */
  ms56xx_set_interval,    /* set_interval */
  NULL,                   /* batch */
  NULL,                   /* fetch */
  NULL,                   /* selftest */
  NULL,                   /* set_calibvalue */
  NULL,                   /* calibrate */
  ms56xx_control          /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms56xx_activate
 *
 * Description: Activate the sensor.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ms56xx_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct ms56xx_sensor_s *priv = (FAR struct ms56xx_sensor_s *)lower;
  FAR struct ms56xx_dev_s *dev = (FAR struct ms56xx_dev_s *)(priv->dev);
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
                          ms56xx_worker, dev,
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
 * Name: ms56xx_set_interval
 *
 * Description: Set data output interval of sensor.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ms56xx_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR unsigned long *interval)
{
  FAR struct ms56xx_sensor_s *priv = (FAR struct ms56xx_sensor_s *)lower;

  priv->interval = *interval;

  return OK;
}

/****************************************************************************
 * Name: ms56xx_control
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ms56xx_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg)
{
  FAR struct ms56xx_sensor_s *priv = (FAR struct ms56xx_sensor_s *)lower;
  FAR struct ms56xx_dev_s *dev = (FAR struct ms56xx_dev_s *)(priv->dev);
  int ret;

  UNUSED(dev);
  switch (cmd)
    {
      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: ms56xx_curtime
 *
 * Description: Helper to get current timestamp.
 *
 * Return:
 *   Timestamp in microseconds
 ****************************************************************************/

static unsigned long ms56xx_curtime(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;
}

/****************************************************************************
 * Name: ms56xx_sendcmd
 *
 * Description:
 *   Send a command (8-bit) to MS56XX
 *
 ****************************************************************************/

static int ms56xx_sendcmd(FAR struct ms56xx_dev_s *dev, uint8_t cmd)
{
  int ret;

  ret = ms56xx_write(dev, &cmd, 1);
  if (ret < 0)
    {
      snerr("ms56xx_sendcmd failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ms56xx_read16
 *
 * Description:
 *   Read 16-bit from a MS56XX register
 *
 ****************************************************************************/

static int ms56xx_read16(FAR struct ms56xx_dev_s *dev, FAR uint8_t *regval)
{
  int ret;

  ret = ms56xx_read(dev, regval, 2);
  if (ret < 0)
    {
      snerr("ms56xx_read16 failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ms56xx_read24
 *
 * Description:
 *   Read 24-bit from a MS56XX register
 *
 ****************************************************************************/

static int ms56xx_read24(FAR struct ms56xx_dev_s *dev, uint8_t *regval)
{
  int ret;

  ret = ms56xx_read(dev, regval, 3);
  if (ret < 0)
    {
      snerr("ms56xx_read24 failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ms56xx_sendcmd
 *
 * Description:
 *   Read barometer data from MS56XX
 *
 ****************************************************************************/

static inline void baro_measure_read(FAR struct ms56xx_dev_s *dev,
                                     FAR struct sensor_baro *baro)
{
  uint32_t press;
  uint32_t temp_raw;
  int32_t temp;
  int32_t deltat;
  int ret;
  uint8_t buffer[3];

  /* Enforce exclusive access */

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return;
    }

  /* Send command to start a D1 (pressure) conversion */

  ret = ms56xx_sendcmd(dev, MS56XX_CMD_CONV_D1_OSR_4096);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS56XX_CMD_CONV_D1_OSR_4096!\n");
      return;
    }

  /* Wait data acquisition */

  up_udelay(10000);

  /* Send command to start a read sequence */

  ret = ms56xx_sendcmd(dev, MS56XX_CMD_START_ADC_READ);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS56XX_CMD_START_ADC_READ!\n");
      return;
    }

  /* Wait data get ready */

  up_udelay(4000);

  ret = ms56xx_read24(dev, buffer);
  if (ret < 0)
    {
      snerr("Fail to read pressure!\n");
      return;
    }

  press = (uint32_t) buffer[0] << 16 |
          (uint32_t) buffer[1] << 8 |
          (uint32_t) buffer[2];

  /* Send command to start a D2 (temperature) conversion */

  ret = ms56xx_sendcmd(dev, MS56XX_CMD_CONV_D2_OSR_4096);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS56XX_CMD_CONV_D2_OSR_4096!\n");
      return;
    }

  /* Wait data acquisition */

  up_udelay(10000);

  /* Send command to start a read sequence */

  ret = ms56xx_sendcmd(dev, MS56XX_CMD_START_ADC_READ);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS56XX_CMD_START_ADC_READ!\n");
      return;
    }

  /* Wait data get ready */

  up_udelay(4000);

  ret = ms56xx_read24(dev, buffer);
  if (ret < 0)
    {
      snerr("Fail to read temperature!\n");
      return;
    }

  temp_raw = (uint32_t) buffer[0] << 16 |
             (uint32_t) buffer[1] << 8 |
             (uint32_t) buffer[2];

  /* Release the mutex */

  nxmutex_unlock(&dev->lock);

  /* Compensate the temp/press with calibration data */

  temp = ms56xx_compensate_temp(dev, temp_raw, &deltat);
  press = ms56xx_compensate_press(dev, press, deltat, &temp);

  baro->timestamp = ms56xx_curtime();
  baro->pressure = press / 100.0f;
  baro->temperature = temp / 100.0f;
}

/****************************************************************************
 * Name: ms56xx_compensate_temp
 *
 * Description:
 *   calculate compensate temperature
 *
 * Input Parameters:
 *   temp - uncompensate value of temperature.
 *
 * Returned Value:
 *   calculate result of compensate temperature.
 *
 ****************************************************************************/

static int32_t ms56xx_compensate_temp(FAR struct ms56xx_dev_s *dev,
                                      uint32_t temp_raw, int32_t *deltat)
{
  FAR struct ms56xx_sensor_s *priv = &dev->priv;
  struct ms56xx_calib_s *c = &priv->calib;
  int32_t dt;
  int32_t temp;

  /* dt = d1 - c5 * 256 */

  dt = temp_raw - ((int32_t) c->c5 << 8);

  /* temp = 2000 + (dt * c6) / 8388608 */

  temp = 2000 + (((int64_t) (dt * c->c6)) >> 23);

  /* Save dt that will be used for pressure calibration */

  *deltat = dt;

  return temp;
}

/****************************************************************************
 * Name: ms56xx_compensate_press
 *
 * Description:
 *   calculate compensate pressure
 *
 * Input Parameters:
 *   press - uncompensate value of pressure.
 *
 * Returned Value:
 *   calculate result of compensate pressure.
 *
 ****************************************************************************/

static uint32_t ms56xx_compensate_press(FAR struct ms56xx_dev_s *dev,
                                        uint32_t press, uint32_t dt,
                                        int32_t *temp)
{
  FAR struct ms56xx_sensor_s *priv = &dev->priv;
  struct ms56xx_calib_s *c = &priv->calib;
  int64_t off = 0;
  int64_t sens = 0;
#if defined(CONFIG_MS56XX_SECOND_ORDER_COMPENSATE)
  int64_t off2 = 0;
  int64_t sens2 = 0;
  int64_t t2 = 0;
  uint64_t delta;
#endif

  switch (dev->config.model)
    {
      case MS56XX_MODEL_MS5607:
        off = ((int64_t) c->c2 << 17) + ((int64_t) (c->c4 * dt) >> 6);
        sens = ((int64_t) c->c1 << 16) + ((int64_t) (c->c3 * dt) >> 7);
#if defined(CONFIG_MS56XX_SECOND_ORDER_COMPENSATE)
        if (*temp < 2000)
          {
            /* Low temperature */

            t2 = ((dt * dt) >> 31);
            delta = *temp - 2000;
            delta *= delta;
            off2 = (61 * delta) >> 4;
            sens2 = 2 * delta;

            if (*temp < -1500)
              {
                /* Very low temperature */

                delta = *temp + 1500;
                delta *= delta;
                off2 += 15 * delta;
                sens2 += 8 * delta;
              }
          }
#endif
        break;

      case MS56XX_MODEL_MS5611:
        off = ((int64_t) c->c2 << 16) + ((int64_t) (c->c4 * dt) >> 7);
        sens = ((int64_t) c->c1 << 15) + ((int64_t) (c->c3 * dt) >> 8);
#if defined(CONFIG_MS56XX_SECOND_ORDER_COMPENSATE)
        if (*temp < 2000)
          {
            /* Low temperature */

            t2 = ((dt * dt) >> 31);
            delta = *temp - 2000;
            delta *= delta;
            off2 = (5 * delta) >> 1;
            sens2 = (5 * delta) >> 2;

            if (*temp < -1500)
              {
                /* Very low temperature */

                delta = *temp + 1500;
                delta *= delta;
                off2 += 7 * delta;
                sens2 += (11 * delta) >> 1;
              }
          }
#endif
        break;
    }

#if defined(CONFIG_MS56XX_SECOND_ORDER_COMPENSATE)
  *temp -= t2;
  off -= off2;
  sens -= sens2;
#endif
  press = (((press * sens) >> 21) - off) >> 15;

  return press;
}

/* Sensor poll functions */

/****************************************************************************
 * Name: ms56xx_worker
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

static void ms56xx_worker(FAR void *arg)
{
  FAR struct ms56xx_dev_s *dev = (FAR struct ms56xx_dev_s *)arg;
  FAR struct ms56xx_sensor_s *priv = &dev->priv;
  FAR struct sensor_lowerhalf_s *lower = &priv->lower;
  struct sensor_baro baro_data;

  /* Re-schedule the worker */

  work_queue(LPWORK, &dev->work,
             ms56xx_worker, dev,
             priv->interval / USEC_PER_TICK);

  /* Read the barometer data */

  baro_measure_read(dev, &baro_data);

  /* Push the data */

  lower->push_event(lower->priv, &baro_data, sizeof(struct sensor_baro));
}

/****************************************************************************
 * Name: ms56xx_initialize
 *
 * Description:
 *   Initialize MS56XX device
 *
 ****************************************************************************/

static int ms56xx_initialize(FAR struct ms56xx_dev_s *dev)
{
  FAR struct ms56xx_sensor_s *priv = &dev->priv;
  uint16_t prom[8];
  uint8_t data[2];
  uint8_t crc;
  int i, ret;

  /* Lock the device */

  nxmutex_lock(&dev->lock);

  /* Get calibration data. */

  ret = ms56xx_sendcmd(dev, MS56XX_CMD_RESET);
  if (ret < 0)
    {
      snerr("ms56xx reset failed\n");
      return ret;
    }

  /* We have to wait before the prom is ready is be read */

  up_udelay(10000);

  for (i = 0; i < 8; i++)
    {
      ret = ms56xx_sendcmd(dev, MS56XX_CMD_ADC_PROM_READ(i));
      if (ret < 0)
        {
          snerr("ms56xx_sendcmd failed\n");
          goto errout;
        }

      ret = ms56xx_read16(dev, data);
      if (ret < 0)
        {
          snerr("ms56xx_read16 failed\n");
          goto errout;
        }

      prom[i] = (uint16_t) data[0] << 8 | (uint16_t) data[1];
    }

  /* Get the 4-bit CRC from PROM */

  crc = (uint8_t)(prom[7] & 0xf);

  /* Verify if the calculated CRC is equal to PROM's CRC */

  if (crc != msxxxx_crc4(prom, 7, 0xff))
    {
      snerr("ERROR: Calculated CRC different from PROM's CRC!\n");
      //ret = -ENODEV;
      //goto errout;
    }

  /* Fill read calibration coefficients */

  priv->calib.c1 = prom[1];
  priv->calib.c2 = prom[2];
  priv->calib.c3 = prom[3];
  priv->calib.c4 = prom[4];
  priv->calib.c5 = prom[5];
  priv->calib.c6 = prom[6];

errout:
  /* Unlock the device */

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms56xx_register
 *
 * Description:
 *   Register the MS56XX character device
 *
 * Input Parameters:
 *   devno   - Instance number for driver
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ms56xx_register(int devno, FAR struct ms56xx_config_s *config)
{
  FAR struct ms56xx_dev_s *dev;
  FAR struct ms56xx_sensor_s *tmp;
  int ret;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the MS56XX device structure */

  dev = kmm_zalloc(sizeof(struct ms56xx_dev_s));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate ms56xx device instance\n");
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

  ret = ms56xx_checkid(dev);
  if (ret < 0)
    {
      goto error_id;
    }

  /* Barometer register */

  tmp = &dev->priv;
  tmp->lower.ops = &g_ms56xx_ops;
  tmp->lower.type = SENSOR_TYPE_BAROMETER;
  tmp->lower.nbuffer = 1;
  tmp->dev = dev;
  tmp->interval = 1000000 / CONFIG_MS56XX_MEASURE_FREQ;
  tmp->enabled = false;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto error_id;
    }

  /* Reset the chip, to give it an initial configuration. */

  ret = ms56xx_initialize(dev);
  if (ret < 0)
    {
      goto error;
    }

  return ret;

error:
  sensor_unregister(&dev->priv.lower, devno);
error_id:
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);

  return ret;
}

#endif
