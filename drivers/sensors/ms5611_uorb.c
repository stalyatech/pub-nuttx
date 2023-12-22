/****************************************************************************
 * drivers/sensors/ms5611_uorb.c
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

#include "ms5611_base.h"
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>

#if defined(CONFIG_SENSORS_MS5611_UORB) && \
    (defined(CONFIG_I2C) || defined(CONFIG_SPI))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MS5611_CMD_RESET              0x1e
#define MS5611_CMD_START_ADC_READ     0x00
#define MS5611_CMD_CONV_D1_OSR_256    0x40 /* D1 = uncompensated pressure */
#define MS5611_CMD_CONV_D1_OSR_512    0x42
#define MS5611_CMD_CONV_D1_OSR_1024   0x44
#define MS5611_CMD_CONV_D1_OSR_2048   0x46
#define MS5611_CMD_CONV_D1_OSR_4096   0x48
#define MS5611_CMD_CONV_D2_OSR_256    0x50 /* D2 = uncompensated pressure */
#define MS5611_CMD_CONV_D2_OSR_512    0x52
#define MS5611_CMD_CONV_D2_OSR_1024   0x54
#define MS5611_CMD_CONV_D2_OSR_2048   0x56
#define MS5611_CMD_CONV_D2_OSR_4096   0x58
#define MS5611_CMD_ADC_READ           0x00
#define MS5611_CMD_ADC_PROM_READ(i)   (0xa0 + (i)*2) /* 0xA0 - 0xAE */

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ms5611_sendcmd(FAR struct ms5611_dev_s *dev,
                          uint8_t cmd);
static int ms5611_readadc(FAR struct ms5611_dev_s *dev,
                          FAR uint8_t *regval);
static int ms5611_readprom(FAR struct ms5611_dev_s *dev, 
                           uint8_t regaddr,
                           FAR uint8_t *regval);

static int32_t ms5611_compensate_temp(FAR struct ms5611_dev_s *dev,
                                      uint32_t temp, int32_t *deltat);
static uint32_t ms5611_compensate_press(FAR struct ms5611_dev_s *dev,
                                        uint32_t press, uint32_t dt);

static unsigned long ms5611_curtime(void);

/* Sensor methods */

static int ms5611_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us);
static int ms5611_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);

/* Sensor poll functions */

static void ms5611_worker(FAR void *arg);

#if 0 /* Please read below */
static int ms5611_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_ms5611_ops =
{
  .activate      = ms5611_activate,
  .fetch         = NULL, /* ms5611_fetch */
  .set_interval  = ms5611_set_interval,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms5611_crc4
 *
 * Description: MS5611 crc4 cribbed from the datasheet.
 *
 * Return:
 *   Checksum failure status
 ****************************************************************************/

static bool ms5611_crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */

	crc_read = n_prom[7];

	/* remove CRC byte */

	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) 
    {
      /* uneven bytes */

      if (cnt & 1) 
        {
          n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

        } 
      else 
        {
          n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
        }

      for (n_bit = 8; n_bit > 0; n_bit--) 
        {
          if (n_rem & 0x8000) 
            {
              n_rem = (n_rem << 1) ^ 0x3000;
            } 
          else 
            {
              n_rem = (n_rem << 1);
            }
        }
    }

	/* final 4 bit remainder is CRC value */

	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */

	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

/****************************************************************************
 * Name: ms5611_curtime
 *
 * Description: Helper to get current timestamp.
 *
 * Return:
 *   Timestamp in microseconds
 ****************************************************************************/

static unsigned long ms5611_curtime(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;
}

/****************************************************************************
 * Name: ms5611_sendcmd
 *
 * Description:
 *   Send a command (8-bit) to MS5611
 *
 ****************************************************************************/

static int ms5611_sendcmd(FAR struct ms5611_dev_s *dev, uint8_t cmd)
{
  return ms5611_write(dev, &cmd, 1);
}

/****************************************************************************
 * Name: ms5611_readprom
 *
 * Description:
 *   Read 16-bit PROM data from a MS5611 register
 *
 ****************************************************************************/

static int ms5611_readprom(FAR struct ms5611_dev_s *dev, uint8_t regaddr, uint8_t *regval)
{
  return ms5611_transfer(dev, &regaddr, 1, regval, 2);
}

/****************************************************************************
 * Name: ms5611_readadc
 *
 * Description:
 *   Read 24-bit ADC calue from the MS5611
 *
 ****************************************************************************/

static int ms5611_readadc(FAR struct ms5611_dev_s *dev, uint8_t *regval)
{
  uint8_t cmd = MS5611_CMD_START_ADC_READ;
  return ms5611_transfer(dev, &cmd, 1, regval, 3);
}

static inline void baro_measure_read(FAR struct ms5611_dev_s *dev,
                                     FAR struct sensor_baro *baro)
{
  uint32_t press;
  uint32_t temp;
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

  ret = ms5611_sendcmd(dev, MS5611_CMD_CONV_D1_OSR_1024);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS5611_CMD_CONV_D1_OSR_1024!\n");
      return;
    }

  /* Wait data acquisition */

  up_udelay(10000);

  /* Send command to start a read sequence */

  ret = ms5611_readadc(dev, buffer);
  if (ret < 0)
    {
      snerr("Fail to read adc!\n");
      return;
    }

  /* get ready data */

  press = (uint32_t) buffer[0] << 16 |
          (uint32_t) buffer[1] << 8 |
          (uint32_t) buffer[2];

  /* Send command to start a D2 (temperature) conversion */

  ret = ms5611_sendcmd(dev, MS5611_CMD_CONV_D2_OSR_1024);
  if (ret < 0)
    {
      snerr("Fail to send cmd MS5611_CMD_CONV_D2_OSR_1024!\n");
      return;
    }

  /* Wait data acquisition */

  up_udelay(10000);

  /* Send command to start a read sequence */

  ret = ms5611_readadc(dev, buffer);
  if (ret < 0)
    {
      snerr("Fail to read adc!\n");
      return;
    }

  /* get ready data */

  temp = (uint32_t) buffer[0] << 16 |
         (uint32_t) buffer[1] << 8 |
         (uint32_t) buffer[2];

  /* Release the mutex */

  nxmutex_unlock(&dev->lock);

  /* Compensate the temp/press with calibration data */

  temp = ms5611_compensate_temp(dev, temp, &deltat);
  press = ms5611_compensate_press(dev, press, deltat);

  baro->timestamp = ms5611_curtime();
  baro->pressure = press / 100.0f;
  baro->temperature = temp / 100.0f;
}

/****************************************************************************
 * Name: ms5611_initialize
 *
 * Description:
 *   Initialize MS5611 device
 *
 ****************************************************************************/

static int ms5611_initialize(FAR struct ms5611_dev_s *dev)
{
  struct ms5611_sensor_s *priv = &dev->priv;
  bool all_zero = true;
  uint16_t prom[8];
  uint8_t data[2];
  int i, retry;
  int ret;

  /* reset and read PROM (try up to 3 times) */

  for (retry = 0; retry < 3; retry++)
    {
      /* Get calibration data. */

      ret = ms5611_sendcmd(dev, MS5611_CMD_RESET);
      if (ret < 0)
        {
          snerr("ms5611 reset failed\n");
          continue;
        }

      /* We have to wait before the prom is ready is be read */

      up_udelay(10000);

	    /* read and convert PROM words */

      for (i = 0; i < 8; i++)
        {
          ret = ms5611_readprom(dev, MS5611_CMD_ADC_PROM_READ(i), data);
          if (ret < 0)
            {
              snerr("ms5611_readprom failed\n");
              continue;
            }

          prom[i] = (uint16_t) data[0] << 8 | (uint16_t) data[1];

          /* Check for zero value */

          if (prom[i] != 0)
            {
              all_zero = false;
            }
        }

      /* Verify if the calculated CRC is equal to PROM's CRC */

      if (!all_zero && ms5611_crc4(prom))
        {
          /* Fill read calibration coefficients */

          priv->calib.c1 = prom[1];
          priv->calib.c2 = prom[2];
          priv->calib.c3 = prom[3];
          priv->calib.c4 = prom[4];
          priv->calib.c5 = prom[5];
          priv->calib.c6 = prom[6];

          return OK;
        }
    }

  /* Check for all zero condition */

  if (all_zero)
    {
      snerr("ERROR: PROM all zero!\n");
      return -ENODEV;      
    }

  /* Fill read calibration coefficients */

  priv->calib.c1 = prom[1];
  priv->calib.c2 = prom[2];
  priv->calib.c3 = prom[3];
  priv->calib.c4 = prom[4];
  priv->calib.c5 = prom[5];
  priv->calib.c6 = prom[6];

  snwarn("WARNING: Calculated CRC different from PROM's CRC!\n");
  return OK;
}

/****************************************************************************
 * Name: ms5611_compensate_temp
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

static int32_t ms5611_compensate_temp(FAR struct ms5611_dev_s *dev,
                                      uint32_t temp, int32_t *deltat)
{
  struct ms5611_sensor_s *priv = &dev->priv;
  struct ms5611_calib_s *c = &priv->calib;
  int32_t dt;

  /* dt = d1 - c5 * 256 */

  dt = temp - ((int32_t) c->c5 << 8);

  /* temp = 2000 + (dt * c6) / 8388608 */

  temp = 2000 + (((int64_t) (dt * c->c6)) >> 23);

  /* Save dt that will be used for pressure calibration */

  *deltat = dt;

  return temp;
}

/****************************************************************************
 * Name: ms5611_compensate_press
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

static uint32_t ms5611_compensate_press(FAR struct ms5611_dev_s *dev,
                                        uint32_t press, uint32_t dt)
{
  struct ms5611_sensor_s *priv = &dev->priv;
  struct ms5611_calib_s *c = &priv->calib;
  int64_t off;
  int64_t sens;

  off = ((int64_t) c->c2 * 65536) + ((int64_t) (c->c4 * dt) / 128);
  sens = ((int64_t) c->c1 * 32768) + ((int64_t) (c->c3 * dt) / 256);
  press = (((press * sens) / 2097152) - off) / 32768;

  return press;
}

/****************************************************************************
 * Name: ms5611_set_interval
 ****************************************************************************/

static int ms5611_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us)
{
  FAR struct ms5611_sensor_s *priv = (FAR struct ms5611_sensor_s *)lower;

  priv->interval = *period_us;

  return OK;
}

/****************************************************************************
 * Name: ms5611_activate
 ****************************************************************************/

static int ms5611_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct ms5611_sensor_s *priv = (FAR struct ms5611_sensor_s *)lower;
  FAR struct ms5611_dev_s *dev = (FAR struct ms5611_dev_s *)(priv->dev);
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
                          ms5611_worker, dev, 
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
 * Name: ms5611_fetch
 ****************************************************************************/

/* N.B. When fetch is enabled the sensortest doesn't respect the
 * interval (-i) parameter, so let keep it comment until further
 * discussion about the "issue".
 */

#if 0
static int ms5611_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR char *buffer, size_t buflen)
{
  FAR struct ms5611_dev_s *dev = container_of(lower,
                                               FAR struct ms5611_dev_s,
                                               sensor_lower);
  struct sensor_baro baro_data;

  if (buflen != sizeof(baro_data))
    {
      return -EINVAL;
    }

  baro_measure_read(priv, &baro_data);

  memcpy(buffer, &baro_data, sizeof(baro_data));

  return buflen;
}
#endif

/* Sensor poll functions */

/****************************************************************************
 * Name: ms5611_worker
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

static void ms5611_worker(FAR void *arg)
{
  FAR struct ms5611_dev_s *dev = (FAR struct ms5611_dev_s *)arg;
  FAR struct ms5611_sensor_s *priv = &dev->priv;
  struct sensor_baro baro_data;
  uint64_t now;

  /* Get the timestamp */

  now = sensor_get_timestamp();

  /* Re-schedule the worker */

  work_queue(LPWORK, &dev->work,
             ms5611_worker, dev, 
             priv->interval / USEC_PER_TICK);

  /* Check the update status */

  if (!priv->enabled || now - priv->last_update < priv->interval)
    {
      return;
    }
  priv->last_update = now;

  /* Read the data */

  baro_measure_read(dev, &baro_data);

  /* Push the event */

  priv->lower.push_event(priv->lower.priv, &baro_data,
                                sizeof(struct sensor_baro));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ms5611_register
 *
 * Description:
 *   Register the MS5611 character device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             MS5611
 *   devno   - Instance number for driver
 *   addr    - The I2C address of the MS5611.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ms5611_register(int devno, FAR struct ms5611_config_s *config)
{
  FAR struct ms5611_dev_s *dev;
  FAR struct ms5611_sensor_s *priv;
  int ret;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the MS5611 device structure */

  dev = kmm_zalloc(sizeof(struct ms5611_dev_s));
  if (dev == NULL)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  nxmutex_init(&dev->lock);

  /* Keep a copy of the config structure, in case the caller discards
   * theirs.
   */

  dev->config = *config;

  /* Register the sensor topic */

  priv = &dev->priv;
  priv->lower.ops = &g_ms5611_ops;
  priv->lower.type = SENSOR_TYPE_BAROMETER;
  priv->lower.nbuffer = 1;
  priv->dev = dev;
  priv->interval = 1000000 / CONFIG_MS5611_MEASURE_FREQ;
  priv->enabled = false;
  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      goto error_sen;
    }

  /* Reset the chip, to give it an initial configuration. */

  ret = ms5611_initialize(dev);
  if (ret < 0)
    {
      goto error;
    }

  sninfo("MS5611 driver loaded successfully!\n");
  return ret;

error:
  sensor_unregister(&dev->priv.lower, devno);
error_sen:
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);

  return ret;
}

#endif
