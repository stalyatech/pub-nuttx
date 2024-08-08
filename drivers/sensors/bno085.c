/****************************************************************************
 * drivers/sensors/bno085.c
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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/bno085.h>
#ifdef CONFIG_SENSORS_BNO085_I2C
#  include <nuttx/i2c/i2c_master.h>
#else
#  include <nuttx/spi/spi.h>
#endif

#include <nuttx/sensors/ceva/sh2/sh2.h>
#include <nuttx/sensors/ceva/sh2/sh2_hal.h>
#include <nuttx/sensors/ceva/sh2/sh2_err.h>
#include <nuttx/sensors/ceva/sh2/sh2_util.h>
#include <nuttx/sensors/ceva/sh2/sh2_SensorValue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BNO085_I2C_RETRIES  (10)
#define BNO085_FAULT_COUNT  (20)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bno085_dev_s
{
  FAR struct bno085_dev_s    *flink;      /* Supports a singly linked list of
                                           * drivers */
  FAR struct bno085_config_s *config;     /* Pointer to the configuration of the BNO085 sensor */
  uint64_t                    timestamp;  /* Units is microseconds */
  mutex_t                     lock;       /* Manages exclusive access to this structure */
  struct sh2_Hal_s            sh2;        /* CEVA SH2 interface */
  sh2_SensorValue_t           value;      /* Sensor value */
  volatile int16_t            intf;       /* Interrupt received but data not read, yet */
  struct work_s               work;       /* The work queue is responsible for
                                           * retrieving the data from the sensor
                                           * after the arrival of new data was
                                           * signalled in an interrupt */
  struct pollfd              *fds[CONFIG_BNO085_NPOLLWAITERS];
  volatile uint8_t            status;     /* Sensor status flag */
  volatile uint32_t           faulty;     /* Sensor fault counter */
  volatile uint8_t            initialized;/* Sensor initialized flag */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bno085_getregs(FAR struct bno085_dev_s *priv,
                          FAR uint8_t *regval, int len);
static int bno085_putregs(FAR struct bno085_dev_s *priv,
                          FAR uint8_t *regval, int len);
static void bno085_worker(FAR void *arg);

/* Character driver methods */

static int     bno085_open(FAR struct file *filep);
static int     bno085_close(FAR struct file *filep);
static ssize_t bno085_read(FAR struct file *filep,
                           FAR char *buffer, size_t len);
static ssize_t bno085_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     bno085_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int     bno085_poll(FAR struct file *filep,
                           FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_bno085fops =
{
  bno085_open,    /* open */
  bno085_close,   /* close */
  bno085_read,    /* read */
  bno085_write,   /* write */
  NULL,           /* seek */
  bno085_ioctl,   /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  bno085_poll     /* poll */
};

/* Single linked list to store instances of drivers */

static struct bno085_dev_s *g_bno085_list = NULL;

/****************************************************************************
 * shtp hal interface functions
 *
 ****************************************************************************/

/****************************************************************************
 * Name: shtp_i2c_hal_open
 *
 ****************************************************************************/

static int shtp_i2c_hal_open(sh2_Hal_t *self)
{
  struct bno085_dev_s *priv = self->priv;
  UNUSED(priv);

  return SH2_OK;
}

/****************************************************************************
 * Name: shtp_i2c_hal_close
 *
 ****************************************************************************/

static void shtp_i2c_hal_close(sh2_Hal_t *self)
{
  struct bno085_dev_s *priv = self->priv;
  UNUSED(priv);
}

/****************************************************************************
 * Name: shtp_i2c_hal_read
 *
 ****************************************************************************/

static int shtp_i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
  struct bno085_dev_s *priv = self->priv;

  if (bno085_getregs(priv, pBuffer, len) == OK)
    {
      *t = priv->timestamp; /* sensor_get_timestamp() */
      return len;
    }

  return SH2_ERR;
}

/****************************************************************************
 * Name: shtp_i2c_hal_write
 *
 ****************************************************************************/

static int shtp_i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
  struct bno085_dev_s *priv = self->priv;

  if (bno085_putregs(priv, pBuffer, len) == OK)
    {
      return len;
    }

  return SH2_ERR;
}

/****************************************************************************
 * Name: shtp_i2c_hal_getTimeUs
 *
 ****************************************************************************/

static uint32_t shtp_i2c_hal_getTimeUs(sh2_Hal_t *self)
{
    return sensor_get_timestamp();
}

/****************************************************************************
 * Name: bno085_getregs
 *
 * Description:
 *   Read (len) bytes from specified device
 *
 ****************************************************************************/

static int bno085_getregs(FAR struct bno085_dev_s *priv, FAR uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BNO085_I2C
  struct i2c_msg_s msg;
  int retries;
  int ret = -EIO;

  msg.frequency = priv->config->freq;
  msg.addr      = priv->config->devid;
  msg.flags     = I2C_M_READ;
  msg.buffer    = regval;
  msg.length    = len;

  /* Perform the transfer */

  for (retries = 0; retries < BNO085_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->dev, &msg, 1);
      if (ret >= 0)
        {
          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == BNO085_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->config->dev);
          if (ret < 0)
            {
              snerr("ERROR: I2C_RESET failed (%d)\n", ret);
              return ret;
            }
#endif
        }
    }

  snerr("ERROR: I2C_TRANSFER failed (%d)\n", ret);
  return ret;
#else /* CONFIG_SENSORS_BNO085_SPI */
  return -EIO;
#endif
}

/****************************************************************************
 * Name: bno085_putregs
 *
 * Description:
 *   Send (len) bytes for specified device
 *
 ****************************************************************************/

static int bno085_putregs(FAR struct bno085_dev_s *priv, FAR uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BNO085_I2C
  struct i2c_msg_s msg;
  int retries;
  int ret = -EIO;

  msg.frequency = priv->config->freq;
  msg.addr      = priv->config->devid;
  msg.flags     = 0;
  msg.buffer    = regval;
  msg.length    = len;

  /* Perform the transfer */

  for (retries = 0; retries < BNO085_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->dev, &msg, 1);
      if (ret >= 0)
        {
          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == BNO085_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->config->dev);
          if (ret < 0)
            {
              snerr("ERROR: I2C_RESET failed (%d)\n", ret);
              return ret;
            }
#endif
        }
    }

  snerr("ERROR: I2C_TRANSFER failed (%d)\n", ret);
  return ret;
#else
  return -EIO;
#endif
}

/****************************************************************************
 * Name: bno085_interrupt_handler
 ****************************************************************************/

static int bno085_interrupt_handler(int irq, FAR void *context,
                                    FAR void *arg)
{
  FAR struct bno085_dev_s *priv = arg;
  int ret = OK;

  /*
   * This function should be called upon a falling edge on the BNO085 new data
   * interrupt pin since it signals that new data has been measured.
   */

  /* Check the initialize flag */

  if (priv->initialized)
  {

    /* Find out which BNO085 device caused the interrupt */

    for (priv = g_bno085_list; priv && priv->config->irq != irq;
        priv = priv->flink);
    DEBUGASSERT(priv != NULL);

    /* Get the timestamp */

    priv->timestamp = sensor_get_timestamp();

    /* Increment the interrupt counter */

    priv->intf++;

    /* Task the worker with retrieving the latest sensor data. We should not do
    * this in a interrupt since it might take too long. Also we cannot lock
    * the bus from within an interrupt.
    */

    /* Schedule the worker immediately */

    if (work_available(&priv->work))
      {
        ret = work_queue(HPWORK, &priv->work, bno085_worker, priv, 0);
        if (ret < 0)
          {
            snerr("ERROR: Failed to queue work: %d\n", ret);
          }
      }
  }

  return ret;
}

/****************************************************************************
 * Name: bno085_worker
 ****************************************************************************/

static void bno085_worker(FAR void *arg)
{
  FAR struct bno085_dev_s *priv = arg;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Acquire the mutex before the data is copied */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      snerr("ERROR: Could not acquire priv->lock: %d\n", ret);
      return;
    }

  /* Check the initialize flag */

  if (priv->initialized)
    {
      /* Service the sensor hub.
      * Sensor reports and event processing handled by callbacks.
      */

      sh2_service();

      /* Re-schedule the worker (100Hz) */

      work_queue(HPWORK, &priv->work, bno085_worker, priv, USEC2TICK(10000));
    }

  /* Give back the mutex */

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: sh2_eventHandler
 *
 * Description:
 *   Handle non-sensor events from the sensor hub
 *
 ****************************************************************************/

static void sh2_eventHandler(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    struct bno085_dev_s *priv = cookie;
    UNUSED(priv);

    /* If we see a reset, set a flag so that sensors will be reconfigured */

    if (pEvent->eventId == SH2_RESET)
      {
        /* Set the status as ready and clear the faulty */

        priv->status = BNO085_STATUS_READY;
        priv->faulty = 0;

        sninfo("EventHandler  id:RESET\n");
      }
    else if (pEvent->eventId == SH2_SHTP_EVENT)
      {
        sninfo("EventHandler  id:SHTP, %d\n", (int)pEvent->shtpEvent);

        if (priv->status & BNO085_STATUS_READY)
          {
            switch (pEvent->shtpEvent)
              {
                case SH2_SHTP_TX_DISCARD:
                  break;

                case SH2_SHTP_INTERRUPTED_PAYLOAD:
                  break;

                case SH2_SHTP_SHORT_FRAGMENT:
                case SH2_SHTP_TOO_LARGE_PAYLOADS:
                case SH2_SHTP_BAD_RX_CHAN:
                case SH2_SHTP_BAD_TX_CHAN:
                case SH2_SHTP_BAD_FRAGMENT:
                case SH2_SHTP_BAD_SN:

                  /* Set the faulty status */

                  if (++priv->faulty >= BNO085_FAULT_COUNT)
                    {
                      priv->status |= BNO085_STATUS_FAULT;
                    }
                  break;
              }
          }
      }
    else if (pEvent->eventId == SH2_GET_FEATURE_RESP)
      {
        sninfo("EventHandler Sensor Config, %d\n", (int)pEvent->sh2SensorConfigResp.sensorId);
      }
    else
    {
        sninfo("EventHandler, unknown event Id: %d\n", (int)pEvent->eventId);
    }
}

/****************************************************************************
 * Name: sh2_sensorHandler
 *
 * Description:
 *   Handle sensor events from the sensor hub
 *
 ****************************************************************************/

static void sh2_sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent)
{
  struct bno085_dev_s *priv = cookie;
  int rc;

  /* Decrement the interrupt counter */

  if (priv->intf > 0)
    {
      priv->intf--;
    }

  /* Decode the event */

  rc = sh2_decodeSensorEvent(&priv->value, pEvent);
  if (rc != SH2_OK)
    {
      snerr("ERROR: Failed to decode sensor event (%d)\n", rc);
      return;
    }

  /* Clear the faulty counter because of data has been received */

  priv->faulty = 0;

  /* Notify the poll waiters */

  poll_notify(priv->fds, CONFIG_BNO085_NPOLLWAITERS, POLLIN);

}

/****************************************************************************
 * Name: sh2_reportProdIds
 *
 * Description:
 *   Read product ids with version info from sensor hub and print them
 *
 ****************************************************************************/

static void sh2_reportProdIds(void)
{
  sh2_ProductIds_t prodIds;
  int status;

  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);

  if (status < 0)
    {
      snerr("ERROR: Failed to read from sh2_getProdIds.\n");
      return;
    }

  /* Report the results */
  for (int n = 0; n < prodIds.numEntries; n++)
    {
      sninfo("Part %d : Version %d.%d.%d Build %d\n",
              (int)prodIds.entry[n].swPartNumber,
              (int)prodIds.entry[n].swVersionMajor, (int)prodIds.entry[n].swVersionMinor,
              (int)prodIds.entry[n].swVersionPatch, (int)prodIds.entry[n].swBuildNumber);
    }
}

/****************************************************************************
 * Name: bno085_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int bno085_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bno085_dev_s *priv  = inode->i_private;
  int ret;

  /* Open SH2 interface (also registers non-sensor event handler.) */

  ret = sh2_open(&priv->sh2, sh2_eventHandler, priv);
  if (ret != SH2_OK)
    {
      snerr("ERROR: Failed to open SH2 interface\n");
      return -ENODEV;
    }

  /* Set the initialize flag */

  priv->initialized = 1;

  /* Register sensor listener */

  sh2_setSensorCallback(sh2_sensorHandler, priv);

  /* Read and display device product ids */

  sh2_reportProdIds();

  /* Start the worker */

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, bno085_worker, priv, 0);
      if (ret < 0)
        {
          snerr("ERROR: Failed to queue work: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: bno085_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int bno085_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bno085_dev_s *priv  = inode->i_private;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Close SH2 interface */

  sh2_close();

  /* Clear receive sensor events */

  sh2_setSensorCallback(NULL, NULL);

  /* Clear the initialize flag */

  priv->initialized = 0;

  return OK;
}

/****************************************************************************
 * Name: bno085_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t bno085_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bno085_dev_s *priv  = inode->i_private;
  FAR sh2_SensorValue_t   *data;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Check if enough memory was provided for the read call */

  if (buflen < sizeof(sh2_SensorValue_t))
    {
      snerr("ERROR: "
            "Not enough memory for reading out a sensor data sample\n");
      return -ENOSYS;
    }

  /* Acquire the mutex before the data is copied */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      snerr("ERROR: Could not acquire priv->datalock: %d\n", ret);
      return ret;
    }

  /* Copy the sensor data into the buffer */

  data = (FAR sh2_SensorValue_t *)buffer;
  memcpy(data, &priv->value, sizeof(FAR sh2_SensorValue_t));

  /* Give back the mutex */

  nxmutex_unlock(&priv->lock);

  return sizeof(FAR sh2_SensorValue_t);
}

/****************************************************************************
 * Name: bno085_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t bno085_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bno085_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int bno085_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bno085_dev_s *priv  = inode->i_private;
  int ret = OK;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Acquire the mutex before the data is copied */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      snerr("ERROR: Could not acquire priv->lock: %d\n", ret);
      return ret;
    }

  switch (cmd)
    {
      case SNIOC_RESET:
        {
          /* Reset the status flag and clear the faulty counter */

          priv->status = 0;
          priv->faulty = 0;

          /* Perform a device reset */

          sh2_devReset();

          /* It is important to wait untill reset has been done !!! */
        }
        break;

      case SNIOC_GETSTATUS:
        {
          /* Get the status data pointer */

          int *ptr = (FAR int *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          /* Return the current status flag */

          *ptr = priv->status;
        }
        break;

      case SNIOC_SETCONFIG:
        {
          /* Get the configuration data pointer */

          struct bno085_reports_s *ptr = (FAR struct bno085_reports_s *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);

          /* Set the configuration */

          ret = sh2_setSensorConfig(ptr->sensorId, &ptr->config);
          if (ret != 0)
            {
              snerr("ERROR: Failed to configure sensor (%d)\n", ptr->sensorId);
            }
        }
        break;

      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  /* Give back the mutex */

  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: bno085_poll
 *
 * Description:
 *   This routine is called during BNO055 device poll
 *
 ****************************************************************************/

static int bno085_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bno085_dev_s *priv  = inode->i_private;
  int ret, i;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Acquire the mutex before the data is copied */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      snerr("ERROR: Could not acquire priv->lock: %d\n", ret);
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_BNO085_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_BNO085_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

out:

  /* Give back the mutex */

  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: bno085_register
 *
 * Description:
 *   Register the BNO085 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/imu0"
 *   config  - Configuration of the SPI/I2C interface to use to communicate
 *             with BNO085
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bno085_register(FAR const char *devpath,
                    FAR struct bno085_config_s *config)
{
  FAR struct bno085_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL && config != NULL);

  /* Initialize the BNO085 device structure */

  priv = kmm_zalloc(sizeof(struct bno085_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->flink     = NULL;
  priv->config    = config;
  priv->timestamp = 0;
  priv->intf      = 0;
  priv->status    = 0;
  priv->faulty    = 0;
  priv->work.worker = NULL;
  priv->initialized = 0;

  /* Initialize sensor interface functions */

  priv->sh2.priv  = priv;
#ifdef CONFIG_SENSORS_BNO085_I2C
  priv->sh2.open  = shtp_i2c_hal_open;
  priv->sh2.close = shtp_i2c_hal_close;
  priv->sh2.read  = shtp_i2c_hal_read;
  priv->sh2.write = shtp_i2c_hal_write;
  priv->sh2.getTimeUs = shtp_i2c_hal_getTimeUs;
#else
  snerr("bno085: No such device\n");
  return -ENODEV;
#endif

  /* Initialize sensor data access mutex */

  nxmutex_init(&priv->lock);

  /* Register the character driver */

  ret = register_driver(devpath, &g_bno085fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return ret;
    }

  /* Since we support multiple BNO085 devices, we will need to add this new
   * instance to a list of device instances so that it can be found by the
   * interrupt handler based on the received IRQ number.
   */

  priv->flink = g_bno085_list;
  g_bno085_list = priv;

  /* Attach the interrupt handler */

  if (priv->config->attach != NULL)
    {
      ret = priv->config->attach(priv->config, &bno085_interrupt_handler, priv);
      if (ret < 0)
        {
          snerr("ERROR: Failed to attach interrupt\n");
          nxmutex_destroy(&priv->lock);
          kmm_free(priv);
          return ret;
        }
    }

  return OK;
}
