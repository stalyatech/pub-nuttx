/****************************************************************************
 * drivers/sensors/bno055.c
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
#include <nuttx/sensors/bno055.h>
#ifdef CONFIG_SENSORS_BNO055_I2C
#  include <nuttx/i2c/i2c_master.h>
#else
#  include <nuttx/spi/spi.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BNO055_I2C_RETRIES  (10)
#define BNO055_FAULT_COUNT  (20)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bno055_dev_s
{
  FAR struct bno055_dev_s    *flink;      /* Supports a singly linked list of
                                           * drivers */
  FAR struct bno055_config_s *config;     /* Pointer to the configuration of the BNO055 sensor */
  uint64_t                    timestamp;  /* Units is microseconds */
  mutex_t                     lock;       /* Manages exclusive access to this structure */
  struct bno055_t             devif;      /* Device interface */
  struct bno055_reports_s     report;     /* Sensor report */
  volatile int16_t            intf;       /* Interrupt received but data not read, yet */
  struct work_s               work;       /* The work queue is responsible for
                                           * retrieving the data from the sensor
                                           * after the arrival of new data was
                                           * signalled in an interrupt */
  struct pollfd              *fds[CONFIG_BNO055_NPOLLWAITERS];
  volatile uint8_t            status;     /* Sensor status flag */
  volatile uint32_t           faulty;     /* Sensor fault counter */
  volatile uint8_t            initialized;/* Sensor initialized flag */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bno055_getregs(FAR struct bno055_dev_s *priv, uint8_t regaddr, 
                          FAR uint8_t *regval, int len);
static int bno055_putregs(FAR struct bno055_dev_s *priv, uint8_t regaddr, 
                          FAR uint8_t *regval, int len);
static void bno055_worker(FAR void *arg);

/* Character driver methods */

static int     bno055_open(FAR struct file *filep);
static int     bno055_close(FAR struct file *filep);
static ssize_t bno055_read(FAR struct file *filep,
                           FAR char *buffer, size_t len);
static ssize_t bno055_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     bno055_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int     bno055_poll(FAR struct file *filep,
                           FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_bno085fops =
{
  bno055_open,    /* open */
  bno055_close,   /* close */
  bno055_read,    /* read */
  bno055_write,   /* write */
  NULL,           /* seek */
  bno055_ioctl,   /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  bno055_poll     /* poll */
};

/* Single linked list to store instances of drivers */

static struct bno055_dev_s *g_bno055_list = NULL;

/****************************************************************************
 * hal interface functions
 *
 ****************************************************************************/

/****************************************************************************
 * Name: hal_ll_read
 *
 ****************************************************************************/

static s8 hal_ll_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  struct bno055_dev_s *priv = g_bno055_list;

  return ((s8)bno055_getregs(priv, reg_addr, reg_data, cnt));
}

/****************************************************************************
 * Name: hal_ll_write
 *
 ****************************************************************************/

static s8 hal_ll_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  struct bno055_dev_s *priv = g_bno055_list;

  return ((s8)bno055_putregs(priv, reg_addr, reg_data, cnt));
}

/****************************************************************************
 * Name: hal_ll_delay
 *
 ****************************************************************************/

static void hal_ll_delay(u32 msek)
{
  up_mdelay(msek);
}

/****************************************************************************
 * Name: bno055_getregs
 *
 * Description:
 *   Read (len) bytes from specified device
 *
 ****************************************************************************/

static int bno055_getregs(FAR struct bno055_dev_s *priv, uint8_t regaddr, 
                          FAR uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BNO055_I2C
  struct i2c_msg_s msg[2];
  int retries;
  int ret = -EIO;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = priv->config->devid;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = priv->config->devid;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = len;

  /* Perform the transfer */

  for (retries = 0; retries < BNO055_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->dev, msg, 2);
      if (ret >= 0)
        {
          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == BNO055_I2C_RETRIES - 1)
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
#else /* CONFIG_SENSORS_BNO055_SPI */
  return -EIO;
#endif
}

/****************************************************************************
 * Name: bno055_putregs
 *
 * Description:
 *   Send (len) bytes for specified device
 *
 ****************************************************************************/

static int bno055_putregs(FAR struct bno055_dev_s *priv, uint8_t regaddr, 
                          FAR uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BNO055_I2C
  struct i2c_msg_s msg[2];
  int retries;
  int ret = -EIO;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = priv->config->devid;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = priv->config->devid;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = regval;
  msg[1].length    = len;

  /* Perform the transfer */

  for (retries = 0; retries < BNO055_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->dev, msg, 2);
      if (ret >= 0)
        {
          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == BNO055_I2C_RETRIES - 1)
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
 * Name: bno055_interrupt_handler
 ****************************************************************************/

static int bno055_interrupt_handler(int irq, FAR void *context,
                                    FAR void *arg)
{
  FAR struct bno055_dev_s *priv = arg;
  int ret = OK;

  /*
   * This function should be called upon a falling edge on the BNO055 new data
   * interrupt pin since it signals that new data has been measured.
   */

  /* Check the initialize flag */

  if (priv->initialized)
  {

    /* Find out which BNO055 device caused the interrupt */

    for (priv = g_bno055_list; priv && priv->config->irq != irq;
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
        ret = work_queue(HPWORK, &priv->work, bno055_worker, priv, 0);
        if (ret < 0)
          {
            snerr("ERROR: Failed to queue work: %d\n", ret);
          }
      }
  }

  return ret;
}

/****************************************************************************
 * Name: sensorHandler
 *
 * Description:
 *   Handle sensor events
 *
 ****************************************************************************/

static void sensorHandler(struct bno055_dev_s *priv)
{
  int ret = 0;

  /* Decrement the interrupt counter */

  if (priv->intf > 0)
    {
      priv->intf--;
    }

  /* Check the sensor reading mode */

  switch (priv->config->opr_mode)
    {
      case BNO055_OPMODE_RAW:
        {
          /* Raw accel X, Y and Z data can read from the register */

          ret += bno055_read_accel_xyz(&priv->report.raw.accel);

          /* Raw mag X, Y and Z data can read from the register */

          ret += bno055_read_mag_xyz(&priv->report.raw.mag);

          /* Raw gyro X, Y and Z data can read from the register */

          ret += bno055_read_gyro_xyz(&priv->report.raw.gyro);
        }
        break;

      case BNO055_OPMODE_FUSION:
        {
          /* Raw Euler H, R and P data can read from the register */

          ret += bno055_read_euler_hrp(&priv->report.fusion.euler);

          /* Raw Quaternion W, X, Y and Z data can read from the register */

          ret += bno055_read_quaternion_wxyz(&priv->report.fusion.quat);
        }
        break;
    }


  if (ret != 0)
    {
      /* Set the faulty status */

      if (++priv->faulty >= BNO055_FAULT_COUNT)
        {
          priv->status |= BNO055_STATUS_FAULT;
        }
    }
  else
    {
      /* Update the operation mode of report */

      priv->report.opr_mode = priv->config->opr_mode;

      /* Clear the faulty counter because of data has been received */

      priv->faulty = 0;

      /* Notify the poll waiters */

      poll_notify(priv->fds, CONFIG_BNO055_NPOLLWAITERS, POLLIN);
    }
}

/****************************************************************************
 * Name: bno055_worker
 ****************************************************************************/

static void bno055_worker(FAR void *arg)
{
  FAR struct bno055_dev_s *priv = arg;
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
      /* Sensor handler */

      sensorHandler(priv);

      /* Re-schedule the worker (100Hz) */

      work_queue(HPWORK, &priv->work, bno055_worker, priv, USEC2TICK(10000));
    }

  /* Give back the mutex */

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: bno055_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int bno055_open(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bno055_dev_s *priv  = inode->i_private;
  int ret;

  /* Initialize the sensor */

  ret = bno055_init(&priv->devif);
  if (ret != OK)
    {
      snerr("ERROR: Failed to open bno055 interface\n");
      return -ENODEV;
    }

  /* Set the initialize flag */

  priv->initialized = 1;

  /* Set the power mode */
  
  ret = bno055_set_power_mode(priv->config->pwr_mode);
  if (ret != OK)
    {
      snerr("ERROR: Failed to set bno055 power mode\n");
      return -EPERM;
    }

  /* Set the operation mode */

  ret = bno055_set_operation_mode(priv->config->opr_mode);
  if (ret != OK)
    {
      snerr("ERROR: Failed to set bno055 operation mode\n");
      return -EPERM;
    }

  /* Start the worker */

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, bno055_worker, priv, 0);
      if (ret < 0)
        {
          snerr("ERROR: Failed to queue work: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: bno055_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int bno055_close(FAR struct file *filep)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bno055_dev_s *priv  = inode->i_private;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Close the interface */

  /* Clear receive sensor events */

  /* Clear the initialize flag */

  priv->initialized = 0;

  return OK;
}

/****************************************************************************
 * Name: bno055_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t bno055_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bno055_dev_s *priv  = inode->i_private;
  FAR struct bno055_reports_s *data;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Check if enough memory was provided for the read call */

  if (buflen < sizeof(struct bno055_reports_s))
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

  data = (FAR struct bno055_reports_s *)buffer;
  memcpy(data, &priv->report, sizeof(FAR struct bno055_reports_s));

  /* Give back the mutex */

  nxmutex_unlock(&priv->lock);

  return sizeof(FAR struct bno055_reports_s);
}

/****************************************************************************
 * Name: bno055_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t bno055_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: bno055_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int bno055_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bno055_dev_s *priv  = inode->i_private;
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

      case SNIOC_SETPOWERMODE:
        {
          /* Get the power mode data pointer */

          int *power_mode = (FAR int *)((uintptr_t)arg);
          DEBUGASSERT(power_mode != NULL);

          /* Set the sensor power mode  */

          ret = bno055_set_power_mode(*power_mode);
          if (ret == OK)
            {
              priv->config->pwr_mode = *power_mode;    
            }
          else
            {
              snerr("ERROR: Failed to set sensor power mode (%d)\n", *power_mode);
            }
        }
        break;

      case SNIOC_SETOPERMODE:
        {
          /* Get the operation mode data pointer */

          int *oper_mode = (FAR int *)((uintptr_t)arg);
          DEBUGASSERT(oper_mode != NULL);

          /* Set the sensor operation mode  */

          ret = bno055_set_operation_mode(*oper_mode);
          if (ret == OK)
            {
              priv->config->opr_mode = *oper_mode;    
            }
          else
            {
              snerr("ERROR: Failed to set sensor operation mode (%d)\n", *oper_mode);
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
 * Name: bno055_poll
 *
 * Description:
 *   This routine is called during BNO055 device poll
 *
 ****************************************************************************/

static int bno055_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bno055_dev_s *priv  = inode->i_private;
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

      for (i = 0; i < CONFIG_BNO055_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_BNO055_NPOLLWAITERS)
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
 * Name: bno055_register
 *
 * Description:
 *   Register the BNO055 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/imu0"
 *   config  - Configuration of the SPI/I2C interface to use to communicate
 *             with BNO055
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno report on failure.
 *
 ****************************************************************************/

int bno055_register(FAR const char *devpath,
                    FAR struct bno055_config_s *config)
{
  FAR struct bno055_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL && config != NULL);

  /* Initialize the BNO055 device structure */

  priv = kmm_zalloc(sizeof(struct bno055_dev_s));
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

  priv->devif.priv       = priv;
#ifdef CONFIG_SENSORS_BNO055_I2C
  priv->devif.dev_addr   = config->devid;
  priv->devif.bus_read   = hal_ll_read;
  priv->devif.bus_write  = hal_ll_write;
  priv->devif.delay_msec = hal_ll_delay;
#else
  snerr("bno055: No such device\n");
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

  /* Since we support multiple BNO055 devices, we will need to add this new
   * instance to a list of device instances so that it can be found by the
   * interrupt handler based on the received IRQ number.
   */

  priv->flink = g_bno055_list;
  g_bno055_list = priv;

  /* Attach the interrupt handler */

  if (priv->config->attach != NULL)
    {
      ret = priv->config->attach(priv->config, &bno055_interrupt_handler, priv);
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
