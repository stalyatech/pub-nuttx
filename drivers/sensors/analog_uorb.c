/****************************************************************************
 * drivers/sensors/analog_uorb.c
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

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/sensors/sensor.h>

#if defined(CONFIG_SENSORS_ANALOG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Internal sensor data */

struct analog_sensor_data_s
{
  uint64_t timestamp;
  struct adc_msg_s sample[CONFIG_SENSORS_ANALOG_GROUPSIZE];
};

/* Lower sensor */

struct analog_priv_s
{
  struct sensor_lowerhalf_s   lower;  /* Common lower interface */
  struct analog_sensor_data_s data;   /* Sensor data */
  struct file dev;                    /* Sensor device */
#ifdef CONFIG_SENSORS_ANALOG_POLL
  uint32_t    interval;               /* Polling interval */
  bool        enabled;                /* Sensor activated */
  sem_t       run;                    /* Locks sensor thread */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor functions */

static int analog_active(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         bool enabled);

static int analog_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen);

static int analog_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          int cmd, unsigned long arg);

#ifdef CONFIG_SENSORS_ANALOG_POLL
static int analog_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us);

static int analog_thread(int argc, FAR char** argv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_analog_ops =
{
#ifdef CONFIG_SENSORS_ANALOG_POLL
  .set_interval = analog_set_interval,
#endif
  .activate     = analog_active,
  .fetch        = analog_fetch,
  .control      = analog_control
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: analog_dev_open
 ****************************************************************************/

static inline int analog_dev_open(FAR struct file *filep,
                                  FAR const char *name,
                                  uint32_t flags)
{
  int ret;

  /* Open the device */

  ret = file_open(filep, name, flags);
  if (ret < 0)
    {
      snerr("Could not open '%s': %s", name, strerror(-ret));
      return ret;
    }

#ifdef CONFIG_SENSORS_ANALOG_SWTRIG
  /* Issue the software trigger to start ADC conversion */

  file_ioctl(filep, ANIOC_TRIGGER, 0);
#endif /* CONFIG_SENSORS_ANALOG_SWTRIG */

  return OK;
}

/****************************************************************************
 * Name: analog_read
 *
 * Description: Perform a measurement and read last measured values
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int analog_read(FAR struct analog_priv_s *priv)
{
  /* Read up to samples */

  size_t readsize = sizeof(priv->data.sample);
  ssize_t nbytes = file_read(&priv->dev, priv->data.sample, readsize);

  /* Handle unexpected return values */

  if (nbytes < 0)
    {
      if (errno != EINTR)
        {
          return -EIO;
        }
    }

  else if (nbytes != readsize)
    {
      return -EIO;
    }

  /* Save the time stamp */

  priv->data.timestamp = sensor_get_timestamp();

#ifdef CONFIG_SENSORS_ANALOG_SWTRIG
  /* Issue the software trigger to start ADC conversion */

  file_ioctl(&priv->dev, ANIOC_TRIGGER, 0);
#endif /* CONFIG_SENSORS_ANALOG_SWTRIG */

  return OK;
}

/****************************************************************************
 * Name: analog_notify
 *
 * Description:
 *   Notify upper about data has been changed.
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ANALOG_POLL
static void analog_notify(FAR struct analog_priv_s *priv)
{
  struct sensor_analog sensor;

  sensor.vbat = priv->data.sample[0].am_data;
  sensor.vrev = priv->data.sample[1].am_data;
  sensor.timestamp = priv->data.timestamp;
  priv->lower.push_event(priv->lower.priv, &sensor, sizeof(struct sensor_analog));
}
#endif

/****************************************************************************
 * Name: analog_fetch
 *
 * Description: Performs a measuremnt cylce and data read with data
 *              conversion.
 *
 * Parameter:
 *   lower  - Pointer to lower half sensor driver instance.
 *   filep  - The pointer of file, represents each user using the sensor.
 *   buffer - Pointer to the buffer for reading data.
 *   buflen - Size of the buffer.
 *
 * Return:
 *   OK - on success
 *
 ****************************************************************************/

static int analog_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen)
{
  FAR struct analog_priv_s *priv = container_of(lower,
                                                struct analog_priv_s,
                                                lower);

  /* Check if the user is reading the right size */

  if (buflen != sizeof(struct sensor_analog))
    {
      snerr("ERROR: You need to read %d bytes from this sensor!\n",
            sizeof(struct sensor_analog));
      return -EINVAL;
    }

  /* Update the user buffer */

  FAR struct sensor_analog *sensor =
    (FAR struct sensor_analog *)buffer;
  
  sensor->vbat = priv->data.sample[0].am_data;
  sensor->vrev = priv->data.sample[1].am_data;
  sensor->timestamp = priv->data.timestamp;

  return buflen;
}

/****************************************************************************
 * Name: analog_control
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int analog_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          int cmd, unsigned long arg)
{
  FAR struct analog_priv_s *priv = container_of(lower,
                                                struct analog_priv_s,
                                                lower);
  int ret = OK;
  UNUSED(priv);

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
 * Name: analog_active
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int analog_active(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         bool enabled)
{
#ifdef CONFIG_SENSORS_ANALOG_POLL
  FAR struct analog_priv_s *priv = container_of(lower,
                                                struct analog_priv_s,
                                                lower);
  bool start_thread = false;

  if (enabled)
    {
      if (!priv->enabled)
        {
          start_thread = true;
        }
    }

  priv->enabled = enabled;

  if (start_thread == true)
    {
      /* Wake up the thread */

      nxsem_post(&priv->run);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: analog_set_interval
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ANALOG_POLL
static int analog_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us)
{
  FAR struct analog_priv_s *priv = container_of(lower,
                                                struct analog_priv_s,
                                                lower);

  /* update interval in microseconds */

  priv->interval = *period_us;
  return OK;
}
#endif

/****************************************************************************
 * Name: analog_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number of arguments
 *   argv - Pointer to argument list
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ANALOG_POLL
static int analog_thread(int argc, char** argv)
{
  FAR struct analog_priv_s *priv = (FAR struct analog_priv_s *)
                                    ((uintptr_t)strtoul(argv[1], NULL, 16));

  while (true)
    {
      int ret;

      if (!priv->enabled)
        {
          /* Waiting to be woken up */

          nxsem_wait(&priv->run);
        }

        /* Read the device */

        ret = analog_read(priv);
        if (!ret)
          {
            if (priv->enabled == true)
              {

                /* Default nofitication when value has been read */
                
                analog_notify(priv);
              }
          }

      /* Sleeping thread before fetching the next sensor data */

      nxsig_usleep(priv->interval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: analog_uorb_register
 *
 * Description:
 *   Register the on-board analog sensor device.
 *
 * Input Parameters:
 *   devno   - The user specifies device number, from 0.
 *   nbuffer - The number of events that the circular buffer can hold.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 ****************************************************************************/

int analog_uorb_register(uint32_t devno, uint32_t nbuffer)
{
  FAR struct analog_priv_s *priv;
#ifdef CONFIG_SENSORS_ANALOG_POLL
  FAR char *argv[2];
  char arg1[32];
#endif
  char name[32];
  int ret;

  /* Initialize the analog sensor device structure */

  priv = kmm_zalloc(sizeof(struct analog_priv_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Open the on-board device */

  ret = analog_dev_open(&priv->dev, CONFIG_SENSORS_ANALOG_DEVPATH, O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      snerr("ERROR: Failed to open on-board analog device\n");
      kmm_free(priv);
      return ret;
    }

  /* Default values */

#ifdef CONFIG_SENSORS_ANALOG_POLL
  priv->interval = CONFIG_SENSORS_ANALOG_POLL_INTERVAL;
  nxsem_init(&priv->run, 0, 0);
#endif

  /* ADC register */

#ifdef CONFIG_SENSORS_ANALOG_POLL
  priv->enabled = false;
#endif
  priv->lower.ops = &g_analog_ops;
  priv->lower.type = SENSOR_TYPE_ANALOG;
  priv->lower.uncalibrated = false;
  priv->lower.nbuffer = nbuffer;

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      goto sensor_err;
    }

#ifdef CONFIG_SENSORS_ANALOG_POLL
  /* Create thread for polling sensor data */

  snprintf(name, sizeof(name), "sensor_analog%lu", devno);
  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create(name,
                       SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_ANALOG_THREAD_STACKSIZE,
                       analog_thread, argv);
  if (ret > 0)
#endif
    {
      sninfo("Analog uORB driver loaded successfully!\n");
      return OK;
    }

sensor_err:
  file_close(&priv->dev);
#ifdef CONFIG_SENSORS_ANALOG_POLL
  nxsem_destroy(&priv->run);
#endif
  sensor_unregister(&priv->lower, devno);
  kmm_free(priv);
  return ret;
}
#endif /* CONFIG_SENSORS_ANALOG */