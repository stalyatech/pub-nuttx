/****************************************************************************
 * drivers/sensors/ublox_gps_uorb.c
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

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <termios.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/gps.h>
#include <nuttx/sensors/ublox_gps.h>
#include <nuttx/serial/tioctl.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ublox_gps_s
{
  FAR struct sensor_lowerhalf_s lower;
  struct file   dev;        /* Serial port device */
  uint32_t      baud;       /* Baudrate of serial port */
  unsigned long interval;   /* Polling interval */
  bool          enabled;    /* Enable/Disable device */
  sem_t         run;        /* Locks measure cycle */
  mutex_t       lock;       /* Manages exclusive to device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor methods */

static int ublox_gps_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable);
static int ublox_gps_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR unsigned long *period_us);
static int ublox_gps_control(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             int cmd, unsigned long arg);

static int ublox_gps_thread(int argc, FAR char** argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_ublox_gps_ops =
{
  .activate     = ublox_gps_activate,
  .set_interval = ublox_gps_set_interval,
  .control      = ublox_gps_control,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ublox_gps_open
 ****************************************************************************/

static inline int ublox_gps_open(FAR struct file *filep,
                                 FAR const char *name,
                                 uint32_t baud,
                                 uint32_t flags)
{
  struct termios opt;
  int ret;

  ret = file_open(filep, name, flags);
  if (ret < 0)
    {
      snerr("Could not open '%s': %s", name, strerror(-ret));
      return ret;
    }

  file_ioctl(filep, TCGETS, &opt);
  cfmakeraw(&opt);

  /* Configure a baud rate.
   * NuttX doesn't support different baud rates for RX and TX.
   * So, both cfisetospeed() and cfisetispeed() are overwritten
   * by cfsetspeed.
   */
#ifdef CONFIG_SERIAL_TERMIOS
  switch (baud)
    {
      case 921600: 
        baud = B921600;
        break;

      case 460800: 
        baud = B460800;
        break;

      case 230400: 
        baud = B230400;
        break;

      case 115200: 
        baud = B115200;
        break;

      case 57600: 
        baud = B57600;
        break;

      case 38400: 
        baud = B38400;
        break;

      case 19200: 
        baud = B19200;
        break;

      case 9600: 
        baud = B9600;
        break;

      default: 
        baud = B38400;
    }

  cfsetspeed(&opt, baud);

  /* Change the attributes now. */

  file_ioctl(filep, TCSETS, &opt);
#endif

  return OK;
}

/****************************************************************************
 * Name: ublox_gps_enable
 ****************************************************************************/

static int ublox_gps_enable(struct ublox_gps_s *priv, bool enable)
{
  return OK;
}

/****************************************************************************
 * Name: ublox_gps_activate
 ****************************************************************************/

static int ublox_gps_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable)
{
  bool start_thread = false;
  FAR struct ublox_gps_s *priv = container_of(lower, 
                                              struct ublox_gps_s, 
                                              lower);
  if (enable)
    {
      if (!priv->enabled)
        {
          start_thread = true;
        }
    }

  priv->enabled = enable;

  if (start_thread)
    {
      /* Wake up the thread */

      nxsem_post(&priv->run);
    }

  return OK;
}

/****************************************************************************
 * Name: ublox_gps_set_interval
 ****************************************************************************/

static int ublox_gps_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR unsigned long *period_us)
{
  FAR struct ublox_gps_s *priv = container_of(lower, 
                                              struct ublox_gps_s, 
                                              lower);
  bool running = priv->enabled;

  /* GNSS must be disabled when fix interval change */

  if (running == true)
    {
      ublox_gps_enable(priv, false);
    }

  /* update interval in microseconds */

  priv->interval = *period_us;

  /* TODO */

  if (running == true)
    {
      ublox_gps_enable(priv, true);
    }

  return OK;
}

/****************************************************************************
 * Name: ublox_gps_control
 ****************************************************************************/

static int ublox_gps_control(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             int cmd, unsigned long arg)
{
  /* TODO */

  return OK;
}

/****************************************************************************
 * Name: ublox_gps_thread
 ****************************************************************************/

static int ublox_gps_thread(int argc, FAR char** argv)
{
  FAR struct ublox_gps_s *priv = (FAR struct ublox_gps_s *)
                                ((uintptr_t)strtoul(argv[1], NULL, 0));
  struct sensor_gps_raw gps_data;
  int ret;
  char ch;

  while (true)
    {
      if (!priv->enabled)
        {
          /* Waiting to be woken up */

          ret = nxsem_wait(&priv->run);
          if (ret < 0)
            {
              continue;
            }
        }


      /* Read until we complete a line */

      gps_data.len = 0;
      do
        {
          ret = file_read(&priv->dev, &ch, 1);
          if (ret == 1)
            {
              gps_data.buf[gps_data.len++] = ch;
            }
        }
      while (ch != '\n');
      gps_data.buf[gps_data.len] = '\0';

      /* Update the sensor event */

      if (gps_data.len > 0)
        {
          priv->lower.push_event(priv->lower.priv, &gps_data, sizeof(struct sensor_gps_raw));
        }

      /* Sleeping thread before fetching the next sensor data */

      nxsig_usleep(priv->interval);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ublox_gps_register
 *
 * Description:
 *   Register the u-blox GPS character device.
 *
 * Input Parameters:
 *   dev     - Serial port name that connected to the GPS device.
 *   baud    - Serial baudrate of the GPS device.
 *   devno   - The user specifies which device of this type, from 0.
 *   nbuffer - The number of events that the circular buffer can hold.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ublox_gps_register(FAR const char *devname, uint32_t baud, uint32_t devno, uint32_t nbuffer)
{
  FAR struct ublox_gps_s *priv;
  FAR char *argv[2];
  char arg1[32];
  char name[32];
  int ret;

  /* Initialize the GPS device structure */

  priv = kmm_zalloc(sizeof(struct ublox_gps_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Open the serial port device */

  ret = ublox_gps_open(&priv->dev, devname, baud, O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      snerr("Failed to open GPS device serial port\n");
      kmm_free(priv);
      return ret;
    }
 
  /* Default values */

  priv->interval = 1000000; 
  nxsem_init(&priv->run, 0, 0);
  nxmutex_init(&priv->lock);

  priv->lower.ops = &g_ublox_gps_ops;
  priv->lower.type = SENSOR_TYPE_GPS_RAW;
  priv->lower.nbuffer = nbuffer;

  /* Register the character driver */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      file_close(&priv->dev);
      nxmutex_destroy(&priv->lock);
      nxsem_destroy(&priv->run);
      kmm_free(priv);
      return ret;
    }

  /* Create thread for polling sensor data */

  snprintf(name, sizeof(name), "sensor_gps_raw%lu", devno);
  snprintf(arg1, 32, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create(name,
                       50,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       ublox_gps_thread, argv);
  if (ret < 0)
    {
      snerr("Failed to create the notification kthread!\n");
      file_close(&priv->dev);
      sensor_unregister(&priv->lower, devno);
      nxmutex_destroy(&priv->lock);
      nxsem_destroy(&priv->run);
      kmm_free(priv);
      return ret;
    }

  sninfo("u-blox GPS driver loaded successfully!\n");
  return OK;
}
