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
  struct gps_lowerhalf_s gps;
  struct file dev;
  bool running;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ublox_gps_activate(FAR struct gps_lowerhalf_s *lower,
                              FAR struct file *filep, 
                              bool enabled);
static int ublox_gps_set_interval(FAR struct gps_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  unsigned long *period_us);
static int ublox_gps_control(FAR struct gps_lowerhalf_s *lower,
                             FAR struct file *filep,
                             int cmd, 
                             unsigned long arg);
static ssize_t ublox_gps_inject_data(FAR struct gps_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     const void *buffer, 
                                     size_t buflen);

static int ublox_gps_thread(int argc, FAR char** argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gps_ops_s g_ublox_gps_ops =
{
  .activate     = ublox_gps_activate,
  .set_interval = ublox_gps_set_interval,
  .control      = ublox_gps_control,
  .inject_data  = ublox_gps_inject_data,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ublox_gps_write
 ****************************************************************************/

static inline int ublox_gps_write(FAR struct file *filep,
                                  FAR const void *buffer,
                                  size_t size)
{
  FAR const char *p = (FAR const char *)buffer;

  while (size > 0)
    {
      int n = file_write(filep, p, size);
      if (n < 0)
        {
          return n;
        }

      p += n;
      size -= n;
    }

  return 0;
}

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

static int ublox_gps_activate(FAR struct gps_lowerhalf_s *gps,
                              FAR struct file *filep,
                              bool enabled)
{
  FAR struct ublox_gps_s *priv =
    container_of(gps, struct ublox_gps_s, gps);
  priv->running = enabled;

  return OK;
}

/****************************************************************************
 * Name: ublox_gps_set_interval
 ****************************************************************************/

static int ublox_gps_set_interval(FAR struct gps_lowerhalf_s *gps,
                                  FAR struct file *filep,
                                  unsigned long *period_us)
{
  FAR struct ublox_gps_s *priv =
    container_of(gps, struct ublox_gps_s, gps);
  uint16_t fix_interval = 0;
  int      ret          = OK;
  bool     running      = priv->running;

  /* GNSS must be disabled when fix interval change */

  if (running == true)
    {
      ublox_gps_enable(priv, false);
    }

  /* Fix interval in seconds */

  fix_interval = (*period_us) / 1000000;

  /* Handle GNSS mode */

  if (fix_interval == 1)
    {
      /* Continuous navigation with 1 Hz rate */
    }
  else if (fix_interval == 0)
    {
      /* Single fix */

    }
  else if (fix_interval < 10)
    {
      /* Periodic navigation, minimum interval is 10s */

      fix_interval = 10;
      *period_us   = fix_interval * 1000000;
    }
  else if (fix_interval > 65535)
    {
      /* Periodic navigation, maximum interval is 65535s */

      fix_interval = 65535;
      *period_us   = fix_interval * 1000000;
    }

  /* TODO */
  /* Inverval set */

  if (running == true)
    {
      ublox_gps_enable(priv, true);
    }

  return ret;
}

/****************************************************************************
 * Name: ublox_gps_control
 ****************************************************************************/

static int ublox_gps_control(FAR struct gps_lowerhalf_s *gps,
                             FAR struct file *filep,
                             int cmd, 
                             unsigned long arg)
{
  /* TODO */

  return OK;
}

/****************************************************************************
 * Name: ublox_gps_inject_data
 ****************************************************************************/

static ssize_t ublox_gps_inject_data(FAR struct gps_lowerhalf_s *gps,
                                     FAR struct file *filep,
                                     const void *buffer, 
                                     size_t buflen)
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
  ssize_t len;
  char buf[256];

  while (true)
    {
      len = file_read(&priv->dev, buf, sizeof(buf));
      if (priv->running && len > 0)
        {
          priv->gps.push_data(priv->gps.priv, buf, len, true);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ublox_gps_init
 *
 * Description:
 *   u-blox GPS driver entrypoint.
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

int ublox_gps_init(FAR const char *devname, uint32_t baud, uint32_t devno, uint32_t nbuffer)
{
  FAR struct ublox_gps_s *gps;
  FAR char *argv[2];
  char arg1[32];
  char name[32];
  int ret;

  /* Alloc memory for sensor */

  gps = kmm_zalloc(sizeof(struct ublox_gps_s));
  if (!gps)
    {
      return -ENOMEM;
    }

  ret = ublox_gps_open(&gps->dev, devname, baud, O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      kmm_free(gps);
      return ret;
    }
 
  /* Create thread for sensor */

  snprintf(name, sizeof(name), "sensor_gps%lu_thread", devno);
  snprintf(arg1, 32, "%p", gps);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create(name,
                       50,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       ublox_gps_thread, argv);
  if (ret < 0)
    {
      file_close(&gps->dev);
      kmm_free(gps);
      return ret;
    }

  /*  Register sensor */

  gps->gps.ops = &g_ublox_gps_ops;

  return gps_register(&gps->gps, devno, nbuffer);
}
