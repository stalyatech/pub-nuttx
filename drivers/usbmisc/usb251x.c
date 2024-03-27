/****************************************************************************
 * drivers/usbmisc/usb251x.c
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
#include <nuttx/crc32.h>

#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/usb/usb251x.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_USB251X
#  define usb251x_err(x, ...)       _err(x, ##__VA_ARGS__)
#  define usb251x_info(x, ...)      _info(x, ##__VA_ARGS__)
#else
#  define usb251x_err(x, ...)       uerr(x, ##__VA_ARGS__)
#  define usb251x_info(x, ...)      uinfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_USB251X_I2C_FREQ
#  define CONFIG_USB251X_I2C_FREQ   100000
#endif

/* Other macros */

#define USB251X_I2C_RETRIES   10

#define USB251X_MFR_LEN_OFS	  (0x13)
#define USB251X_PRD_LEN_OFS	  (0x14)
#define USB251X_SER_LEN_OFS	  (0x15)
#define USB251X_MFR_STR_OFS	  (0x16)
#define USB251X_PRD_STR_OFS	  (0x54)
#define USB251X_SER_STR_OFS	  (0x92)

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct usb251x_dev_s
{
  FAR struct usb251x_config_s *config;  /* Platform specific configuration */
  mutex_t devlock;                      /* Manages exclusive access */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static int usb251x_open(FAR struct file *filep);
static int usb251x_close(FAR struct file *filep);
static int usb251x_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int usb251x_config_ports(struct usb251x_dev_s *priv);
static int usb251x_attach(FAR struct usb251x_dev_s *priv);
static int usb251x_detach(FAR struct usb251x_dev_s *priv);
static int usb251x_reset(FAR struct usb251x_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_usb251xops =
{
  usb251x_open,  /* open */
  usb251x_close, /* close */
  NULL,          /* read */
  NULL,          /* write */
  NULL,          /* seek */
  usb251x_ioctl, /* ioctl */
  NULL,          /* mmap */
  NULL,          /* truncate */
  NULL           /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*******************************************************************************
* Function Name  : create_utf16_str
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static int create_utf16_str(const char *inp_str, char *utf16_str)
{
  DEBUGASSERT(inp_str != NULL && utf16_str != NULL);

  /* get the string length */

  int slen = strlen(inp_str);

  /* convert the string value to the UTF16_LE */

  for (int i=0; i<slen; i++)
    {
      utf16_str[2*i+0] = inp_str[i];
      utf16_str[2*i+1] = 0;
    }

  return (2*slen);
}

/*******************************************************************************
* Function Name  : insert_mfr_str
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void insert_mfr_str(struct usb251x_config_s *config)
{
  char utf16_str[62] = {0};

  DEBUGASSERT(config != NULL);

  /* create the utf16 data */

  int len = create_utf16_str(config->mfr_str, utf16_str);

  /* update the config */

  config->config[USB251X_MFR_LEN_OFS] = len/2;

  memcpy(&config->config[USB251X_MFR_STR_OFS], utf16_str, len);
}

/*******************************************************************************
* Function Name  : insert_prd_str
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void insert_prd_str(struct usb251x_config_s *config)
{
  char utf16_str[62] = {0};

  DEBUGASSERT(config != NULL);

  /* create the utf16 data */

  int len = create_utf16_str(config->prd_str, utf16_str);

  /* update the config */

  config->config[USB251X_PRD_LEN_OFS] = len/2;

  memcpy(&config->config[USB251X_PRD_STR_OFS], utf16_str, len);
}

/*******************************************************************************
* Function Name  : insert_ser_str
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void insert_ser_str(struct usb251x_config_s *config)
{
  char utf16_str[62] = {0};

  DEBUGASSERT(config != NULL);

  /* create the utf16 data */

  int len = create_utf16_str(config->ser_str, utf16_str);

  /* update the config */

  config->config[USB251X_SER_LEN_OFS] = len/2;

  memcpy(&config->config[USB251X_SER_STR_OFS], utf16_str, len);
}

/****************************************************************************
 * Name: usb251x_block_read
 *
 * Description:
 *   Read register block from USB251X
 *
 * Input Parameters:
 *   priv    - pointer to USB251X Private Structure
 *   regaddr - register address to read
 *   regvals - register values to read
 *   len     - length of read values
 *
 * Returned Value:
 *   Returns OK in case of success, otherwise ERROR
 ****************************************************************************/

static int usb251x_block_read(FAR struct usb251x_dev_s *priv, uint8_t regaddr, uint8_t *regvals, uint8_t len)
{
  uint8_t rxbuffer[CONFIG_USB251X_BLOCK_SIZE+1];
  struct i2c_msg_s msg[2];
  int retries;
  int ret = -EIO;

  DEBUGASSERT(priv != NULL && priv->config != NULL && regvals != NULL);

  /* Format two messages: The first is a write which is never terminated
   * with STOP condition.
   */

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = priv->config->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  /* The second is either a read (rbuflen > 0) with a repeated start or a
   * write (rbuflen < 0) with no restart.
   */

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = priv->config->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = rxbuffer;
  msg[1].length    = len + 1;

  /* Perform the transfer */

  for (retries = 0; retries < USB251X_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->i2c, msg, 2);
      if (ret >= 0)
        {
          memcpy(regvals, &rxbuffer[1], len);
          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == USB251X_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->config->i2c);
          if (ret < 0)
            {
              usb251x_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  usb251x_info("reg:%02X, error:%d\n", regaddr, ret);
  return ret;
}

/****************************************************************************
 * Name: usb251x_block_write
 *
 * Description:
 *   Write a value to an 8-bit USB251X register
 *
 * Input Parameters:
 *   priv    - pointer to USB251X Private Structure
 *   regaddr - register address to write
 *   regvals - register values to write
 *   len     - length of write values
 *
 * Returned Value:
 *   Returns OK in case of success, otherwise ERROR
 ****************************************************************************/

static int usb251x_block_write(FAR struct usb251x_dev_s *priv, uint8_t regaddr, uint8_t *regvals, uint8_t len)
{
  uint8_t txbuffer[CONFIG_USB251X_BLOCK_SIZE+2];
  struct i2c_msg_s msg;
  int retries;
  int ret = -EIO;

  DEBUGASSERT(priv != NULL && priv->config != NULL && regvals != NULL);

  /* Setup to the data to be transferred (register address and values). */

  txbuffer[0] = regaddr;
  txbuffer[1] = len;
  memcpy(&txbuffer[2], regvals, len);

  /* Set up the I2C configuration */

  msg.frequency = priv->config->freq;
  msg.addr      = priv->config->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = len + 2;

  /* Perform the transfer */

  for (retries = 0; retries < USB251X_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->i2c, &msg, 1);
      if (ret == OK)
        {
          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == USB251X_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->config->i2c);
          if (ret < 0)
            {
              usb251x_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  usb251x_err("ERROR: failed reg:%02X, error:%d\n", regaddr, ret);
  return ret;
}

/*******************************************************************************
* Function Name  : usb251x_portmap
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static int usb251x_portmap(struct usb251x_dev_s *priv, enum usb251x_ports_e phyport, uint8_t logport, bool stat)
{
  uint8_t port_map[4];
  int ret;

  /* read the current port remap status */

  ret = usb251x_block_read(priv, USB251X_PRTR12_REG, port_map, 4);
  if (ret < 0)
    {
      usb251x_err("ERROR: USB251X port(%d) config failed: %d\n", phyport, ret);
      return ret;
    }

  /* check the physical port number */
  switch ((uint8_t)phyport)
  {
    case 1:
      port_map[0] &= 0xF0;
      if (stat) {
          port_map[0] |= logport;
      }
      break;
        
    case 2:
      port_map[0] &= 0x0F;
      if (stat) {
          port_map[0] |= (logport << 4);
      }
      break;

    case 3:
      port_map[1] &= 0xF0;
      if (stat) {
          port_map[1] |= logport;
      }
      break;
        
    case 4:
      port_map[1] &= 0x0F;
      if (stat) {
          port_map[1] |= (logport << 4);
      }
      break;

    case 5:
      port_map[2] &= 0xF0;
      if (stat) {
          port_map[2] |= logport;
      }
      break;
        
    case 6:
      port_map[2] &= 0x0F;
      if (stat) {
          port_map[2] |= (logport << 4);
      }
      break;

    case 7:
      port_map[3] &= 0xF0;
      if (stat) {
          port_map[3] |= logport;
      }
      break;
  }

  /* write the new port remap status */
  
  ret = usb251x_block_write(priv, USB251X_PRTR12_REG, port_map, 4);
  if (ret < 0)
    {
      usb251x_err("ERROR: USB251X port(%d) config failed: %d\n", phyport, ret);
      return ret;
    }

  return OK;
}

/*******************************************************************************
* Function Name  : usb251x_config_ports
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static int usb251x_config_ports(struct usb251x_dev_s *priv)
{
    int ret = 0;
    int phy, log;

    /* Is the port mapping enabled ?*/

    if (priv->config->portmap != NULL) 
      {
        /* Scan all physical ports */

        for (phy = 1; phy < priv->config->portnum + 1; phy++)
          {
            /* Get the logical port number */

            log = priv->config->portmap(phy);
            if (log > 0)
              {
                /* Map and enable the port */

                ret += usb251x_portmap(priv, phy, log, true);
              }
          }
      }
    else 
      {
        /* Scan all physical ports */

        for (phy = 1; phy < priv->config->portnum + 1; phy++)
          {
            /* Map and enable the port with default order */

            ret += usb251x_portmap(priv, phy, phy, true);
          }
      }

    return ret;
}

/****************************************************************************
 * Name: usb251x_config
 *
 * Description:
 *   Setup USB251X chip
 *
 ****************************************************************************/

static int usb251x_config(FAR struct usb251x_dev_s *priv,
                         struct usb251x_config_s *config)
{
  uint8_t buf[CONFIG_USB251X_BLOCK_SIZE];
	uint32_t crc1 = 0xffffffff;
	uint32_t crc2 = 0xffffffff;
  uint8_t i, addr, *cfg;
  int ret = ERROR;

  /* reset the hub */

  /* update the configuration word with custom info */

  insert_mfr_str(config);
  insert_prd_str(config);
  insert_ser_str(config);
  up_mdelay(100);

  /* update the configuration */
  
  for (i = 0; i < 8; i++)
    {
        /* update the register address */
        addr = i * CONFIG_USB251X_BLOCK_SIZE; 

        /* get the configuration part */
        cfg = &config->config[addr];
        memcpy(buf, cfg, CONFIG_USB251X_BLOCK_SIZE);

        /* write configuration block */

        ret = usb251x_block_write(priv, addr, cfg, CONFIG_USB251X_BLOCK_SIZE);
        if (ret < 0)
          {
            usb251x_err("ERROR: Failed to write register(s)\n");
            goto err_out;
          }
        
        /* calculate the crc value of configuration part */

        crc1 = crc32part(cfg, CONFIG_USB251X_BLOCK_SIZE, crc1);
    }

  /* read back the configuration */

  for (i = 0; i < 8; i++)
    {
        /* update the register address */
        addr = i * CONFIG_USB251X_BLOCK_SIZE; 

        /* read configuration block */

        ret = usb251x_block_read(priv, addr, buf, CONFIG_USB251X_BLOCK_SIZE);
        if (ret < 0)
          {
            usb251x_err("ERROR: Failed to write register(s)\n");
            goto err_out;
          } 

        /* calculate the crc value of configuration part */
        
        crc2 = crc32part(buf, CONFIG_USB251X_BLOCK_SIZE, crc2);
    }

  /* Check for configuration validity */
  
  ret = ERROR;
  if (crc1 == crc2)
    {
      /* check the ids */

      /* confiures the hub ports */

      ret = usb251x_config_ports(priv);
      if (ret < 0)
        {
          usb251x_err("ERROR: Failed to config port(s)\n");
          goto err_out;
        } 

      /* attach the USB up-stream port */

      return usb251x_attach(priv);
    }

err_out:
  return ret;
}

/****************************************************************************
 * Name: usb251x_reset
 *
 * Description:
 *   Reset USB251X HW and clear I2C registers
 *
 ****************************************************************************/

static int usb251x_reset(FAR struct usb251x_dev_s *priv)
{
  int ret = OK;
  uint8_t reg = STCD_RESET;

  ret = usb251x_block_write(priv, USB251X_STCD_REG, &reg, 1);
  if (ret < 0)
    {
      usb251x_err("ERROR: Failed to write command register\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: usb251x_attach
 *
 * Description:
 *   Attach USB251X to the bus
 *
 ****************************************************************************/

static int usb251x_attach(FAR struct usb251x_dev_s *priv)
{
  int ret = OK;
  uint8_t reg = STCD_ATTACH;

  ret = usb251x_block_write(priv, USB251X_STCD_REG, &reg, 1);
  if (ret < 0)
    {
      usb251x_err("ERROR: Failed to write command register\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: usb251x_detach
 *
 * Description:
 *   Detach USB251X from the bus
 *
 ****************************************************************************/

static int usb251x_detach(FAR struct usb251x_dev_s *priv)
{
  int ret = OK;
  uint8_t reg = STCD_DETACH;

  ret = usb251x_block_write(priv, USB251X_STCD_REG, &reg, 1);
  if (ret < 0)
    {
      usb251x_err("ERROR: Failed to write command register\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: usb251x_open
 *
 * Description:
 *   This function is called whenever the USB251X device is opened.
 *
 ****************************************************************************/

static int usb251x_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: usb251x_close
 *
 * Description:
 *   This routine is called when the USB251X device is closed.
 *
 ****************************************************************************/

static int usb251x_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: usb251x_ioctl
 * Description:
 *   This routine is called when ioctl function call is performed for
 *   the USB251X device.
 ****************************************************************************/

static int usb251x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usb251x_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  usb251x_info("cmd: 0x%02X, arg:%lu\n", cmd, arg);

  switch (cmd)
  {
  case USBCIOC_READ_VID:
    {

    }
    break;

  case USBCIOC_READ_PID:
    {

    }
    break;

  case USBCIOC_READ_DID:
    {

    }
    break;

  case USBCIOC_CONFIG:
    {
      ret = usb251x_config(priv, (struct usb251x_config_s *)arg);
    }
    break;

  case USBCIOC_ATTACH:
    {
      ret = usb251x_attach(priv);
    }
    break;

  case USBCIOC_DETACH:
    {
      ret = usb251x_detach(priv);
    }
    break;

  case USBCIOC_RESET:
    {
      ret = usb251x_reset(priv);
    }
    break;

  default:
    {
      usb251x_err("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
    }
    break;
  }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int usb251x_register(FAR const char *devpath, FAR struct usb251x_config_s *config)
{
  FAR struct usb251x_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && config != NULL && config->i2c != NULL);

  /* Initialize the USB251X device structure */

  priv = (FAR struct usb251x_dev_s *)
                kmm_zalloc(sizeof(struct usb251x_dev_s));
  if (!priv)
    {
      usb251x_err("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize device structure mutex */

  nxmutex_init(&priv->devlock);

  /* Save the configuration data */

  priv->config = config;

  /* Register the character driver */

  ret = register_driver(devpath, &g_usb251xops, 0666, priv);
  if (ret < 0)
    {
      usb251x_err("ERROR: Failed to register driver: %d\n", ret);
      goto errout_with_priv;
    }

  /* Configure the device */

  return usb251x_config(priv, config);

errout_with_priv:
  nxmutex_destroy(&priv->devlock);
  kmm_free(priv);
  return ret;
}
