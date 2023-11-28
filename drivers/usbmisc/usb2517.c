/****************************************************************************
 * drivers/usbmisc/usb2517.c
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

#include <nuttx/usb/usb2517.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_USB2517
#  define usb2517_err(x, ...)        _err(x, ##__VA_ARGS__)
#  define usb2517_info(x, ...)       _info(x, ##__VA_ARGS__)
#else
#  define usb2517_err(x, ...)        uerr(x, ##__VA_ARGS__)
#  define usb2517_info(x, ...)       uinfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_USB2517_I2C_FREQUENCY
#  define CONFIG_USB2517_I2C_FREQUENCY 100000
#endif

/* Other macros */

#define USB2517_I2C_RETRIES   10

#define USB2517_MFR_LEN_OFS	  (0x13)
#define USB2517_PRD_LEN_OFS	  (0x14)
#define USB2517_SER_LEN_OFS	  (0x15)
#define USB2517_MFR_STR_OFS	  (0x16)
#define USB2517_PRD_STR_OFS	  (0x54)
#define USB2517_SER_STR_OFS	  (0x92)


/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct usb2517_dev_s
{
  FAR struct usb2517_config_s *config;  /* Platform specific configuration */
  FAR struct i2c_master_s *i2c;         /* I2C interface */
  uint8_t addr;                         /* I2C address */
  mutex_t devlock;                      /* Manages exclusive access */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static int usb2517_open(FAR struct file *filep);
static int usb2517_close(FAR struct file *filep);
static int usb2517_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int usb2517_portcfg(struct usb2517_dev_s *priv, enum usb2517_ports_e port, bool stat);
static int usb2517_config_ports(struct usb2517_dev_s *priv);
static int usb2517_attach(FAR struct usb2517_dev_s *priv);
static int usb2517_detach(FAR struct usb2517_dev_s *priv);
static int usb2517_reset(FAR struct usb2517_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_usb2517ops =
{
  usb2517_open,  /* open */
  usb2517_close, /* close */
  NULL,          /* read */
  NULL,          /* write */
  NULL,          /* seek */
  usb2517_ioctl, /* ioctl */
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
static void insert_mfr_str(struct usb2517_config_s *config)
{
  char utf16_str[62] = {0};

  DEBUGASSERT(config != NULL);

  /* create the utf16 data */

  int len = create_utf16_str(config->mfr_str, utf16_str);

  /* update the config */

  config->config[USB2517_MFR_LEN_OFS] = len/2;

  memcpy(&config->config[USB2517_MFR_STR_OFS], utf16_str, len);
}

/*******************************************************************************
* Function Name  : insert_prd_str
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void insert_prd_str(struct usb2517_config_s *config)
{
  char utf16_str[62] = {0};

  DEBUGASSERT(config != NULL);

  /* create the utf16 data */

  int len = create_utf16_str(config->prd_str, utf16_str);

  /* update the config */

  config->config[USB2517_PRD_LEN_OFS] = len/2;

  memcpy(&config->config[USB2517_PRD_STR_OFS], utf16_str, len);
}

/*******************************************************************************
* Function Name  : insert_ser_str
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void insert_ser_str(struct usb2517_config_s *config)
{
  char utf16_str[62] = {0};

  DEBUGASSERT(config != NULL);

  /* create the utf16 data */

  int len = create_utf16_str(config->ser_str, utf16_str);

  /* update the config */

  config->config[USB2517_SER_LEN_OFS] = len/2;

  memcpy(&config->config[USB2517_SER_STR_OFS], utf16_str, len);
}

/****************************************************************************
 * Name: usb2517_block_read
 *
 * Description:
 *   Read register block from USB2517
 *
 * Input Parameters:
 *   priv    - pointer to USB2517 Private Structure
 *   regaddr - register address to read
 *   regvals - register values to read
 *   len     - length of read values
 *
 * Returned Value:
 *   Returns OK in case of success, otherwise ERROR
 ****************************************************************************/

static int usb2517_block_read(FAR struct usb2517_dev_s *priv, uint8_t regaddr, uint8_t *regvals, uint8_t len)
{
  uint8_t rxbuffer[CONFIG_USB2517_BLOCK_SIZE+1];
  struct i2c_msg_s msg[2];
  int retries;
  int ret = -EIO;

  DEBUGASSERT(priv != NULL && regvals != NULL);

  /* Format two messages: The first is a write which is never terminated
   * with STOP condition.
   */

  msg[0].frequency = CONFIG_USB2517_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  /* The second is either a read (rbuflen > 0) with a repeated start or a
   * write (rbuflen < 0) with no restart.
   */

  msg[1].frequency = CONFIG_USB2517_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = rxbuffer;
  msg[1].length    = len + 1;

  /* Perform the transfer */

  for (retries = 0; retries < USB2517_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret >= 0)
        {
          memcpy(regvals, &rxbuffer[1], len);
          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == USB2517_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              usb2517_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  usb2517_info("reg:%02X, error:%d\n", regaddr, ret);
  return ret;
}

/****************************************************************************
 * Name: usb2517_block_write
 *
 * Description:
 *   Write a value to an 8-bit USB2517 register
 *
 * Input Parameters:
 *   priv    - pointer to USB2517 Private Structure
 *   regaddr - register address to write
 *   regvals - register values to write
 *   len     - length of write values
 *
 * Returned Value:
 *   Returns OK in case of success, otherwise ERROR
 ****************************************************************************/

static int usb2517_block_write(FAR struct usb2517_dev_s *priv, uint8_t regaddr, uint8_t *regvals, uint8_t len)
{
  uint8_t txbuffer[CONFIG_USB2517_BLOCK_SIZE+2];
  struct i2c_msg_s msg;
  int retries;
  int ret = -EIO;

  DEBUGASSERT(priv != NULL && regvals != NULL);

  /* Setup to the data to be transferred (register address and values). */

  txbuffer[0] = regaddr;
  txbuffer[1] = len;
  memcpy(&txbuffer[2], regvals, len);

  /* Set up the I2C configuration */

  msg.frequency = CONFIG_USB2517_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = len + 2;

  /* Perform the transfer */

  for (retries = 0; retries < USB2517_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->i2c, &msg, 1);
      if (ret == OK)
        {
          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == USB2517_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              usb2517_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  usb2517_err("ERROR: failed reg:%02X, error:%d\n", regaddr, ret);
  return ret;
}

/*******************************************************************************
* Function Name  : usb2517_portcfg
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static int usb2517_portcfg(struct usb2517_dev_s *priv, enum usb2517_ports_e port, bool stat)
{
  uint8_t port_map[4];
  int ret;

  /* read the current port remap status */

  ret = usb2517_block_read(priv, USB2517_PRTR12_REG, port_map, 4);
  if (ret < 0)
    {
      usb2517_err("ERROR: USB2517 port(%d) config failed: %d\n", port, ret);
      return ret;
    }

  /* scan the all logical port numbers */

  uint8_t log_num = 0, num;
  
  for (uint8_t i = 0; i < 4; i++)
    {
      /* find the last logical number */
      if ((num = (port_map[i] & 0xff) & 7) > log_num) {
          log_num = num;
      }
      if ((num = ((port_map[i] >> 4) & 0xff) & 7) > log_num) {
          log_num = num;
      }
    }

  /* update the next logical number */
  log_num++;

  /* check the physical port number */
  switch ((uint8_t)port)
  {
    case 1:
      port_map[0] &= 0xF0;
      if (stat) {
          port_map[0] |= log_num;
      }
      break;
        
    case 2:
      port_map[0] &= 0x0F;
      if (stat) {
          port_map[0] |= (log_num << 4);
      }
      break;

    case 3:
      port_map[1] &= 0xF0;
      if (stat) {
          port_map[1] |= log_num;
      }
      break;
        
    case 4:
      port_map[1] &= 0x0F;
      if (stat) {
          port_map[1] |= (log_num << 4);
      }
      break;

    case 5:
      port_map[2] &= 0xF0;
      if (stat) {
          port_map[2] |= log_num;
      }
      break;
        
    case 6:
      port_map[2] &= 0x0F;
      if (stat) {
          port_map[2] |= (log_num << 4);
      }
      break;

    case 7:
      port_map[3] &= 0xF0;
      if (stat) {
          port_map[3] |= log_num;
      }
      break;
  }

  /* write the new port remap status */
  
  ret = usb2517_block_write(priv, USB2517_PRTR12_REG, port_map, 4);
  if (ret < 0)
    {
      usb2517_err("ERROR: USB2517 port(%d) config failed: %d\n", port, ret);
      return ret;
    }

  return OK;
}

/*******************************************************************************
* Function Name  : usb2517_config_ports
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static int usb2517_config_ports(struct usb2517_dev_s *priv)
{
    int ret = 0;

    /* Enable the MCU USB port */
    ret += usb2517_portcfg(priv, USB2517_MCU_PORT, true);

    /* Enable the F9P USB port */
    ret += usb2517_portcfg(priv, USB2517_F9P_PORT, true);

    /* Enable the F9H1 USB port */
    ret += usb2517_portcfg(priv, USB2517_F9H1_PORT, true);

    /* Enable the F9H2 USB port */
    ret += usb2517_portcfg(priv, USB2517_F9H2_PORT, true);

    /* Enable the XBEE USB port */
    ret += usb2517_portcfg(priv, USB2517_XBEE_PORT, true);

    /* Enable the LTE USB port */
    ret += usb2517_portcfg(priv, USB2517_M2M_PORT, true);

    return ret;
}

/****************************************************************************
 * Name: usb2517_config
 *
 * Description:
 *   Setup USB2517 chip
 *
 ****************************************************************************/

static int usb2517_config(FAR struct usb2517_dev_s *priv,
                         struct usb2517_config_s *config)
{
  uint8_t buf[CONFIG_USB2517_BLOCK_SIZE];
	uint32_t crc1 = 0xffffffff;
	uint32_t crc2 = 0xffffffff;
  uint8_t i, addr, *cfg;
  int ret = OK;

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
        addr = i * CONFIG_USB2517_BLOCK_SIZE; 

        /* get the configuration part */
        cfg = &config->config[addr];

        /* write configuration block */

        ret = usb2517_block_write(priv, addr, cfg, CONFIG_USB2517_BLOCK_SIZE);
        if (ret < 0)
          {
            usb2517_err("ERROR: Failed to write register(s)\n");
            goto err_out;
          }
        
        /* calculate the crc value of configuration part */

        crc1 = crc32part(cfg, CONFIG_USB2517_BLOCK_SIZE, crc1);
    }

  /* read back the configuration */

  for (i = 0; i < 8; i++)
    {
        /* update the register address */
        addr = i * CONFIG_USB2517_BLOCK_SIZE; 

        /* read configuration block */

        ret = usb2517_block_read(priv, addr, buf, CONFIG_USB2517_BLOCK_SIZE);
        if (ret < 0)
          {
            usb2517_err("ERROR: Failed to write register(s)\n");
            goto err_out;
          } 

        /* calculate the crc value of configuration part */
        
        crc2 = crc32part(buf, CONFIG_USB2517_BLOCK_SIZE, crc2);
    }

  /* Check for configuration validity */
  
  if (crc1 == crc2)
    {
      /* check the ids */

      /* confiures the hub ports */
      ret = usb2517_config_ports(priv);
      if (ret < 0)
        {
          usb2517_err("ERROR: Failed to config port(s)\n");
          goto err_out;
        } 

      /* attach the USB up-stream port */
      return usb2517_attach(priv);
    }

err_out:
  return ret;
}

/****************************************************************************
 * Name: usb2517_reset
 *
 * Description:
 *   Reset USB2517 HW and clear I2C registers
 *
 ****************************************************************************/

static int usb2517_reset(FAR struct usb2517_dev_s *priv)
{
  int ret = OK;
  uint8_t reg = STCD_RESET;

  ret = usb2517_block_write(priv, USB2517_STCD_REG, &reg, 1);
  if (ret < 0)
    {
      usb2517_err("ERROR: Failed to write command register\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: usb2517_attach
 *
 * Description:
 *   Attach USB2517 to the bus
 *
 ****************************************************************************/

static int usb2517_attach(FAR struct usb2517_dev_s *priv)
{
  int ret = OK;
  uint8_t reg = STCD_ATTACH;

  ret = usb2517_block_write(priv, USB2517_STCD_REG, &reg, 1);
  if (ret < 0)
    {
      usb2517_err("ERROR: Failed to write command register\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: usb2517_detach
 *
 * Description:
 *   Detach USB2517 from the bus
 *
 ****************************************************************************/

static int usb2517_detach(FAR struct usb2517_dev_s *priv)
{
  int ret = OK;
  uint8_t reg = STCD_DETACH;

  ret = usb2517_block_write(priv, USB2517_STCD_REG, &reg, 1);
  if (ret < 0)
    {
      usb2517_err("ERROR: Failed to write command register\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: usb2517_open
 *
 * Description:
 *   This function is called whenever the USB2517 device is opened.
 *
 ****************************************************************************/

static int usb2517_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: usb2517_close
 *
 * Description:
 *   This routine is called when the USB2517 device is closed.
 *
 ****************************************************************************/

static int usb2517_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: usb2517_ioctl
 * Description:
 *   This routine is called when ioctl function call is performed for
 *   the USB2517 device.
 ****************************************************************************/

static int usb2517_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usb2517_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  usb2517_info("cmd: 0x%02X, arg:%lu\n", cmd, arg);

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
      ret = usb2517_config(priv, (struct usb2517_config_s *)arg);
    }
    break;

  case USBCIOC_ATTACH:
    {
      ret = usb2517_attach(priv);
    }
    break;

  case USBCIOC_DETACH:
    {
      ret = usb2517_detach(priv);
    }
    break;

  case USBCIOC_RESET:
    {
      ret = usb2517_reset(priv);
    }
    break;

  default:
    {
      usb2517_err("ERROR: Unrecognized cmd: %d\n", cmd);
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

int usb2517_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, FAR struct usb2517_config_s *config)
{
  FAR struct usb2517_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL && config != NULL);

  /* Initialize the USB2517 device structure */

  priv = (FAR struct usb2517_dev_s *)
                kmm_zalloc(sizeof(struct usb2517_dev_s));
  if (!priv)
    {
      usb2517_err("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize device structure mutex */

  nxmutex_init(&priv->devlock);

  priv->i2c    = i2c;
  priv->addr   = addr;
  priv->config = config;

  /* Register the character driver */

  ret = register_driver(devpath, &g_usb2517ops, 0666, priv);
  if (ret < 0)
    {
      usb2517_err("ERROR: Failed to register driver: %d\n", ret);
      goto errout_with_priv;
    }

  /* Configure the device */

  return usb2517_config(priv, config);

errout_with_priv:
  nxmutex_destroy(&priv->devlock);
  kmm_free(priv);
  return ret;
}
