/****************************************************************************
 * drivers/sensors/ms56xx_base.c
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

#if defined(CONFIG_SENSORS_MS56XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device uses SPI Mode 0: CPOL=0, CPHA=0 */

#define MS56XX_SPI_MODE   (SPIDEV_MODE0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_MS56XX_SPI
/****************************************************************************
 * Name: ms56xx_read_spi
 *
 * Description:
 *   Read from MS56XX with SPI bus
 *
 ****************************************************************************/

static int ms56xx_read_spi(FAR struct ms56xx_dev_s *dev,
                           FAR uint8_t *buf, uint8_t len)
{
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;
  int ret;

  /* We'll probably return the number of bytes asked for. */

  ret = len;

  /* Grab and configure the SPI master device: always mode 0, 20MHz if it's a
   * data register, 1MHz otherwise (per datasheet).
   */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, MS56XX_SPI_MODE);
  SPI_SETFREQUENCY(spi, dev->config.freq);

  /* Select the chip. */

  SPI_SELECT(spi, id, true);

  /* Clock in the data. */

  while (0 != len--)
    {
      *buf++ = (uint8_t) (SPI_SEND(spi, 0xff));
    }

  /* Deselect the chip, release the SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return ret;
}

/****************************************************************************
 * Name: ms56xx_write_spi
 *
 * Description:
 *   Write to MS56XX with SPI bus
 *
 ****************************************************************************/

static int ms56xx_write_spi(FAR struct ms56xx_dev_s *dev,
                            FAR const uint8_t *buf, uint8_t len)
{
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;
  int ret;

  /* Hopefully, we'll return all the bytes they're asking for. */

  ret = len;

  /* Grab and configure the SPI master device. */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, MS56XX_SPI_MODE);
  SPI_SETFREQUENCY(spi, dev->config.freq);

  /* Select the chip. */

  SPI_SELECT(spi, id, true);

  /* Send the data. */

  while (0 != len--)
    {
      SPI_SEND(spi, *buf++);
    }

  /* Release the chip and SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return ret;
}


/****************************************************************************
 * Name: ms56xx_transfer_spi
 *
 * Description:
 *   Write/Read to/from MS56XX with SPI bus
 *
 ****************************************************************************/

static int ms56xx_transfer_spi(FAR struct ms56xx_dev_s *dev,
                               FAR const uint8_t *txbuf, uint8_t txlen,
                               FAR uint8_t *rxbuf, uint8_t rxlen)
{
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* Grab and configure the SPI master device. */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, MS56XX_SPI_MODE);
  SPI_SETFREQUENCY(spi, dev->config.freq);

  /* Select the chip. */

  SPI_SELECT(spi, id, true);

  /* Send the data. */

  while (0 != txlen--)
    {
      SPI_SEND(spi, *txbuf++);
    }

  /* Receive the data. */

  while (0 != rxlen--)
    {
      *rxbuf++ = (uint8_t) (SPI_SEND(spi, 0xff));
    }

  /* Release the chip and SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return OK;
}
#endif /* CONFIG_MS56XX_SPI */

#ifdef CONFIG_MS56XX_I2C
/****************************************************************************
 * Name: ms56xx_read_i2c
 *
 * Description:
 *   Read from MS56XX with I2C bus
 *
 ****************************************************************************/

static int ms56xx_read_i2c(FAR struct ms56xx_dev_s *dev,
                           FAR uint8_t *buf, uint8_t len)
{
  struct i2c_msg_s msg[1];
  int ret;

  msg[0].frequency = dev->config.freq;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_READ;
  msg[0].buffer    = (FAR uint8_t *)buf;
  msg[0].length    = len;

  ret = I2C_TRANSFER(dev->config.i2c, msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(read) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ms56xx_write_i2c
 *
 * Description:
 *   Write to MS56XX with I2C bus
 *
 ****************************************************************************/

static int ms56xx_write_i2c(FAR struct ms56xx_dev_s *dev,
                            FAR const uint8_t *buf, uint8_t len)
{
  struct i2c_msg_s msg[1];
  int ret;

  msg[0].frequency = dev->config.freq;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = 0;
  msg[0].buffer    = (FAR uint8_t *)buf;
  msg[0].length    = len;

  ret = I2C_TRANSFER(dev->config.i2c, msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(write) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: icm20689_read_reg_i2c
 *
 * Description:
 *   Read from ICM20689 registers with I2C bus
 *
 ****************************************************************************/

static int ms56xx_transfer_i2c(FAR struct ms56xx_dev_s *dev,
                               FAR const uint8_t *txbuf, uint8_t txlen,
                               FAR uint8_t *rxbuf, uint8_t rxlen)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = dev->config.freq;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = (FAR uint8_t *)txbuf;
  msg[0].length    = txlen;

  msg[1].frequency = dev->config.freq;
  msg[1].addr      = dev->config.addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = rxbuf;
  msg[1].length    = rxlen;

  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(write/read) failed: %d\n", ret);
      return ret;
    }

  return OK;
}
#endif /* CONFIG_MS56XX_I2C */

/* ms56xx_read()
 *
 * Reads a block of @len byte-wide registers, starting at @regaddr,
 * from the device connected to @dev. Bytes are returned in @buf,
 * which must have a capacity of at least @len bytes.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes read, or a negative errno.
 */

int ms56xx_read(FAR struct ms56xx_dev_s *dev,
                FAR uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_MS56XX_SPI
  /* If we're wired to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return ms56xx_read_spi(dev, buf, len);
    }
#endif /* CONFIG_MS56XX_SPI */

#ifdef CONFIG_MS56XX_I2C
  /* If we're wired to I2C, use that function. */

  if (dev->config.i2c != NULL)
    {
      return ms56xx_read_i2c(dev, buf, len);
    }
#endif /* CONFIG_MS56XX_I2C */

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* ms56xx_write()
 *
 * Writes a block of @len byte-wide registers, starting at @regaddr,
 * using the values in @buf to the device connected to @dev. Register
 * values are taken in numerical order from @buf, i.e.:
 *
 *   buf[0] -> register[@reg_addr]
 *   buf[1] -> register[@reg_addr + 1]
 *   ...
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes written, or a negative errno.
 */

int ms56xx_write(FAR struct ms56xx_dev_s *dev,
                 FAR const uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_MS56XX_SPI
  /* If we're connected to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return ms56xx_write_spi(dev, buf, len);
    }
#endif /* CONFIG_MS56XX_SPI */

#ifdef CONFIG_MS56XX_I2C
  if (dev->config.i2c != NULL)
    {
      return ms56xx_write_i2c(dev, buf, len);
    }
#endif /* CONFIG_MS56XX_I2C */

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/****************************************************************************
 * Name: ms56xx_transfer
 *
 * Description:
 *   Send/receive data to/from MS56XX device
 *
 ****************************************************************************/

int ms56xx_transfer(FAR struct ms56xx_dev_s *dev,
                    FAR const uint8_t *txbuf, uint8_t txlen,
                    FAR uint8_t *rxbuf, uint8_t rxlen)
{
#ifdef CONFIG_MS56XX_SPI
  /* If we're connected to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return ms56xx_transfer_spi(dev, txbuf, txlen, rxbuf, rxlen);
    }
#endif /* CONFIG_MS56XX_SPI */

#ifdef CONFIG_MS56XX_I2C
  if (dev->config.i2c != NULL)
    {
      return ms56xx_transfer_i2c(dev, txbuf, txlen, rxbuf, rxlen);
    }
#endif /* CONFIG_MS56XX_I2C */

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/****************************************************************************
 * Name: ms56xx_checkid
 *
 * Description:
 *   Read and verify the MS56XX chip ID
 *
 ****************************************************************************/

int ms56xx_checkid(FAR struct ms56xx_dev_s *dev)
{
  return OK;
}

#endif /* CONFIG_SENSORS_MS56XX */
