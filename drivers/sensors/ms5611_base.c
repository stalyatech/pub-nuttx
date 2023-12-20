/****************************************************************************
 * drivers/sensors/ms5611_base.c
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

#if defined(CONFIG_SENSORS_MS5611)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

#ifdef CONFIG_MS5611_SPI
/****************************************************************************
 * Name: ms5611_read_spi
 *
 * Description:
 *   Read from MS5611 with SPI bus
 *
 ****************************************************************************/

static int ms5611_read_spi(FAR struct ms5611_dev_s *dev,
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
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETFREQUENCY(spi, 1000000);

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
 * Name: ms5611_write_spi
 *
 * Description:
 *   Write to MS5611 with SPI bus
 *
 ****************************************************************************/

static int ms5611_write_spi(FAR struct ms5611_dev_s *dev,
                            FAR const uint8_t *buf, uint8_t len)
{
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;
  int ret;

  /* Hopefully, we'll return all the bytes they're asking for. */

  ret = len;

  /* Grab and configure the SPI master device. */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETFREQUENCY(spi, 1000000);

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

#endif /* CONFIG_MS5611_SPI */

#ifdef CONFIG_MS5611_I2C
/****************************************************************************
 * Name: ms5611_read_i2c
 *
 * Description:
 *   Read from MS5611 with I2C bus
 *
 ****************************************************************************/

static int ms5611_read_i2c(FAR struct ms5611_dev_s *dev,
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
 * Name: ms5611_write_i2c
 *
 * Description:
 *   Write to MS5611 with I2C bus
 *
 ****************************************************************************/

static int ms5611_write_i2c(FAR struct ms5611_dev_s *dev,
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
#endif /* CONFIG_MS5611_I2C */

/* ms5611_read()
 *
 * Reads a block of @len byte-wide registers, starting at @regaddr,
 * from the device connected to @dev. Bytes are returned in @buf,
 * which must have a capacity of at least @len bytes.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes read, or a negative errno.
 */

int ms5611_read(FAR struct ms5611_dev_s *dev,
                FAR uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_MS5611_SPI
  /* If we're wired to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return ms5611_read_spi(dev, buf, len);
    }
#endif /* CONFIG_MS5611_SPI */

#ifdef CONFIG_MS5611_I2C
  /* If we're wired to I2C, use that function. */

  if (dev->config.i2c != NULL)
    {
      return ms5611_read_i2c(dev, buf, len);
    }
#endif /* CONFIG_MS5611_I2C */

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* ms5611_write()
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

int ms5611_write(FAR struct ms5611_dev_s *dev,
                 FAR const uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_MS5611_SPI
  /* If we're connected to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return ms5611_write_spi(dev, buf, len);
    }
#endif /* CONFIG_MS5611_SPI */

#ifdef CONFIG_MS5611_I2C
  if (dev->config.i2c != NULL)
    {
      return ms5611_write_i2c(dev, buf, len);
    }
#endif /* CONFIG_MS5611_I2C */

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

#endif /* CONFIG_SENSORS_MS5611 */
