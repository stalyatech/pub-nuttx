/****************************************************************************
 * drivers/sensors/icm20689_base.c
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

#include "icm20689_base.h"

#if defined(CONFIG_SENSORS_ICM20689)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device uses SPI Mode 3: CPOL=1, CPHA=1 */

#define ICM20689_SPI_MODE   (SPIDEV_MODE3)

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

#ifdef CONFIG_ICM20689_SPI
/****************************************************************************
 * Name: icm20689_read_reg_spi
 *
 * Description:
 *   Read from ICM20689 registers with SPI bus
 *
 ****************************************************************************/

static int icm20689_read_reg_spi(FAR struct icm20689_dev_s *dev,
                                enum icm20689_regaddr_e reg_addr,
                                FAR uint8_t *buf, uint8_t len)
{
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* Grab and configure the SPI master device: always mode 0, maximum 8MHz if it's a
   * data register, 1MHz otherwise (per datasheet).
   */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, ICM20689_SPI_MODE);

  if ((reg_addr >= ACCEL_XOUT_H) && ((reg_addr + len) <= GYRO_ZOUT_L))
    {
      SPI_SETFREQUENCY(spi, dev->config.freq);
    }
  else
    {
      SPI_SETFREQUENCY(spi, 100000);
    }

  /* Select the chip. */

  SPI_SELECT(spi, id, true);

  /* Send the read request. */

  SPI_SEND(spi, reg_addr | ICM_REG_READ);

  /* Clock in the data. */

  while (0 != len--)
    {
      *buf++ = (uint8_t) (SPI_SEND(spi, 0xff));
    }

  /* Deselect the chip, release the SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return OK;
}

/****************************************************************************
 * Name: icm20689_write_reg_spi
 *
 * Description:
 *   Write to ICM20689 registers with SPI bus
 *
 ****************************************************************************/

static int icm20689_write_reg_spi(FAR struct icm20689_dev_s *dev,
                                 enum icm20689_regaddr_e regaddr,
                                 FAR const uint8_t *buf, uint8_t len)
{
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* Grab and configure the SPI master device. */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, ICM20689_SPI_MODE);
  SPI_SETFREQUENCY(spi, 1000000);

  /* Select the chip. */

  SPI_SELECT(spi, id, true);

  /* Send the write request. */

  SPI_SEND(spi, regaddr | ICM_REG_WRITE);

  /* Send the data. */

  while (0 != len--)
    {
      SPI_SEND(spi, *buf++);
    }

  /* Release the chip and SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return OK;
}

#endif /* CONFIG_ICM20689_SPI */

#ifdef CONFIG_ICM20689_I2C
/****************************************************************************
 * Name: icm20689_read_reg_i2c
 *
 * Description:
 *   Read from ICM20689 registers with I2C bus
 *
 ****************************************************************************/

static int icm20689_read_reg_i2c(FAR struct icm20689_dev_s *dev,
                                uint8_t regaddr, FAR uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = dev->config.freq;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = dev->config.freq;
  msg[1].addr      = dev->config.addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = len;

  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(read) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: icm20689_write_reg_i2c
 *
 * Description:
 *   Write to ICM20689 registers with I2C bus
 *
 ****************************************************************************/

static int icm20689_write_reg_i2c(FAR struct icm20689_dev_s *dev,
                                  uint8_t regaddr, FAR const uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = dev->config.freq;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;
  msg[1].frequency = dev->config.freq;
  msg[1].addr      = dev->config.addr;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = (FAR uint8_t *)buf;
  msg[1].length    = len;
  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(write) failed: %d\n", ret);
      return ret;
    }

  return OK;
}
#endif /* CONFIG_ICM20689_I2C */

/* icm20689_read_reg()
 *
 * Reads a block of @len byte-wide registers, starting at @regaddr,
 * from the device connected to @dev. Bytes are returned in @buf,
 * which must have a capacity of at least @len bytes.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes read, or a negative errno.
 */

int icm20689_read_reg(FAR struct icm20689_dev_s *dev,
                      enum icm20689_regaddr_e regaddr, FAR uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_ICM20689_SPI
  /* If we're wired to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return icm20689_read_reg_spi(dev, regaddr, buf, len);
    }
#endif /* CONFIG_ICM20689_SPI */

#ifdef CONFIG_ICM20689_I2C
  /* If we're wired to I2C, use that function. */

  if (dev->config.i2c != NULL)
    {
      return icm20689_read_reg_i2c(dev, regaddr, buf, len);
    }
#endif /* CONFIG_ICM20689_I2C */

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* icm20689_write_reg()
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

int icm20689_write_reg(FAR struct icm20689_dev_s *dev,
                       enum icm20689_regaddr_e regaddr, FAR const uint8_t *buf, uint8_t len)
{
#ifdef CONFIG_ICM20689_SPI
  /* If we're connected to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return icm20689_write_reg_spi(dev, regaddr, buf, len);
    }
#endif /* CONFIG_ICM20689_SPI */

#ifdef CONFIG_ICM20689_I2C
  if (dev->config.i2c != NULL)
    {
      return icm20689_write_reg_i2c(dev, regaddr, buf, len);
    }
#endif /* CONFIG_ICM20689_I2C */

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/****************************************************************************
 * icm20689_modify_reg()
 *
 * Description:
 *   Modify a 8-bit register value by address
 *   icm20689_modify_reg(d,v,m,a) defined as:
 *   icm20689_write_reg(d,(icm20689_read_reg(d,a) & ~(m)) | ((v) & (m)), (a))
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes written, or a negative errno.
 ****************************************************************************/

int icm20689_modify_reg(FAR struct icm20689_dev_s *dev,
                        enum icm20689_regaddr_e regaddr, uint8_t clearbits, uint8_t setbits)
{
  uint8_t buf = 0xff;

  icm20689_read_reg(dev, regaddr, &buf, sizeof(buf));

  buf = (buf & ~clearbits) | (setbits & clearbits);

  return icm20689_write_reg(dev, regaddr, &buf, sizeof(buf));
}

/****************************************************************************
 * Name: icm20689_checkid
 *
 * Description:
 *   Read and verify the ICM20689 chip ID
 *
 ****************************************************************************/

int icm20689_checkid(FAR struct icm20689_dev_s *dev)
{
  uint8_t devid;
  int ret;

  /* Read device ID */

  ret = icm20689_read_reg(dev, WHO_AM_I, &devid, sizeof(devid));
  if (ret < 0)
    {
      return ret;
    }

  sninfo("devid: %04x\n", devid);
  if (devid != ICM20689_DEVID)
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_SENSORS_ICM20689 */
