/****************************************************************************
 * drivers/spi/trion.c
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>

#include <nuttx/spi/trion.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int 			trion_open	(FAR struct file *filep);
static int 			trion_close	(FAR struct file *filep);
static ssize_t 	trion_read	(FAR struct file *filep, FAR char *buffer,
                           	 size_t buflen);
static ssize_t 	trion_write	(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int 			trion_ioctl	(FAR struct file *filep, int cmd, unsigned long arg);

/* Helper functions */

static int trion_init_fpga	(FAR struct trion_dev_s *dev);

static int trion_writeblk		(FAR struct trion_dev_s *dev,
                           	 FAR const char *buffer,
                           	 size_t buflen);

static int trion_endwrite		(FAR struct trion_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_trion_fops =
{
  trion_open, 	/* open */
  trion_close, 	/* close */
  trion_read,  	/* read */
  trion_write, 	/* write */
  NULL,        	/* seek */
  trion_ioctl, 	/* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trion_open
 *
 * Description:
 *  This function is called whenever the Efinix device is opened.
 *
 ****************************************************************************/

static int trion_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct trion_dev_s *dev = inode->i_private;

	/* Validate parameters */

  DEBUGASSERT(dev != NULL);

	/* Make sure that the device is not already open */

  if (dev->is_open)
    {
      return -EBUSY;
    }

	/* Power on the device */

  dev->ops->power(dev, true);

	/* Release the reset line */

  dev->ops->reset(dev, false);

	/* Mark the device as open */

  dev->is_open = true;

  return OK;
}

/****************************************************************************
 * Name: trion_close
 *
 * Description:
 *  This function is called whenever the Efinix device is closed.
 *
 ****************************************************************************/

static int trion_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct trion_dev_s *dev = inode->i_private;

	/* Validate parameters */

  DEBUGASSERT(dev != NULL);

	/* Make sure that the device is open */

  if (dev->in_progress)
    {
			/* End writing to the FPGA */

      if (trion_endwrite(dev) != OK)
        {
          spierr("ERROR: Failed to end writing to FPGA\n");
          dev->is_open = false;
          return -EIO;
        }
    }

	/* Mark the device as closed */

  dev->is_open = false;

  return OK;
}

/****************************************************************************
 * Name: trion_configspi
 *
 * Description:
 *   Configure the SPI instance for to match the DAT-31R5-SP+
 *   specifications
 *
 ****************************************************************************/

static inline void trion_configspi(FAR struct spi_dev_s *spi)
{
	/* Validate parameters */

  DEBUGASSERT(spi != NULL);

  /* Configure SPI Mode for the Efinix Trion */

  SPI_SETMODE(spi, CONFIG_SPI_TRION_MODE);
  SPI_SETBITS(spi, 8);

  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_SPI_TRION_FREQUENCY);
}

/****************************************************************************
 * Name: trion_init_fpga
 *
 * Description:
 *  Initialize the FPGA - set it to SPI Master load mode
 *  Reset the FPGA with the CS pin active low
 *  and send 8 dummy bits with CS high to start the SPI transfer.
 *
 ****************************************************************************/

static int trion_init_fpga(FAR struct trion_dev_s *dev)
{
	/* Validate parameters */

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(dev->spi != NULL);

	/* Lock SPI */

  SPI_LOCK(dev->spi, true);

	/* Configure SPI for the Efinix Trion */

  trion_configspi(dev->spi);

	/* Enter to the SPI Passive mode */

	/* Select the device */

  dev->ops->select(dev, true);
  up_udelay(2);

	/* Assert the reset line */

  dev->ops->reset(dev, true);
  up_udelay(2);

	/* Release the reset line */

  dev->ops->reset(dev, false);
  up_udelay(2);

	/* Mark the FPGA as in progress */

  dev->in_progress = true;

  return 0;
}

/****************************************************************************
 * Name: trion_writeblk
 *
 * Description:
 *  Write block to the Efinix FPGA, max 4096 bytes
 ****************************************************************************/

static inline int trion_writeblk(FAR struct trion_dev_s *dev, FAR const char *buffer,
                								 size_t buflen)
{
  uint32_t nbytes;

	/* Validate parameters */

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(dev->spi != NULL);
  DEBUGASSERT(buffer != NULL);
  DEBUGASSERT(buflen > 0);

	/* Check if the FPGA is initialized */

  if (!dev->in_progress)
    {
      spierr("ERROR: FPGA not initialized\n");
      return -EINVAL;
    }

	/* Configure SPI for the Efinix Trion */

  trion_configspi(dev->spi);

	/* Send the data */

  while (buflen > 0)
    {
      nbytes = buflen;
      if (nbytes >= TRION_SPI_MAX_XFER)
        {
          nbytes = TRION_SPI_MAX_XFER;
        }

      SPI_SNDBLOCK(dev->spi, buffer, nbytes);

      buffer += nbytes;
      buflen -= nbytes;
    }

  return 0;
}

/****************************************************************************
 * Name: trion_endwrite
 *
 * Description:
 *  End writing bitstream to the Efinix FPGA
 ****************************************************************************/

static int trion_endwrite(FAR struct trion_dev_s *dev)
{
	/* Configure SPI for the Efinix Trion */

  trion_configspi(dev->spi);
  int done = 0;
	int stat = 0;

	/* Validate parameters */

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(dev->spi != NULL);

	/* Check if the FPGA is initialized */

  if (!dev->in_progress)
    {
      spierr("ERROR: FPGA not initialized\n");
      return -EINVAL;
    }

	/* Send dummy cycles */

  for (size_t i = 0; i < TRION_SPI_FINAL_CLK_CYCLES + 7 / 8; i++)
    {
      SPI_SEND(dev->spi, 0xff);
    }

	/* Deselect the device */

  dev->ops->select(dev, false);

	/* Check if CDONE is high */

  done = dev->ops->get_done(dev);
  if (done == 0)
    {
      spierr("ERROR: CDONE not high after writing to FPGA\n");
      SPI_LOCK(dev->spi, false);
      return -ENODEV;
    }

	/* Check if NSTATUS is high */

  stat = dev->ops->get_stat(dev);
  if (stat == 0)
    {
      spierr("ERROR: NSTATUS not high after writing to FPGA\n");
      SPI_LOCK(dev->spi, false);
      return -ENODEV;
    }

	/* Unlock SPI */

  SPI_LOCK(dev->spi, false);

	/* Clear the in_progress flag */

  dev->in_progress = false;

  return OK;
}

/****************************************************************************
 * Name: trion_write
 *
 * Description:
 *  Write buffer to the Efinix FPGA
 ****************************************************************************/

static ssize_t trion_write(FAR struct file *filep, FAR const char *buffer, 
													 size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct trion_dev_s *dev = inode->i_private;
  int ret;

	/* Validate parameters */

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(dev->spi != NULL);
  DEBUGASSERT(buffer != NULL);

	/* Initialize the FPGA */

  if (!dev->in_progress)
    {
			/* Initialize the FPGA */

      ret = trion_init_fpga(dev);
      if (ret < 0)
        {
          spierr("ERROR: Failed to initialize FPGA: %d\n", ret);
          return ret;
        }
    }

	/* Write to the FPGA */

  ret = trion_writeblk(dev, buffer, buflen);
  if (ret < 0)
    {
      spierr("ERROR: Failed to write to FPGA: %d\n", ret);
      return ret;
    }

  return buflen;
}

/****************************************************************************
 * Name: trion_read
 *
 * Description:
 *   Read is ignored.
 ****************************************************************************/

static ssize_t trion_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: trion_ioctl
 *
 * Description:
 *   The only available ICTL is RFIOC_SETATT. It expects a struct
 *   attenuator_control* as the argument to set the attenuation
 *   level. The channel is ignored as the DAT-31R5-SP+ has just a
 *   single attenuator.
 ****************************************************************************/

static int trion_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct trion_dev_s *dev = inode->i_private;
  int ret = OK;

	/* Validate parameters */

  DEBUGASSERT(dev != NULL);

  switch (cmd)
    {
    case FPGAIOC_WRITE_INIT:
      ret = trion_init_fpga(dev);
      break;

    case FPGAIOC_WRITE:
      ret = trion_writeblk(dev, (FAR const char *)arg, sizeof(arg));
      break;

    case FPGAIOC_WRITE_COMPLETE:
      ret = trion_endwrite(dev);
      break;

    default:
      spiinfo("Unrecognized cmd: %d\n", cmd);
      ret = -EINVAL;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trion_register
 *
 * Description:
 *   Register the ice_v character device as 'devpath'.
 *
 ****************************************************************************/

int trion_register(FAR const char *path, FAR struct trion_dev_s *dev)
{
  int ret;

  /* Sanity check */

  DEBUGASSERT(dev != NULL);

  /* Register the character driver */

  ret = register_driver(path, &g_trion_fops, 0666, dev);
  if (ret < 0)
    {
      spierr("ERROR: Failed to register driver: %d\n", ret);
    }

  return ret;
}
