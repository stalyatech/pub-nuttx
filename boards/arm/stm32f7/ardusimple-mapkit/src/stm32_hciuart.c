/****************************************************************************
 * boards/arm/stm32f7/ardusimple-mapkit/src/stm32_hciuart.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/wireless/bluetooth/bt_uart_shim.h>

#include "ardusimple-mapkit.h"

#ifdef HAVE_HCIUART

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITTHREAD_PRIORITY
# define CONFIG_HCIDEV_THREAD_PRIORITY  CONFIG_BOARD_INITTHREAD_PRIORITY-1
#else
# define CONFIG_HCIDEV_THREAD_PRIORITY  100
#endif

#ifndef CONFIG_HCIDEV_THREAD_STACKSIZE
# define CONFIG_HCIDEV_THREAD_STACKSIZE 2048
#endif

/****************************************************************************
 * Include the firmware blob(s).
 ****************************************************************************/

#include "../firmware/src/bt_fw.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
/****************************************************************************
 * Name: hcidev_thread
 *
 * Description:
 *   Initialize the Bluetooth HCI UART driver.
 *
 ****************************************************************************/
static pthread_addr_t hcidev_thread(pthread_addr_t arg)
{
  int ret;

  /* Instantiate the HCI UART lower half interface
   * Then initialize the HCI UART upper half driver with the bluetooth stack
   */

  ret = btuart_register(btuart_shim_getdevice(BT_TTY_DEVPATH, 
                                              BT_REG_DEVPATH));
  if (ret < 0)
    {
      wlerr("ERROR: btuart_register() failed: %d\n", ret);
    }

  return NULL;
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hciuart_dev_initialize
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   Bluetooth HCI UART driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int hciuart_dev_initialize(void)
{
#ifdef CONFIG_BOARD_LATE_INITIALIZE
  struct sched_param sparam;
  pthread_attr_t     attr;
  pthread_t          tid;
  int                ret;

  /* Start the network initialization thread to perform the network bring-up
   * asynchronously.
   */

  pthread_attr_init(&attr);
  sparam.sched_priority = CONFIG_HCIDEV_THREAD_PRIORITY;
  pthread_attr_setschedparam(&attr, &sparam);
  pthread_attr_setstacksize(&attr, CONFIG_HCIDEV_THREAD_STACKSIZE);

  wlinfo("Starting hcidev thread\n");
  ret = pthread_create(&tid, &attr, hcidev_thread, NULL);
  if (ret != OK)
    {
      wlerr("ERROR: Failed to create hcidev thread: %d\n", ret);
      hcidev_thread(NULL);
    }
  else
    {
      /* Detach the thread because we will not be joining to it */

      pthread_detach(tid);

      /* Name the thread */

      pthread_setname_np(tid, "hcidev");
    }

  return OK;
#else
  int ret;

  /* Instantiate the HCI UART lower half interface
   * Then initialize the HCI UART upper half driver with the bluetooth stack
   */

  ret = btuart_register(btuart_shim_getdevice(BT_TTY_DEVPATH, 
                                              BT_REG_DEVPATH));
  if (ret < 0)
    {
      wlerr("ERROR: btuart_register() failed: %d\n", ret);
    }

  return ret;
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
}

#endif /* HAVE_HCIUART */
