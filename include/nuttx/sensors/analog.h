/****************************************************************************
 * include/nuttx/sensors/analog.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_ANALOG_H
#define __INCLUDE_NUTTX_SENSORS_ANALOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

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

int analog_uorb_register(uint32_t devno, uint32_t nbuffer);

#ifdef __cplusplus
}
#endif

#endif
