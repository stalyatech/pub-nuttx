/****************************************************************************
 * include/nuttx/sensors/ublox_gps.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_UBLOX_GPS_H
#define __INCLUDE_NUTTX_SENSORS_UBLOX_GPS_H

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
 * Name: ublox_gps_register
 *
 * Description:
 *   u-blox GPS driver entrypoint.
 *
 * Input Parameters:
 *   name    - Serial port name that connected to the GPS device.
 *   baud    - Serial port name baudrate of the GPS device.
 *   devno   - The user specifies which device of this type, from 0.
 *   nbuffer - The number of events that the circular buffer can hold.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ublox_gps_register(FAR const char *name, uint32_t baud, uint32_t devno, uint32_t nbuffer);

#ifdef __cplusplus
}
#endif

#endif
