/****************************************************************************
 * boards/arm/stm32f7/ardusimple-mapkit/src/stm32_platform.c
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
#include <sys/types.h>
#include <stdio.h>

#include "ardusimple-mapkit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: platform_user_verify
 *
 * Description:
 *   If CONFIG_NSH_LOGIN_PLATFORM is defined, then platform-specific logic
 *   must provide this function in order verify user credentials as part of
 *   the login process.
 *
 * Input Parameters:
 *   username/password - User credentials to be verified.
 *
 * Returned value:
 *   1 - The user credentials are verified
 *   0 - The user credentials are incorrect
 *  <0 - An error occurred.  The returned value is a negated errno number.
 *
 ****************************************************************************/

#if defined(CONFIG_NSH_LOGIN_PLATFORM)
int platform_user_verify(FAR const char *username, FAR const char *password)
{
  if ((strncmp(username , "root", 4) == 0) && (strncmp(password , "root", 4) == 0))
    {
      return 1;
    }

  return 0;
}
#endif
