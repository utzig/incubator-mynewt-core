/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <string.h>
#include <inttypes.h>

#include "syscfg/syscfg.h"
#include "sysflash/sysflash.h"
#include "hal/hal_bsp.h"
#include "hal/hal_flash.h"
#include "flash_map/flash_map.h"
#include "os/os.h"
#include "bootutil/image.h"
#include "bootutil/bootutil.h"
#include "bootutil_priv.h"

//const uint32_t BOOTAPI_MAGIC = 0xdeadbeef;

static int
_flash_map_size(int *amount)
{
    *amount = sizeof sysflash_map_dflt / sizeof sysflash_map_dflt[0];
    return 0;
}

static int
_flash_map_info(int index, int *device, uint32_t *offset, uint32_t *size)
{
    return 0;
}

__attribute__((section(".bootapi_vt")))
const struct boot_itf bootapi_vt = {
    .bootapi_magic      = 0xbaadf00d,
    .flash_map_size     = _flash_map_size,
    .flash_map_info     = _flash_map_info,
    .bootapi_version    = BOOTAPI_VERSION(0),
};

struct boot_itf *p_bootapi_vt = NULL;
