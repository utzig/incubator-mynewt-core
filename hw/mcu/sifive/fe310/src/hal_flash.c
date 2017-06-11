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

#include <string.h>
#include <assert.h>
#include "mcu/fe310_hal.h"
#include <hal/hal_flash_int.h>

#define E300_FLASH_SECTOR_SZ    4096

static int e300_flash_read(const struct hal_flash *dev, uint32_t address,
        void *dst, uint32_t num_bytes);
static int e300_flash_write(const struct hal_flash *dev, uint32_t address,
        const void *src, uint32_t num_bytes);
static int e300_flash_erase_sector(const struct hal_flash *dev,
        uint32_t sector_address);
static int e300_flash_sector_info(const struct hal_flash *dev, int idx,
        uint32_t *address, uint32_t *sz);
static int e300_flash_init(const struct hal_flash *dev);

static const struct hal_flash_funcs e300_flash_funcs = {
    .hff_read = e300_flash_read,
    .hff_write = e300_flash_write,
    .hff_erase_sector = e300_flash_erase_sector,
    .hff_sector_info = e300_flash_sector_info,
    .hff_init = e300_flash_init
};

const struct hal_flash e300_flash_dev = {
    .hf_itf = &e300_flash_funcs,
    .hf_base_addr = 0x20000000,
    .hf_size = 8 * 1024 * 1024,  /* XXX read from factory info? */
    .hf_sector_cnt = 4096,       /* XXX read from factory info? */
    .hf_align = 1
};

static int
e300_flash_read(const struct hal_flash *dev, uint32_t address, void *dst,
        uint32_t num_bytes)
{
    memcpy(dst, (void *)address, num_bytes);
    return 0;
}

/*
 * Flash write is done by writing 4 bytes at a time at a word boundary.
 */
static int
e300_flash_write(const struct hal_flash *dev, uint32_t address,
        const void *src, uint32_t num_bytes)
{
    return -1;
}

static int
e300_flash_erase_sector(const struct hal_flash *dev, uint32_t sector_address)
{
    return -1;
}

static int
e300_flash_sector_info(const struct hal_flash *dev, int idx,
        uint32_t *address, uint32_t *sz)
{
    assert(idx < e300_flash_dev.hf_sector_cnt);
    *address = dev->hf_base_addr + idx * E300_FLASH_SECTOR_SZ;
    *sz = E300_FLASH_SECTOR_SZ;
    return 0;
}

static int
e300_flash_init(const struct hal_flash *dev)
{
    return 0;
}
