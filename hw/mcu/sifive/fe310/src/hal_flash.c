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
#include <mcu/platform.h>

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

#define FLASH_CMD_READ_STATUS_REGISTER 0x05
#define FLASD_CMD_WRITE_ENABLE 0x06
#define FLASH_CMD_PAGE_PROGRAM 0x02
#define FLASH_CMD_SECTOR_ERASE 0x20

#define FLASH_STATUS_BUSY 0x01
#define FLASH_STATUS_WEN  0x02

static int
e300_flash_read(const struct hal_flash *dev, uint32_t address, void *dst,
        uint32_t num_bytes)
{
    memcpy(dst, (void *)address, num_bytes);
    return 0;
}

static int __attribute((section(".data.e300_flash_transmit")))
e300_flash_transmit(uint8_t out_byte)
{
    int in_byte;

    /* Empty RX fifo */
    while ((int)SPI0_REG(SPI_REG_RXFIFO) >= 0) {
    }
    SPI0_REG(SPI_REG_TXFIFO) = out_byte;
    do {
         in_byte = SPI0_REG(SPI_REG_RXFIFO);
    } while (in_byte < 0);

    return in_byte;
}

static int __attribute((section(".data.e300_flash_wait_till_ready")))
e300_flash_wait_till_ready(void)
{
    int status;
    do {
        SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
        e300_flash_transmit(FLASH_CMD_READ_STATUS_REGISTER);
        /* Wait for ready */
        status = e300_flash_transmit(0xFF);
        SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_AUTO;
    } while (status & FLASH_STATUS_BUSY);

    return 0;
}

static int __attribute((section(".data.e300_flash_write_enable")))
e300_flash_write_enable(void)
{
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
    e300_flash_transmit(FLASD_CMD_WRITE_ENABLE);
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_AUTO;
    return 0;
}

static int  __attribute((section(".data.e300_flash_write_page"))) __attribute((noinline))
e300_flash_write_page(const struct hal_flash *dev, uint32_t address,
                      const void *src, uint32_t num_bytes)
{
    int sr;
    const uint8_t *p = src;
    __HAL_DISABLE_INTERRUPTS(sr);

    /* Diable auto mode */
    SPI0_REG(SPI_REG_FCTRL) = 0;
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
    SPI0_REG(SPI_REG_FMT) &= ~SPI_FMT_DIR(SPI_DIR_TX);

    e300_flash_wait_till_ready();
    e300_flash_write_enable();

    /* Page program */
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
    e300_flash_transmit(FLASH_CMD_PAGE_PROGRAM);
    e300_flash_transmit(address >> 16);
    e300_flash_transmit(address >> 8);
    e300_flash_transmit(address);
    while (num_bytes--) {
        e300_flash_transmit(*p++);
    }
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_AUTO;

    /* Wait for ready */
    e300_flash_wait_till_ready();

    /* Enable auto mode */
    SPI0_REG(SPI_REG_FCTRL) = 1;

    __HAL_ENABLE_INTERRUPTS(sr);
    return 0;
}

static int
e300_flash_write(const struct hal_flash *dev, uint32_t address,
        const void *src, uint32_t num_bytes)
{
    const int page_size = 256;
    uint32_t page_end;
    uint32_t chunk;

    while (num_bytes > 0) {
        page_end = (address + page_size) & ~(page_size - 1);
        if (address + num_bytes < page_end) {
            page_end = address + num_bytes;
        }
        chunk = page_end - address;
        if (e300_flash_write_page(dev, address, src, chunk) < 0) {
            return -1;
        }
        address += chunk;
        num_bytes -= chunk;
        src += chunk;
    }
    return 0;
}

static int __attribute((section(".data.e300_flash_erase_sector"))) __attribute((noinline))
e300_flash_erase_sector(const struct hal_flash *dev, uint32_t sector_address)
{
    int sr;
    __HAL_DISABLE_INTERRUPTS(sr);

    /* Diable auto mode */
    SPI0_REG(SPI_REG_FCTRL) = 0;
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
    SPI0_REG(SPI_REG_FMT) &= ~SPI_FMT_DIR(SPI_DIR_TX);

    e300_flash_wait_till_ready();
    e300_flash_write_enable();

    /* Erase sector */
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
    e300_flash_transmit(FLASH_CMD_SECTOR_ERASE);
    e300_flash_transmit(sector_address >> 16);
    e300_flash_transmit(sector_address >> 8);
    e300_flash_transmit(sector_address);
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_AUTO;

    /* Wait for ready */
    e300_flash_wait_till_ready();

    /* Enable auto mode */
    SPI0_REG(SPI_REG_FCTRL) = 1;

    __HAL_ENABLE_INTERRUPTS(sr);

    return 0;
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
