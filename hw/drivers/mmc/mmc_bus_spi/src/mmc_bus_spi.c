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

#include <syscfg/syscfg.h>
#include <hal/hal_gpio.h>
#include <hal/hal_spi.h>
#include <mmc/mmc_bus.h>

/*
 * NOTE: MMC initialization accepts clocks in the range 100-400KHz, which can
 *       later be switched to high-speeds in the MHz range.
 */

#define SLOW_BAUD MYNEWT_VAL(MMC_SLOW_BAUD)
#define FAST_BAUD MYNEWT_VAL(MMC_FAST_BAUD)

static struct hal_spi_settings mmc_spi_settings = {
    .data_order = HAL_SPI_MSB_FIRST,
    .data_mode  = HAL_SPI_MODE0,
    .baudrate   = 0,
    .word_size  = HAL_SPI_WORD_SIZE_8BIT,
};

struct spi_bus {
    int  num;
    int  ss_pin;
};

static struct spi_bus g_spi;

struct mmc_bus mmc_bus_spi;

int
mmc_spi_init(struct mmc_bus **bus, int spi_num, int ss_pin)
{
    int rc;

    //XXX: should be done by hal_spi_init?
    hal_gpio_init_out(ss_pin, 1);

    // XXX: let this be called by BSP initialization
#if 0
    rc = hal_spi_init(spi_num, &spi_cfg, HAL_SPI_TYPE_MASTER);
    if (rc) {
        return (rc);
    }
#endif

    // TODO: add locking
    mmc_spi_settings.baudrate = SLOW_BAUD;
    rc = hal_spi_config(spi_num, &mmc_spi_settings);
    if (rc) {
        return (rc);
    }

    hal_spi_set_txrx_cb(spi_num, NULL, NULL);
    hal_spi_enable(spi_num);

    g_spi.num = spi_num;
    g_spi.ss_pin = ss_pin;

    mmc_bus_spi.data = &g_spi;
    *bus = &mmc_bus_spi;

    return 0;
}

static int
mmc_spi_fast_clock(struct mmc_bus *bus)
{
    int rc;

    // TODO: add locking
    mmc_spi_settings.baudrate = FAST_BAUD;
    rc = hal_spi_config(((struct spi_bus *)bus->data)->num, &mmc_spi_settings);
    if (rc) {
        return (rc);
    }
    return 0;
}

static void
mmc_spi_enable(struct mmc_bus *bus)
{
    assert(bus && bus->data);
    hal_gpio_write(((struct spi_bus *)bus->data)->ss_pin, 0);
}

static void
mmc_spi_disable(struct mmc_bus *bus)
{
    assert(bus && bus->data);
    hal_gpio_write(((struct spi_bus *)bus->data)->ss_pin, 1);
}

static uint8_t
mmc_spi_txrx_byte(struct mmc_bus *bus, uint8_t by)
{
    assert(bus && bus->data);
    return hal_spi_tx_val(((struct spi_bus *)bus->data)->num, by);
}

struct mmc_bus mmc_bus_spi = {
    .fast_clock = mmc_spi_fast_clock,
    .enable     = mmc_spi_enable,
    .disable    = mmc_spi_disable,
    .txrx_byte  = mmc_spi_txrx_byte,
    .data       = NULL,
};
