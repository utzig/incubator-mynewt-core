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
#include <stdlib.h>
#include <assert.h>

#include <syscfg/syscfg.h>

#include <hal/hal_i2c.h>
#include <hal/hal_gpio.h>

#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "fsl_port.h"


#define HAL_I2C_MAX_DEVS    3

//#define I2C_ADDRESS         0xae

struct k64f_hal_i2c {
    I2C_Type *handle;
    int32_t clk_src;
    bool started;
};

#if MYNEWT_VAL(I2C_0)
static struct k64f_hal_i2c i2c0 = {
    .handle = I2C0,
    .clk_src = I2C0_CLK_SRC,
    .started = false,
};
#endif
#if MYNEWT_VAL(I2C_1)
static struct k64f_hal_i2c i2c1 = {
    .handle = I2C1,
    .clk_src = I2C1_CLK_SRC,
    .started = false,
};
#endif
#if MYNEWT_VAL(I2C_2)
static struct k64f_hal_i2c i2c2 = {
    .handle = I2C2,
    .clk_src = I2C2_CLK_SRC,
    .started = false,
};
#endif

static struct k64f_hal_i2c *hal_i2c_devs[HAL_I2C_MAX_DEVS] = {
#if MYNEWT_VAL(I2C_0)
    &i2c0,
#else
    NULL,
#endif
#if MYNEWT_VAL(I2C_1)
    &i2c1,
#else
    NULL,
#endif
#if MYNEWT_VAL(I2C_2)
    &i2c2,
#else
    NULL,
#endif
};

int
hal_i2c_init(uint8_t i2c_num, void *usercfg)
{
    //struct stm32f4_hal_i2c_cfg *cfg = (struct k64f_hal_i2c_cfg *)usercfg;
    struct k64f_hal_i2c *dev;
    //int rc;

    i2c_master_config_t config = {
        .enableMaster = true,
#if defined(FSL_FEATURE_I2C_HAS_HIGH_DRIVE_SELECTION) && FSL_FEATURE_I2C_HAS_HIGH_DRIVE_SELECTION
        .enableHighDrive = false,
#endif
#if defined(FSL_FEATURE_I2C_HAS_STOP_HOLD_OFF) && FSL_FEATURE_I2C_HAS_STOP_HOLD_OFF
        .enableStopHold = false,
#endif
        .baudRate_Bps = 100000,
        .glitchFilterWidth = 0
    };

    if (i2c_num >= HAL_I2C_MAX_DEVS || !(dev = hal_i2c_devs[i2c_num])) {
        return -1;
    }

#if 0
    rc = hal_gpio_init_af(cfg->hic_pin_sda, cfg->hic_pin_af, HAL_GPIO_PULL_UP,
                          1);
    if (rc) {
        goto err;
    }
    rc = hal_gpio_init_af(cfg->hic_pin_scl, cfg->hic_pin_af, HAL_GPIO_PULL_UP,
                          1);
    if (rc) {
        goto err;
    }
#endif

    I2C_MasterInit(dev->handle, &config, dev->clk_src);
    dev->started = false;

    return 0;
}

int
hal_i2c_master_write(uint8_t i2c_num, struct hal_i2c_master_data *pdata,
                     uint32_t timeout, uint8_t last_op)
{
    struct k64f_hal_i2c *dev;
    status_t status = kStatus_Success;

    if (i2c_num >= HAL_I2C_MAX_DEVS || !(dev = hal_i2c_devs[i2c_num])) {
        return -1;
    }

    if (!dev->started) {
        I2C_MasterStart(dev->handle, pdata->address << 1, kI2C_Write);

        /* TODO: timeout */
        do {
            status = I2C_MasterGetStatusFlags(dev->handle);
        } while (!(status & kI2C_IntPendingFlag));

        if (status & kI2C_ReceiveNakFlag) {
            return -1;
        }

        dev->started = true;
    }

    status = I2C_MasterWriteBlocking(dev->handle, pdata->buffer, pdata->len);

    if (status != kStatus_Success || last_op) {
        I2C_MasterStop(dev->handle);
        dev->started = false;
    }

    return (status != kStatus_Success) ? -1 : 0;
}

int
hal_i2c_master_read(uint8_t i2c_num, struct hal_i2c_master_data *pdata,
                    uint32_t timeout, uint8_t last_op)
{
    struct k64f_hal_i2c *dev;
    status_t status = kStatus_Success;

    if (i2c_num >= HAL_I2C_MAX_DEVS || !(dev = hal_i2c_devs[i2c_num])) {
        return -1;
    }

    if (!dev->started) {
        I2C_MasterStart(dev->handle, pdata->address << 1, kI2C_Read);

        /* TODO: timeout */
        do {
            status = I2C_MasterGetStatusFlags(dev->handle);
        } while (!(status & kI2C_IntPendingFlag));

        if (status & kI2C_ReceiveNakFlag) {
            return -1;
        }

        dev->started = true;
    }

    status = I2C_MasterReadBlocking(dev->handle, pdata->buffer, pdata->len);

    if (status != kStatus_Success || last_op) {
        I2C_MasterStop(dev->handle);
        dev->started = false;
    }

    if (status != kStatus_Success) {
        return -1;
    }

    return 0;
}

int
hal_i2c_master_probe(uint8_t i2c_num, uint8_t address, uint32_t timeout)
{
    struct k64f_hal_i2c *dev;
    status_t status = kStatus_Success;

    if (i2c_num >= HAL_I2C_MAX_DEVS || !(dev = hal_i2c_devs[i2c_num])) {
        return -1;
    }

    I2C_MasterStart(dev->handle, address << 1, kI2C_Read);

    /* TODO: timeout */
    do {
        status = I2C_MasterGetStatusFlags(dev->handle);
    } while (!(status & kI2C_IntPendingFlag));

    I2C_MasterStop(dev->handle);

    /* FIXME: is a NAK received when no device is attached? */
    if (status & kI2C_ReceiveNakFlag) {
        return -1;
    }

    return 0;
}
