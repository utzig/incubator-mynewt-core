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

#include <stddef.h>
#include <stdint.h>
#include <errno.h>
#include <sys/types.h>
#include <stdio.h>
#include "os/mynewt.h"
#include "bsp/bsp.h"
#include "hal/hal_bsp.h"
#include "hal/hal_flash_int.h"
#include "flash_map/flash_map.h"
#include "hal/hal_flash.h"
#if MYNEWT_VAL(UART_0) || MYNEWT_VAL(UART_1) || MYNEWT_VAL(UART_2) || \
    MYNEWT_VAL(UART_3) || MYNEWT_VAL(UART_4) || MYNEWT_VAL(UART_5)
#include "uart/uart.h"
#include "uart_hal/uart_hal.h"
#include "hal/hal_uart.h"
#endif
#include "clock_config.h"

#if MYNEWT_VAL(UART_0)
static struct uart_dev os_bsp_uart0;
#endif

/*
 * What memory to include in coredump.
 */
static const struct hal_bsp_mem_dump dump_cfg[] = {
    [0] = {
        .hbmd_start = &__DATA_ROM,
        .hbmd_size = RAM_SIZE
    }
};

static void init_hardware(void)
{
}

const struct hal_flash *
hal_bsp_flash_dev(uint8_t id)
{
    if (id == 0) {
        //FIXME
        //return &rv32m1_flash_dev;
        //assert(0);
    }
    return NULL;
}

const struct hal_bsp_mem_dump *
hal_bsp_core_dump(int *area_cnt)
{
    *area_cnt = sizeof(dump_cfg) / sizeof(dump_cfg[0]);
    return dump_cfg;
}

int
hal_bsp_power_state(int state)
{
    return (0);
}

/**
 * Returns the configured priority for the given interrupt. If no priority
 * configured, return the priority passed in
 *
 * @param irq_num
 * @param pri
 *
 * @return uint32_t
 */
uint32_t
hal_bsp_get_nvic_priority(int irq_num, uint32_t pri)
{
    /* Add any interrupt priorities configured by the bsp here */
    return pri;
}

void
hal_bsp_init(void)
{
    int rc = 0;

    (void)rc;

    /* Init pinmux and other hardware setup. */
    init_hardware();
    BOARD_BootClockRUN();

#if MYNEWT_VAL(UART_0)
    rc = os_dev_create((struct os_dev *) &os_bsp_uart0, "uart0",
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, NULL);
    assert(rc == 0);
#endif
}
