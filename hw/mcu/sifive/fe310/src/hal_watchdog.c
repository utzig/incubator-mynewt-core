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
#include <hal/hal_watchdog.h>
#include <env/freedom-e300-hifive1/platform.h>
#include <mcu/mcu.h>
#include <mcu/fe310_hal.h>

#if !WATCHDOG_RESET
static void watchdog_irq(void)
{
    int cfg = AON_REG(AON_WDOGCFG);
    AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE;
    AON_REG(AON_WDOGCFG) = cfg & ~(AON_WDOGCFG_CMPIP |
                                   AON_WDOGCFG_ENCOREAWAKE |
                                   AON_WDOGCFG_ENALWAYS);
    assert(0);
    while (1) {
    }
}
#endif

int
hal_watchdog_init(uint32_t expire_msecs)
{
    int scale = 0;
    int pending_ints;
    uint64_t expiration = ((uint64_t)expire_msecs * RTC_FREQ) / 1000;
    while (expiration > 65534) {
        expiration >>= 1;
        scale++;
    }

    if (scale > 15) {
        return -1;
    }

#if WATCHDOG_RESET
    AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE;
    AON_REG(AON_WDOGCFG) = AON_WDOGCFG_RSTEN | AON_WDOGCFG_ZEROCMP | scale;
#else
    AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE;
    AON_REG(AON_WDOGCFG) = AON_WDOGCFG_ZEROCMP | scale;
    plic_set_handler(INT_WDOGCMP, watchdog_irq, 7);
    pending_ints = PLIC_REG(PLIC_PENDING_OFFSET);
    /* If wdog interrupt is already pending, clear it */
    if (pending_ints & (1 << INT_WDOGCMP)) {
        int cr;
        int claimed;
        __HAL_DISABLE_INTERRUPTS(cr);
        plic_enable_interrupt(INT_WDOGCMP);
        do {
            claimed = PLIC_REG(PLIC_CLAIM_OFFSET);
            PLIC_REG(PLIC_CLAIM_OFFSET) = claimed;
        } while (claimed != INT_WDOGCMP);

        __HAL_ENABLE_INTERRUPTS(cr);
    } else {
        plic_enable_interrupt(INT_WDOGCMP);
    }
#endif
    AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE;
    AON_REG(AON_WDOGCMP) = (uint32_t)expiration;

    return 0;
}

void
hal_watchdog_enable(void)
{
    AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE;
    AON_REG(AON_WDOGCFG) |= AON_WDOGCFG_ENCOREAWAKE;
}

void
hal_watchdog_tickle(void)
{
    AON_REG(AON_WDOGKEY) = AON_WDOGKEY_VALUE;
    AON_REG(AON_WDOGFEED) = AON_WDOGFEED_VALUE;
}

