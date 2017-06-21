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
#include <inttypes.h>
#include <mcu/cortex_m4.h>

/* FIXME: this is declared by mcuboot, but right now both mcuboot
 *        and apps/boot provide the same includes which clash!
 */
struct mcuboot_api_itf;

#if defined(__arm__)
    #define MCUBOOT_API_INIT_(m)                                            \
        extern const struct mcuboot_api_itf mcuboot_api_vt;                 \
        asm("mov r4, %0\n\t"                                                \
            "mov r5, %1"  :: "r" (m), "r" (&mcuboot_api_vt) : "r4", "r5");
#elif defined(__mips__)
    /* TODO */
    #define MCUBOOT_API_INIT_(m)
#elif defined(__i386__)
    /* TODO */
    #define MCUBOOT_API_INIT_(m)
#endif

#define MCUBOOT_API_MAGIC             0xb00710ad
#define MCUBOOT_API_INIT(x)           MCUBOOT_API_INIT_(MCUBOOT_API_MAGIC)

/**
 * Boots the image described by the supplied image header.
 *
 * @param hdr                   The header for the image to boot.
 */
void
hal_system_start(void *img_start)
{
    /* Turn off interrupts. */
    __disable_irq();

    /* Set the VTOR to default. */
    SCB->VTOR = 0;

    // Memory barriers for good measure.
    __ISB();
    __DSB();

    /* First word contains initial MSP value. */
    __set_MSP(*(uint32_t *)img_start);
    __set_PSP(*(uint32_t *)img_start);

    /* Second word contains address of entry point (Reset_Handler). */
    void (*entry)(void) = (void (*)(void))*(uint32_t *)(img_start + 4);

    /* FIXME: this should be a macro which is ARCH specific, as defined in
     * MCUBOOT_API_INIT, but currently mcuboot and the existing boot loader
     * in mynewt conflict. To be fixed soon!
     *
     * This has to be done just before jumping into the app, to avoid
     * overwriting the registers...
     */
    MCUBOOT_API_INIT();

    /* Jump to image. */
    entry();

    /* Should never reach this point */
    while (1)
        ;
}
