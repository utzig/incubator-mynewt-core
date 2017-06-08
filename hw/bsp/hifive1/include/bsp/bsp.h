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
#ifndef __HIFIVE_BSP_H
#define __HIFIVE_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* LED pins */
#define LED_BLINK_PIN       (3)

#define GREEN_LED_PIN       (3)
#define BLUE_LED_PIN        (5)
#define RED_LED_PIN         (6)

#define CONSOLE_UART        "uart0"

extern uint8_t _ram_start;
#define RAM_SIZE        0x8000

static inline int
pin_to_offset(int pin)
{
    return pin < 8 ? pin + 16 : pin - 8;
}

#ifdef __cplusplus
}
#endif

#endif  /* __HIFIVE_BSP_H */
