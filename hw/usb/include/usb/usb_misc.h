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
/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __USB_MISC_H__
#define __USB_MISC_H__

#include <syscfg/syscfg.h>

#if !MYNEWT_VAL(MCU_LITTLE_ENDIAN) && !MYNEWT_VAL(MCU_BIG_ENDIAN)
#error "USB requires defining the the MCU endianess"
#endif

#if MYNEWT_VAL(MCU_LITTLE_ENDIAN) && MYNEWT_VAL(MCU_BIG_ENDIAN)
#error "USB requires defining the one MCU endianess"
#endif

#ifndef STRUCT_PACKED
#define STRUCT_PACKED
#endif

#ifndef STRUCT_UNPACKED
#define STRUCT_UNPACKED __attribute__((__packed__))
#endif

#define USB_SHORT_GET_LOW(x) (((uint16_t)x) & 0xFF)
#define USB_SHORT_GET_HIGH(x) ((uint8_t)(((uint16_t)x) >> 8) & 0xFF)

#define USB_LONG_GET_BYTE0(x) ((uint8_t)(((uint32_t)(x))) & 0xFF)
#define USB_LONG_GET_BYTE1(x) ((uint8_t)(((uint32_t)(x)) >> 8) & 0xFF)
#define USB_LONG_GET_BYTE2(x) ((uint8_t)(((uint32_t)(x)) >> 16) & 0xFF)
#define USB_LONG_GET_BYTE3(x) ((uint8_t)(((uint32_t)(x)) >> 24) & 0xFF)

#define USB_MEM4_ALIGN_MASK 0x03

#define USB_MEM4_ALIGN(n) ((n + 3) & (0xFFFFFFFC))
#define USB_MEM32_ALIGN(n) ((n + 31) & (0xFFFFFFE0))
#define USB_MEM64_ALIGN(n) ((n + 63) & (0xFFFFFFC0))

#define SWAP2BYTE_CONST(n) ((((n) & 0x00FF) << 8) | (((n) & 0xFF00) >> 8))
#define SWAP4BYTE_CONST(n) \
    ((((n) & 0x000000FF) << 24) | (((n) & 0x0000FF00) << 8) | \
     (((n) & 0x00FF0000) >> 8) | (((n) & 0xFF000000) >> 24))

#if MYNEWT_VAL(MCU_BIG_ENDIAN)

#define USB_SHORT_TO_LITTLE_ENDIAN(n)   SWAP2BYTE_CONST(n)
#define USB_LONG_TO_LITTLE_ENDIAN(n)    SWAP4BYTE_CONST(n)
#define USB_SHORT_FROM_LITTLE_ENDIAN(n) SWAP2BYTE_CONST(n)
#define USB_LONG_FROM_LITTLE_ENDIAN(n)  SWAP2BYTE_CONST(n)

#define USB_SHORT_TO_BIG_ENDIAN(n)      (n)
#define USB_LONG_TO_BIG_ENDIAN(n)       (n)
#define USB_SHORT_FROM_BIG_ENDIAN(n)    (n)
#define USB_LONG_FROM_BIG_ENDIAN(n)     (n)

#define USB_LONG_TO_LITTLE_ENDIAN_ADDRESS(n, m) \
    {                                           \
        m[0] = ((n >> 24) & 0xFF);              \
        m[1] = ((n >> 16) & 0xFF);              \
        m[2] = ((n >> 8) & 0xFF);               \
        m[3] = (n & 0xFF);                      \
    }

#define USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(n) \
    ((uint32_t)(((uint32_t)n[0] << 24U) | ((uint32_t)n[1] << 16U) | \
                ((uint32_t)n[2] << 8U) | ((uint32_t)n[3] << 0U)))

#define USB_LONG_TO_BIG_ENDIAN_ADDRESS(n, m) \
    {                                        \
        m[3] = ((n >> 24U) & 0xFFU);         \
        m[2] = ((n >> 16U) & 0xFFU);         \
        m[1] = ((n >> 8U) & 0xFFU);          \
        m[0] = (n & 0xFFU);                  \
    }

#define USB_LONG_FROM_BIG_ENDIAN_ADDRESS(n) \
    ((uint32_t)(((uint32_t)n[3] << 24U) | ((uint32_t)n[2] << 16U) | \
                ((uint32_t)n[1] << 8U) | ((uint32_t)n[0] << 0U)))

#define USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(n, m) \
    {                                            \
        m[0] = ((n >> 8U) & 0xFFU);              \
        m[1] = (n & 0xFFU);                      \
    }

#define USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(n) ((uint32_t)(((uint32_t)n[0] << \
    8U) | ((uint32_t)n[1] << 0U)))

#define USB_SHORT_TO_BIG_ENDIAN_ADDRESS(n, m) \
    {                                         \
        m[1] = ((n >> 8U) & 0xFFU);           \
        m[0] = (n & 0xFFU);                   \
    }

#define USB_SHORT_FROM_BIG_ENDIAN_ADDRESS(n) ((uint32_t)(((uint32_t)n[1] << \
    8U) | ((uint32_t)n[0] << 0U)))

#else /* MYNEWT_VAL(MCU_BIG_ENDIAN) */

#define USB_SHORT_TO_LITTLE_ENDIAN(n) (n)
#define USB_LONG_TO_LITTLE_ENDIAN(n) (n)
#define USB_SHORT_FROM_LITTLE_ENDIAN(n) (n)
#define USB_LONG_FROM_LITTLE_ENDIAN(n) (n)

#define USB_SHORT_TO_BIG_ENDIAN(n) SWAP2BYTE_CONST(n)
#define USB_LONG_TO_BIG_ENDIAN(n) SWAP4BYTE_CONST(n)
#define USB_SHORT_FROM_BIG_ENDIAN(n) SWAP2BYTE_CONST(n)
#define USB_LONG_FROM_BIG_ENDIAN(n) SWAP4BYTE_CONST(n)

#define USB_LONG_TO_LITTLE_ENDIAN_ADDRESS(n, m) \
    {                                           \
        m[3] = ((n >> 24U) & 0xFFU);            \
        m[2] = ((n >> 16U) & 0xFFU);            \
        m[1] = ((n >> 8U) & 0xFFU);             \
        m[0] = (n & 0xFFU);                     \
    }

#define USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(n) \
    ((uint32_t)(((uint32_t)n[3] << 24) | ((uint32_t)n[2] << 16) | \
                ((uint32_t)n[1] << 8) | ((uint32_t)n[0] << 0)))

#define USB_LONG_TO_BIG_ENDIAN_ADDRESS(n, m) \
    {                                        \
        m[0] = ((n >> 24) & 0xFF);           \
        m[1] = ((n >> 16) & 0xFF);           \
        m[2] = ((n >> 8) & 0xFF);            \
        m[3] = (n & 0xFF);                   \
    }

#define USB_LONG_FROM_BIG_ENDIAN_ADDRESS(n) \
    ((uint32_t)(((uint32_t)n[0] << 24U) | ((uint32_t)n[1] << 16U) | \
                ((uint32_t)n[2] << 8U) | ((uint32_t)n[3] << 0U)))

#define USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(n, m)  \
    do {                                          \
        m[1] = ((n >> 8U) & 0xFFU);               \
        m[0] = (n & 0xFFU);                       \
    } while (0)

#define USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(n)  \
    ((uint32_t)(((uint32_t)n[1] << 8U) | ((uint32_t)n[0] << 0U)))

#define USB_SHORT_TO_BIG_ENDIAN_ADDRESS(n, m) \
    {                                         \
        m[0] = ((n >> 8U) & 0xFFU);           \
        m[1] = (n & 0xFFU);                   \
    }

#define USB_SHORT_FROM_BIG_ENDIAN_ADDRESS(n)  \
    ((uint32_t)(((uint32_t)n[0] << 8U) | ((uint32_t)n[1] << 0U)))

#endif

#endif /* __USB_MISC_H__ */
