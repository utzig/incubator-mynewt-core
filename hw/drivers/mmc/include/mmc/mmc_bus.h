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

#ifndef __MMC_BUS_H__
#define __MMC_BUS_H__

#include "os/mynewt.h"

#ifdef __cplusplus
extern "C" {
#endif

struct mmc_bus {
    int     (*fast_clock)(struct mmc_bus *bus);
    void    (*disable)(struct mmc_bus *bus);
    void    (*enable)(struct mmc_bus *bus);
    uint8_t (*txrx_byte)(struct mmc_bus *bus, uint8_t b);
    void    *data;
};

int mmc_spi_init(struct mmc_bus **bus, int spi_num, int ss_pin);

#ifdef __cplusplus
}
#endif

#endif /* __MMC_BUS_H__ */
