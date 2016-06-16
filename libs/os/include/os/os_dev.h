/**
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

#ifndef _OS_DEV_H
#define _OS_DEV_H

#include "os/queue.h"

/*
 * Initialization order, defines when a device should be initialized
 * by the Mynewt kernel.
 *
 */
#define OS_DEV_INIT_PRIMARY   (1)
#define OS_DEV_INIT_SECONDARY (2)
#define OS_DEV_INIT_KERNEL    (3)

typedef int (*os_dev_init_func_t)(void *arg);

#define OS_DEV_INIT_F_CRITICAL (1 << 0)

/*
 * Device structure.
 *
 */
struct os_dev {
    char *od_name;
    uint8_t od_stage;
    uint8_t od_priority;
    uint8_t od_init_flags;
    uint8_t __pad1;
    void *od_arg;
    os_dev_init_func_t od_init;
    STAILQ_ENTRY(os_dev) od_next;
};

int os_dev_add(struct os_dev *);
int os_dev_init(uint8_t stage);
struct os_dev *os_dev_lookup(char *name);

#endif /* _OS_DEV_H */
