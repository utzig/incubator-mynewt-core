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

#include "os/os.h"
#include "os/queue.h"
#include "os/os_dev.h"

#include <string.h>

static STAILQ_HEAD(, os_dev) g_os_dev_list;

/**
 * Add this device to the kernel.  This function adds the
 * device to the OS
 */
int
os_dev_add(struct os_dev *dev)
{
    struct os_dev *cur_dev;

    /* Add devices to the list, sorted first by stage, then by
     * priority.
     */
    cur_dev = NULL;
    STAILQ_FOREACH(cur_dev, &g_os_dev_list, od_next) {
        if (cur_dev->od_stage > dev->od_stage) {
            continue;
        }

        if (dev->od_priority >= cur_dev->od_priority) {
            break;
        }
    }

    if (cur_dev) {
        STAILQ_INSERT_AFTER(&g_os_dev_list, cur_dev, dev, od_next);
    } else {
        STAILQ_INSERT_TAIL(&g_os_dev_list, dev, od_next);
    }

    return (0);
}

int
os_dev_init(uint8_t stage)
{
    struct os_dev *dev;
    int rc;

    STAILQ_FOREACH(dev, &g_os_dev_list, od_next) {
        if (dev->od_stage == stage) {
            rc = dev->od_init(dev->od_arg);
            if (dev->od_init_flags & OS_DEV_INIT_F_CRITICAL &&
                    rc != 0) {
                goto err;
            }
        }
    }

    return (0);
err:
    return (rc);
}

struct os_dev *
os_dev_lookup(char *name)
{
    struct os_dev *dev;

    dev = NULL;
    STAILQ_FOREACH(dev, &g_os_dev_list, od_next) {
        if (!strcmp(dev->od_name, name)) {
            break;
        }
    }
    return (dev);
}


