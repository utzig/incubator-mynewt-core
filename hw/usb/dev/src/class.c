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

#include <os/os.h>
#include <usb/usb.h>
#include <dev/dev.h>
#include <dev/ch9.h>
#include <dev/class.h>

#include <hal_usb/hal_usb.h>

#if MYNEWT_VAL(USB_DEVICE_CONFIG_NUM)

#if MYNEWT_VAL(USB_DEVICE_CONFIG_HID)
#include <hid/hid.h>
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_CDC_ACM)
#include <cdc/cdc.h>
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_MSC)
#include <msc/msc.h>
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_AUDIO)
#include "usb_device_audio.h"
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_PHDC)
#include "usb_device_phdc.h"
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_VIDEO)
#include "usb_device_video.h"
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_PRINTER)
#include "usb_device_printer.h"
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_DFU)
#include "usb_device_dfu_config.h"
#include "usb_device_dfu.h"
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_CCID)
#include "usb_device_ccid.h"
#endif

/*
 * FIXME: this should be an slist of similar! class drivers
 * register themselves
 */
static const usb_device_class_map_t s_UsbDeviceClassInterfaceMap[] = {
#if MYNEWT_VAL(USB_DEVICE_CONFIG_HID)
    {
        .init   = usb_dev_hid_init,
        .deinit = usb_dev_hid_deinit,
        .cb     = usb_dev_hid_event,
        .type   = kUSB_DeviceClassTypeHid,
    },
#endif
#if MYNEWT_VAL(USB_DEVICE_CONFIG_CDC_ACM)
    {
        .init   = usb_dev_cdc_init,
        .deinit = usb_dev_cdc_deinit,
        .cb     = usb_dev_cdc_event,
        .type   = kUSB_DeviceClassTypeCdc,
    },
#endif
#if MYNEWT_VAL(USB_DEVICE_CONFIG_MSC)
    {
        .init   = usb_dev_msc_init,
        .deinit = usb_dev_msc_deinit,
        .cb     = usb_dev_msc_event,
        .type   = kUSB_DeviceClassTypeMsc,
    },
#endif
#if MYNEWT_VAL(USB_DEVICE_CONFIG_AUDIO)
    {
        .init   = USB_DeviceAudioInit,
        .deinit = USB_DeviceAudioDeinit,
        .cb     = USB_DeviceAudioEvent,
        .type   = kUSB_DeviceClassTypeAudio,
    },
#endif
#if MYNEWT_VAL(USB_DEVICE_CONFIG_PHDC)
    {
        .init   = USB_DevicePhdcInit,
        .deinit = USB_DevicePhdcDeinit,
        .cb     = USB_DevicePhdcEvent,
        .type   = kUSB_DeviceClassTypePhdc
    },
#endif
#if MYNEWT_VAL(USB_DEVICE_CONFIG_VIDEO)
    {
        .init   = USB_DeviceVideoInit,
        .deinit = USB_DeviceVideoDeinit,
        .cb     = USB_DeviceVideoEvent,
        .type   = kUSB_DeviceClassTypeVideo
    },
#endif
#if MYNEWT_VAL(USB_DEVICE_CONFIG_PRINTER)
    {
        .init   = USB_DevicePrinterInit,
        .deinit = USB_DevicePrinterDeinit,
        .cb     = USB_DevicePrinterEvent,
        .type   = kUSB_DeviceClassTypePrinter,
    },
#endif
#if MYNEWT_VAL(USB_DEVICE_CONFIG_DFU)
    {
        .init   = usb_device_dfu_init,
        .deinit = usb_device_dfu_deinit,
        .cb     = usb_device_dfu_event,
        .type   = kUSB_DeviceClassTypeDfu
    },
#endif
#if MYNEWT_VAL(USB_DEVICE_CONFIG_CCID)
    {
        .init   = USB_DeviceCcidInit,
        .deinit = USB_DeviceCcidDeinit,
        .cb     = USB_DeviceCcidEvent,
        .type   = kUSB_DeviceClassTypeCcid
    },
#endif
    {
        .init   = NULL,
        .deinit = NULL,
        .cb     = NULL,
        .type   = 0,
    },
};

static usb_device_common_class_t s_UsbDeviceCommonClassStruct[MYNEWT_VAL(USB_DEVICE_CONFIG_NUM)];

static int
usb_device_class_alloc_handle(uint8_t ctrl_id, usb_device_common_class_t **handle)
{
    int i;
    int err = USB_BUSY;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);

    /* Check the controller is initialized or not. */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDeviceCommonClassStruct[i].handle &&
            s_UsbDeviceCommonClassStruct[i].ctrl_id == ctrl_id) {
            err = USB_ERR;
            goto out;
        }
    }

    /* Get a free common class handle. */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (!s_UsbDeviceCommonClassStruct[i].handle) {
            s_UsbDeviceCommonClassStruct[i].ctrl_id = ctrl_id;
            *handle = &s_UsbDeviceCommonClassStruct[i];
            err = 0;
            goto out;
        }
    }

out:
    OS_EXIT_CRITICAL(sr);
    return err;
}

static int
usb_device_class_free_handle(uint8_t ctrl_id)
{
    int i;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);

    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDeviceCommonClassStruct[i].handle &&
            s_UsbDeviceCommonClassStruct[i].ctrl_id == ctrl_id) {
            s_UsbDeviceCommonClassStruct[i].handle = NULL;
            s_UsbDeviceCommonClassStruct[i].configs = NULL;
            s_UsbDeviceCommonClassStruct[i].ctrl_id = 0;
            OS_EXIT_CRITICAL(sr);
            return 0;
        }
    }

    OS_EXIT_CRITICAL(sr);
    return USB_INVALID_PARAM;
}

static int
usb_device_class_handle_by_id(uint8_t ctrl_id, usb_device_common_class_t **handle)
{
    int i;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDeviceCommonClassStruct[i].handle &&
            s_UsbDeviceCommonClassStruct[i].ctrl_id == ctrl_id) {
            *handle = &s_UsbDeviceCommonClassStruct[i];
            OS_EXIT_CRITICAL(sr);
            return 0;
        }
    }

    OS_EXIT_CRITICAL(sr);
    return USB_INVALID_PARAM;
}

static int
usb_device_class_handle_by_device(usb_device_handle device,
                                  usb_device_common_class_t **handle)
{
    int i;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDeviceCommonClassStruct[i].handle == device) {
            *handle = &s_UsbDeviceCommonClassStruct[i];
            OS_EXIT_CRITICAL(sr);
            return 0;
        }
    }

    OS_EXIT_CRITICAL(sr);
    return USB_INVALID_PARAM;
}

int
usb_device_class_get_handle(uint8_t ctrl_id, usb_device_handle *handle)
{
    int32_t i;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDeviceCommonClassStruct[i].handle &&
            s_UsbDeviceCommonClassStruct[i].ctrl_id == ctrl_id) {
            *handle = s_UsbDeviceCommonClassStruct[i].handle;
            OS_EXIT_CRITICAL(sr);
            return 0;
        }
    }
    OS_EXIT_CRITICAL(sr);
    return USB_INVALID_PARAM;
}

int
usb_device_class_event(usb_device_handle handle, usb_device_class_event_t event, void *param)
{
    usb_device_common_class_t *classHandle;
    uint8_t mapIndex;
    uint8_t mapLen;
    uint8_t classIndex;
    int rc = USB_ERR;
    int err = USB_ERR;

    if (!param) {
        return USB_INVALID_PARAM;
    }

    /* Get the common class handle according to the device handle. */
    rc = usb_device_class_handle_by_device(handle, &classHandle);
    if (rc) {
        return USB_INVALID_PARAM;
    }

    mapLen = sizeof(s_UsbDeviceClassInterfaceMap) / sizeof(s_UsbDeviceClassInterfaceMap[0]);
    for (classIndex = 0; classIndex < classHandle->configs->count; classIndex++) {
        for (mapIndex = 0; mapIndex < mapLen; mapIndex++) {
            if (s_UsbDeviceClassInterfaceMap[mapIndex].type ==
                classHandle->configs->config[classIndex].info->type) {
                rc = s_UsbDeviceClassInterfaceMap[mapIndex].cb(
                    (void *)classHandle->configs->config[classIndex].handle,
                    event, param);
                if (rc == USB_INVALID_REQ) {
                    return USB_INVALID_REQ;
                } else if (!rc) {
                    /* For composite device, it should return kStatus_USB_Success
                     * once a valid request has been handled
                     */
                    err = 0;
                }
                break;
            }
        }
    }

    return err;
}

int
usb_device_class_cb(usb_device_handle handle, uint32_t event, void *param)
{
    usb_device_common_class_t *classHandle;
    int err = USB_ERR;

    err = usb_device_class_handle_by_device(handle, &classHandle);
    if (err) {
        return err;
    }

    if (event == kUSB_DeviceEventBusReset) {
        usb_device_control_pipe_init(handle, classHandle);
        usb_device_class_event(handle, kUSB_DeviceClassEventDeviceReset, classHandle);
    }

    return classHandle->configs->deviceCallback(handle, event, param);
}

int
usb_device_class_init(uint8_t ctrl_id, usb_dev_class_configs_t *configs,
        usb_device_handle *handle)
{
    usb_device_common_class_t *classHandle;
    int err = USB_ERR;
    uint8_t mapIndex;
    uint8_t mapLen;
    uint8_t classIndex;

    if (!handle || !configs || !configs->deviceCallback) {
        return USB_INVALID_PARAM;
    }

    err = usb_device_class_alloc_handle(ctrl_id, &classHandle);
    if (err) {
        return err;
    }

    classHandle->configs = configs;
    err = usb_dev_init(ctrl_id, usb_device_class_cb, &classHandle->handle);
    if (err) {
        usb_device_deinit(classHandle->handle);
        usb_device_class_free_handle(ctrl_id);
        return err;
    }

    //FIXME: refactor this is because it's the same loop of .cb calling fn ...
    mapLen = sizeof(s_UsbDeviceClassInterfaceMap) / sizeof(usb_device_class_map_t);
    for (classIndex = 0; classIndex < classHandle->configs->count; classIndex++) {
        for (mapIndex = 0; mapIndex < mapLen; mapIndex++) {
            if (classHandle->configs->config[classIndex].info->type ==
                s_UsbDeviceClassInterfaceMap[mapIndex].type) {
                (void)s_UsbDeviceClassInterfaceMap[mapIndex].init(
                    ctrl_id, &classHandle->configs->config[classIndex],
                    &classHandle->configs->config[classIndex].handle);
            }
        }
    }

    *handle = classHandle->handle;
    return err;
}

int
usb_device_class_deinit(uint8_t ctrl_id)
{
    usb_device_common_class_t *classHandle;
    int err = USB_ERR;
    uint8_t mapIndex;
    uint8_t mapLen;
    uint8_t classIndex;

    err = usb_device_class_handle_by_id(ctrl_id, &classHandle);
    if (err) {
        return err;
    }

    //FIXME: refactor...
    mapLen = sizeof(s_UsbDeviceClassInterfaceMap) / sizeof(usb_device_class_map_t);
    for (classIndex = 0; classIndex < classHandle->configs->count; classIndex++) {
        for (mapIndex = 0; mapIndex < mapLen; mapIndex++) {
            if (classHandle->configs->config[classIndex].info->type ==
                s_UsbDeviceClassInterfaceMap[mapIndex].type) {
                (void)s_UsbDeviceClassInterfaceMap[mapIndex].deinit(
                    classHandle->configs->config[classIndex].handle);
            }
        }
    }

    err = usb_device_deinit(classHandle->handle);
    if (!err) {
        (void)usb_device_class_free_handle(ctrl_id);
    }

    return err;
}

int
usb_device_class_get_speed(uint8_t ctrl_id, uint8_t *speed)
{
    usb_device_common_class_t *classHandle;
    int err = USB_ERR;

    err = usb_device_class_handle_by_id(ctrl_id, &classHandle);
    if (!err) {
        return err;
    }

    return usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusSpeed, speed);
}

#endif /* MYNEWT_VAL(USB_DEVICE_CONFIG_NUM) */
