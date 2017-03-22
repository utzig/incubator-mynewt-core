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
#include <device/device.h>
#include <device/ch9.h>
#include <device/class.h>

#include <hal_usb/hal_usb.h>

#if MYNEWT_VAL(USB_DEVICE_CONFIG_NUM)

#if MYNEWT_VAL(USB_DEVICE_CONFIG_HID)
#include "usb_device_hid.h"
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_CDC_ACM)
#include <cdc/cdc.h>
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_MSC)
#include "usb_device_msc.h"
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
        .init   = USB_DeviceHidInit,
        .deinit = USB_DeviceHidDeinit,
        .cb     = USB_DeviceHidEvent,
        .type   = kUSB_DeviceClassTypeHid,
    },
#endif
#if MYNEWT_VAL(USB_DEVICE_CONFIG_CDC_ACM)
    {
        .init   = usb_device_cdc_init,
        .deinit = usb_device_cdc_deinit,
        .cb     = usb_device_cdc_event,
        .type   = kUSB_DeviceClassTypeCdc,
    },
#endif
#if MYNEWT_VAL(USB_DEVICE_CONFIG_MSC)
    {
        .init   = USB_DeviceMscInit,
        .deinit = USB_DeviceMscDeinit,
        .cb     = USB_DeviceMscEvent,
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

//FIXME: this must be moved from here, USB_GLOBAL is platform specific
//USB_GLOBAL
static usb_device_common_class_t s_UsbDeviceCommonClassStruct[MYNEWT_VAL(USB_DEVICE_CONFIG_NUM)];

/*!
 * This function allocates a a device common class handle.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param handle          It is out parameter, is used to return pointer of the device common class handle to the
 * caller.
 */
static usb_status_t
usb_device_class_alloc_handle(uint8_t controllerId, usb_device_common_class_t **handle)
{
    int i;
    usb_status_t err = kStatus_USB_Busy;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);

    /* Check the controller is initialized or not. */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDeviceCommonClassStruct[i].handle &&
            s_UsbDeviceCommonClassStruct[i].controllerId == controllerId) {
            err = kStatus_USB_Error;
            goto out;
        }
    }

    /* Get a free common class handle. */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (!s_UsbDeviceCommonClassStruct[i].handle) {
            s_UsbDeviceCommonClassStruct[i].controllerId = controllerId;
            *handle = &s_UsbDeviceCommonClassStruct[i];
            err = kStatus_USB_Success;
            goto out;
        }
    }

out:
    OS_EXIT_CRITICAL(sr);
    return err;
}

/*!
 * This function frees a device common class handle.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 */
static usb_status_t
usb_device_class_free_handle(uint8_t controllerId)
{
    int i;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);

    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDeviceCommonClassStruct[i].handle &&
            s_UsbDeviceCommonClassStruct[i].controllerId == controllerId) {
            s_UsbDeviceCommonClassStruct[i].handle = NULL;
            s_UsbDeviceCommonClassStruct[i].configList = NULL;
            s_UsbDeviceCommonClassStruct[i].controllerId = 0;
            OS_EXIT_CRITICAL(sr);
            return kStatus_USB_Success;
        }
    }

    OS_EXIT_CRITICAL(sr);
    return kStatus_USB_InvalidParameter;
}

static usb_status_t
usb_device_class_handle_by_id(uint8_t controllerId,
                              usb_device_common_class_t **handle)
{
    int i;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDeviceCommonClassStruct[i].handle &&
            s_UsbDeviceCommonClassStruct[i].controllerId == controllerId) {
            *handle = &s_UsbDeviceCommonClassStruct[i];
            OS_EXIT_CRITICAL(sr);
            return kStatus_USB_Success;
        }
    }

    OS_EXIT_CRITICAL(sr);
    return kStatus_USB_InvalidParameter;
}

static usb_status_t
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
            return kStatus_USB_Success;
        }
    }

    OS_EXIT_CRITICAL(sr);
    return kStatus_USB_InvalidParameter;
}

usb_status_t
usb_device_class_get_handle(uint8_t controllerId, usb_device_handle *handle)
{
    int32_t i;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDeviceCommonClassStruct[i].handle &&
            s_UsbDeviceCommonClassStruct[i].controllerId == controllerId) {
            *handle = s_UsbDeviceCommonClassStruct[i].handle;
            OS_EXIT_CRITICAL(sr);
            return kStatus_USB_Success;
        }
    }
    OS_EXIT_CRITICAL(sr);
    return kStatus_USB_InvalidParameter;
}

/*!
 * This function handles the event passed to the class drivers.
 *
 * @param event           The event codes. Please refer to the enumeration usb_device_class_event_t.
 * @param param           The param type is determined by the event code.
 */
usb_status_t
usb_device_class_event(usb_device_handle handle, usb_device_class_event_t event, void *param)
{
    usb_device_common_class_t *classHandle;
    uint8_t mapIndex;
    uint8_t mapLen;
    uint8_t classIndex;
    usb_status_t rc = kStatus_USB_Error;
    usb_status_t err = kStatus_USB_Error;

    if (!param) {
        return kStatus_USB_InvalidParameter;
    }

    /* Get the common class handle according to the device handle. */
    rc = usb_device_class_handle_by_device(handle, &classHandle);
    if (rc != kStatus_USB_Success) {
        return kStatus_USB_InvalidParameter;
    }

    mapLen = sizeof(s_UsbDeviceClassInterfaceMap) / sizeof(s_UsbDeviceClassInterfaceMap[0]);
    for (classIndex = 0; classIndex < classHandle->configList->count; classIndex++) {
        for (mapIndex = 0; mapIndex < mapLen; mapIndex++) {
            if (s_UsbDeviceClassInterfaceMap[mapIndex].type ==
                classHandle->configList->config[classIndex].classInfomation->type) {
                rc = s_UsbDeviceClassInterfaceMap[mapIndex].cb(
                    (void *)classHandle->configList->config[classIndex].classHandle,
                    event, param);
                if (rc == kStatus_USB_InvalidRequest) {
                    return kStatus_USB_InvalidRequest;
                }
                /* For composite device, it should return kStatus_USB_Success
                 * once a valid request has been handled
                 */
                else if (rc == kStatus_USB_Success) {
                    err = kStatus_USB_Success;
                }
                break;
            }
        }
    }

    return err;
}

/*!
 * This function handles the common class callback.
 *
 * @param event           The event codes. Please refer to the enumeration usb_device_event_t.
 * @param param           The param type is determined by the event code.
 */
usb_status_t
usb_device_class_cb(usb_device_handle handle, uint32_t event, void *param)
{
    usb_device_common_class_t *classHandle;
    usb_status_t err = kStatus_USB_Error;

    err = usb_device_class_handle_by_device(handle, &classHandle);
    if (err != kStatus_USB_Success) {
        return err;
    }

    if (event == kUSB_DeviceEventBusReset) {
        usb_device_control_pipe_init(handle, classHandle);
        usb_device_class_event(handle, kUSB_DeviceClassEventDeviceReset, classHandle);
    }

    return classHandle->configList->deviceCallback(handle, event, param);
}

/*!
 * This function is used to initialize the common class and the supported classes.
 *
 * @param[in] controllerId   The controller id of the USB IP. Please refer to the
 *                           enumeration #usb_controller_index_t.
 * @param[in] configList     The class configurations. The pointer must point to the goblal variable.
 *                           Please refer to the structure #usb_device_class_config_list_struct_t.
 * @param[out] handle        It is out parameter, is used to return pointer of the
 *                           device handle to the caller. The value of parameter
 *                           is a pointer points the device handle, and this design
 *                           is uesd to make simple device align with composite
 *                           device. For composite device, there are many kinds of
 *                           class handle, but there is only one device handle.
 *                           So the handle points to a device instead of a class.
 *                           And the class handle can be got from the
 *                           #usb_device_class_config_struct_t::classHandle after
 *                           the function successfully.
 */
usb_status_t
usb_device_class_init(uint8_t controllerId,
                      usb_device_class_config_list_struct_t *configList,
                      usb_device_handle *handle)
{
    usb_device_common_class_t *classHandle;
    usb_status_t err = kStatus_USB_Error;
    uint8_t mapIndex;
    uint8_t mapLen;
    uint8_t classIndex;

    if (!handle || !configList || !configList->deviceCallback) {
        return kStatus_USB_InvalidParameter;
    }

    err = usb_device_class_alloc_handle(controllerId, &classHandle);
    if (err != kStatus_USB_Success) {
        return err;
    }

    classHandle->configList = configList;
    err = usb_device_init(controllerId, usb_device_class_cb, &classHandle->handle);
    if (err != kStatus_USB_Success) {
        usb_device_deinit(classHandle->handle);
        usb_device_class_free_handle(controllerId);
        return err;
    }

    //FIXME: refactor this is because it's the same loop of .cb calling fn ...
    mapLen = sizeof(s_UsbDeviceClassInterfaceMap) / sizeof(usb_device_class_map_t);
    for (classIndex = 0; classIndex < classHandle->configList->count; classIndex++) {
        for (mapIndex = 0; mapIndex < mapLen; mapIndex++) {
            if (classHandle->configList->config[classIndex].classInfomation->type ==
                s_UsbDeviceClassInterfaceMap[mapIndex].type) {
                (void)s_UsbDeviceClassInterfaceMap[mapIndex].init(
                    controllerId, &classHandle->configList->config[classIndex],
                    &classHandle->configList->config[classIndex].classHandle);
            }
        }
    }

    *handle = classHandle->handle;
    return err;
}

/*!
 * This function is used to de-initialize the common class and the supported classes.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 */
usb_status_t
usb_device_class_deinit(uint8_t controllerId)
{
    usb_device_common_class_t *classHandle;
    usb_status_t err = kStatus_USB_Error;
    uint8_t mapIndex;
    uint8_t mapLen;
    uint8_t classIndex;

    err = usb_device_class_handle_by_id(controllerId, &classHandle);
    if (err != kStatus_USB_Success) {
        return err;
    }

    //FIXME: refactor...
    mapLen = sizeof(s_UsbDeviceClassInterfaceMap) / sizeof(usb_device_class_map_t);
    for (classIndex = 0; classIndex < classHandle->configList->count; classIndex++) {
        for (mapIndex = 0; mapIndex < mapLen; mapIndex++) {
            if (classHandle->configList->config[classIndex].classInfomation->type ==
                s_UsbDeviceClassInterfaceMap[mapIndex].type) {
                (void)s_UsbDeviceClassInterfaceMap[mapIndex].deinit(
                    classHandle->configList->config[classIndex].classHandle);
            }
        }
    }

    err = usb_device_deinit(classHandle->handle);
    if (err == kStatus_USB_Success) {
        (void)usb_device_class_free_handle(controllerId);
    }

    return err;
}

/*!
 * This function is used to get the USB bus speed.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param speed          It is an OUT parameter, return current speed of the controller.
 */
usb_status_t
usb_device_class_get_speed(uint8_t controllerId, uint8_t *speed)
{
    usb_device_common_class_t *classHandle;
    usb_status_t err = kStatus_USB_Error;

    /* Get the common class handle according to the controller id. */
    err = usb_device_class_handle_by_id(controllerId, &classHandle);
    if (err != kStatus_USB_Success) {
        return err;
    }

    return usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusSpeed, speed);
}

#endif /* MYNEWT_VAL(USB_DEVICE_CONFIG_NUM) */
