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

#include <stdio.h>
#include <stdlib.h>

#include <usb/usb.h>
#include <dev/dev.h>
#include <dev/class.h>

#if MYNEWT_VAL(USB_DEVICE_CONFIG_HID)
#include <hid/hid.h>

#include <hal_usb/hal_usb.h>

static int usb_dev_hid_alloc_handle(usb_dev_hid_t **handle);
static int USB_DeviceHidFreeHandle(usb_dev_hid_t *handle);
static int USB_DeviceHidInterruptIn(usb_device_handle handle,
                                             usb_dev_ep_cb_msg_t *message,
                                             void *callbackParam);
static int USB_DeviceHidInterruptOut(usb_device_handle handle,
                                              usb_dev_ep_cb_msg_t *message,
                                              void *callbackParam);
static int USB_DeviceHidEndpointsInit(usb_dev_hid_t *hid);
static int USB_DeviceHidEndpointsDeinit(usb_dev_hid_t *hid);

static usb_dev_hid_t s_UsbDeviceHidHandle[MYNEWT_VAL(USB_DEVICE_CONFIG_HID)];

/* full speed device */
#define DEVICE_RELEASE                                 0x0101

uint8_t g_hid_device_descriptor[DESC_LEN_DEVICE] = {
    DESC_LEN_DEVICE,
    DESC_TYPE_DEVICE,
    USB_SHORT_GET_LOW(USB_SPEC_RELEASE),
    USB_SHORT_GET_HIGH(USB_SPEC_RELEASE),
    /* HID class/subclass/protocol (TODO: add spec section) */
    0, 0, 0,
    /* max packet size */
    64,
#if MYNEWT_VAL(USB_VENDOR_ID)
    (uint8_t)((uint16_t)MYNEWT_VAL(USB_VENDOR_ID) >> 8),
    (uint8_t)MYNEWT_VAL(USB_VENDOR_ID),
#else
    #error "Missing USB_VENDOR_ID syscfg"
#endif
#if MYNEWT_VAL(USB_PRODUCT_ID)
    (uint8_t)((uint16_t)MYNEWT_VAL(USB_PRODUCT_ID) >> 8),
    (uint8_t)MYNEWT_VAL(USB_PRODUCT_ID),
#else
    #error "Missing USB_PRODUCT_ID syscfg"
#endif
#if MYNEWT_VAL(USB_DEVICE_RELEASE)
    USB_SHORT_GET_LOW(MYNEWT_VAL(DEVICE_RELEASE)),
    USB_SHORT_GET_HIGH(MYNEWT_VAL(DEVICE_RELEASE)),
#else
    USB_SHORT_GET_LOW(DEVICE_RELEASE),
    USB_SHORT_GET_HIGH(DEVICE_RELEASE),
#endif
    0x01,
    0x02,
    0x00,
#if MYNEWT_VAL(USB_DEVICE_CONFIGURATION_COUNT)
    MYNEWT_VAL(USB_DEVICE_CONFIGURATION_COUNT),
#else
    1,
#endif
};

static int
usb_dev_hid_alloc_handle(usb_dev_hid_t **handle)
{
    int i;
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_HID); i++) {
        if (!s_UsbDeviceHidHandle[i].handle) {
            *handle = &s_UsbDeviceHidHandle[i];
            return 0;
        }
    }

    return USB_BUSY;
}

static int
USB_DeviceHidFreeHandle(usb_dev_hid_t *handle)
{
    handle->handle = NULL;
    handle->config = (usb_dev_class_config_t *)NULL;
    handle->configuration = 0;
    handle->alternate = 0;
    return 0;
}

static int
USB_DeviceHidInterruptIn(usb_device_handle handle, usb_dev_ep_cb_msg_t *msg,
        void *param)
{
    usb_dev_hid_t *hid;
    int err = USB_ERR;

    hid = (usb_dev_hid_t *)param;

    if (!hid) {
        return USB_INVALID_HANDLE;
    }
    hid->interruptInPipeBusy = 0;
    if (hid->config && hid->config->cb) {
        err = hid->config->cb((class_handle_t)hid,
                kUSB_DeviceHidEventSendResponse, msg);
    }

    return err;
}

static int
USB_DeviceHidInterruptOut(usb_device_handle handle, usb_dev_ep_cb_msg_t *msg,
        void *param)
{
    usb_dev_hid_t *hid;
    int err = USB_ERR;

    hid = (usb_dev_hid_t *)param;

    if (!hid) {
        return USB_INVALID_HANDLE;
    }
    hid->interruptOutPipeBusy = 0;
    if (hid->config && hid->config->cb) {
        err = hid->config->cb((class_handle_t)hid,
                kUSB_DeviceHidEventRecvResponse, msg);
    }

    return err;
}

static int
USB_DeviceHidEndpointsInit(usb_dev_hid_t *hid)
{
    usb_device_interface_list_t *interfaceList;
    usb_dev_itf_t *interface = NULL;
    int i;
    int err = USB_ERR;

    if (!hid->configuration) {
        return err;
    }

    if (hid->configuration > hid->config->info->configurations) {
        return err;
    }

    if (!hid->config->info->interfaceList) {
        return err;
    }
    interfaceList = &hid->config->info->interfaceList[hid->configuration - 1];

    for (i = 0; i < interfaceList->count; i++) {
        if (USB_DEVICE_CONFIG_HID_CLASS_CODE == interfaceList->itfs[i].classCode) {
            for (int index = 0; index < interfaceList->itfs[i].count; index++) {
                if (interfaceList->itfs[i].itf[index].alternateSetting == hid->alternate) {
                    interface = &interfaceList->itfs[i].itf[index];
                    break;
                }
            }
            hid->interfaceNumber = interfaceList->itfs[i].itf_num;
            break;
        }
    }
    if (!interface) {
        return err;
    }

    hid->interfaceHandle = interface;

    for (i = 0; i < interface->eps.count; i++) {
        usb_dev_ep_init_t epInitStruct;
        usb_dev_ep_cb_t ep_cb;
        epInitStruct.zlt = 0;
        epInitStruct.endpointAddress = interface->eps.ep[i].ep_addr;
        epInitStruct.maxPacketSize = interface->eps.ep[i].maxPacketSize;
        epInitStruct.transferType = interface->eps.ep[i].transferType;

        if (USB_IN == ((epInitStruct.endpointAddress & USB_DESC_EP_ADDR_DIR_MASK) >>
                       USB_DESC_EP_ADDR_DIR_SHIFT)) {
            ep_cb.fn = USB_DeviceHidInterruptIn;
        } else {
            ep_cb.fn = USB_DeviceHidInterruptOut;
        }
        ep_cb.param = hid;

        err = usb_dev_ep_init(hid->handle, &epInitStruct, &ep_cb);
    }
    return err;
}

static int
USB_DeviceHidEndpointsDeinit(usb_dev_hid_t *hid)
{
    int err = USB_ERR;
    int i;

    if (!hid->interfaceHandle) {
        return err;
    }

    for (i = 0; i < hid->interfaceHandle->eps.count; i++)
    {
        err = usb_dev_ep_deinit(hid->handle,
                                hid->interfaceHandle->eps.ep[i].ep_addr);
    }
    hid->interfaceHandle = NULL;
    return err;
}

int
usb_dev_hid_event(void *handle, uint32_t event, void *param)
{
    usb_dev_hid_t *hid;
    usb_device_hid_report_struct_t report;
    uint16_t interfaceAlternate;
    uint8_t *temp8;
    uint8_t alternate;
    int i;
    int err = USB_ERR;

    if (!param || !handle) {
        return USB_INVALID_HANDLE;
    }

    hid = (usb_dev_hid_t *)handle;

    switch (event)
    {
    case kUSB_DeviceClassEventDeviceReset:
        hid->configuration = 0;
        hid->interruptInPipeBusy = 0;
        hid->interruptOutPipeBusy = 0;
        hid->interfaceHandle = NULL;
        break;
    case kUSB_DeviceClassEventSetConfiguration:
        temp8 = (uint8_t *)param;
        if (!hid->config) {
            break;
        }
        if (*temp8 == hid->configuration) {
            break;
        }

        if (hid->configuration) {
            err = USB_DeviceHidEndpointsDeinit(hid);
        }

        hid->configuration = *temp8;
        hid->alternate = 0;

        err = USB_DeviceHidEndpointsInit(hid);
        break;
    case kUSB_DeviceClassEventSetInterface:
        if (!hid->config) {
            break;
        }

        interfaceAlternate = *((uint16_t *)param);
        alternate = (uint8_t)(interfaceAlternate & 0xFF);

        if (hid->interfaceNumber != ((uint8_t)(interfaceAlternate >> 8))) {
            break;
        }

        if (alternate == hid->alternate) {
            break;
        }

        err = USB_DeviceHidEndpointsDeinit(hid);
        hid->alternate = alternate;

        err = USB_DeviceHidEndpointsInit(hid);
        break;
    case kUSB_DeviceClassEventSetEndpointHalt:
        if (!hid->config || !hid->interfaceHandle) {
            break;
        }
        temp8 = (uint8_t *)param;
        for (i = 0; i < hid->interfaceHandle->eps.count; i++) {
            if (*temp8 == hid->interfaceHandle->eps.ep[i].ep_addr) {
                err = usb_dev_ep_stall(hid->handle, *temp8);
            }
        }
        break;
    case kUSB_DeviceClassEventClearEndpointHalt:
        if (!hid->config || !hid->interfaceHandle) {
            break;
        }
        temp8 = (uint8_t *)param;
        for (i = 0; i < hid->interfaceHandle->eps.count; i++) {
            if (*temp8 == hid->interfaceHandle->eps.ep[i].ep_addr) {
                err = usb_dev_ep_unstall(hid->handle, *temp8);
            }
        }
        break;
    case kUSB_DeviceClassEventClassRequest:
        if (param) {
            usb_dev_ctrl_req_t *req = (usb_dev_ctrl_req_t *)param;

            if ((req->setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) !=
                USB_REQ_TYPE_RECIPIENT_INTERFACE) {
                break;
            }

            if ((req->setup->wIndex & 0xFF) != hid->interfaceNumber) {
                break;
            }

            switch (req->setup->bRequest) {
            case USB_DEVICE_HID_REQUEST_GET_REPORT:
                report.reportType = (req->setup->wValue & 0xFF00) >> 8;
                report.reportId = req->setup->wValue & 0xFF;
                err = hid->config->cb((class_handle_t)hid,
                        kUSB_DeviceHidEventGetReport, &report);
                req->buf = report.reportBuffer;
                req->len = report.reportLength;
                break;
            case USB_DEVICE_HID_REQUEST_GET_IDLE:
                err = hid->config->cb((class_handle_t)hid,
                        kUSB_DeviceHidEventGetIdle, &hid->idleRate);
                req->buf = &hid->idleRate;
                break;
            case USB_DEVICE_HID_REQUEST_GET_PROTOCOL:
                err = hid->config->cb((class_handle_t)hid,
                        kUSB_DeviceHidEventGetIdle, &hid->protocol);
                req->buf = &hid->protocol;
                break;
            case USB_DEVICE_HID_REQUEST_SET_REPORT:
                report.reportType = (req->setup->wValue & 0xFF00) >> 8;
                report.reportId = req->setup->wValue & 0xFF;
                if (req->is_setup) {
                    report.reportLength = req->len;
                    err = hid->config->cb((class_handle_t)hid,
                            kUSB_DeviceHidEventRequestReportBuffer, &report);
                    req->buf = report.reportBuffer;
                    req->len = report.reportLength;
                } else {
                    report.reportBuffer = req->buf;
                    report.reportLength = req->len;
                    err = hid->config->cb((class_handle_t)hid,
                            kUSB_DeviceHidEventSetReport, &report);
                }
                break;
            case USB_DEVICE_HID_REQUEST_SET_IDLE:
                hid->idleRate = (req->setup->wValue & 0xFF00) >> 8;
                err = hid->config->cb((class_handle_t)hid,
                            kUSB_DeviceHidEventSetIdle, &req->setup->wValue);
                break;
            case USB_DEVICE_HID_REQUEST_SET_PROTOCOL:
                hid->protocol = req->setup->wValue & 0xFF;
                err = hid->config->cb((class_handle_t)hid,
                            kUSB_DeviceHidEventSetProtocol, &hid->protocol);
                break;
            default:
                err = USB_INVALID_REQ;
                break;
            }
        }
        break;
    default:
        break;
    }
    return err;
}

int
usb_dev_hid_init(uint8_t controllerId,
                 usb_dev_class_config_t *config,
                 class_handle_t *handle)
{
    usb_dev_hid_t *hid;
    int err = USB_ERR;

    err = usb_dev_hid_alloc_handle(&hid);
    if (err) {
        return err;
    }

    err = usb_device_class_get_handle(controllerId, &hid->handle);
    if (err) {
        return err;
    }

    if (!hid->handle) {
        return USB_INVALID_HANDLE;
    }

    hid->config = config;
    hid->configuration = 0;
    hid->alternate = 0xff;

    *handle = (class_handle_t)hid;
    return err;
}

int
usb_dev_hid_deinit(class_handle_t handle)
{
    usb_dev_hid_t *hid;
    int err = USB_ERR;

    hid = (usb_dev_hid_t *)handle;
    if (!hid) {
        return USB_INVALID_HANDLE;
    }

    err = USB_DeviceHidEndpointsDeinit(hid);
    USB_DeviceHidFreeHandle(hid);
    return err;
}

int
usb_dev_hid_send(class_handle_t handle, uint8_t ep, uint8_t *buf, uint32_t len)
{
    usb_dev_hid_t *hid;
    int err = USB_ERR;

    if (!handle) {
        return USB_INVALID_HANDLE;
    }
    hid = (usb_dev_hid_t *)handle;

    if (hid->interruptInPipeBusy) {
        return USB_BUSY;
    }

    err = usb_device_send_req(hid->handle, ep, buf, len);
    if (!err) {
        hid->interruptInPipeBusy = 1;
    }
    return err;
}

int
usb_dev_hid_recv(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length)
{
    usb_dev_hid_t *hid;
    int err = USB_ERR;

    if (!handle) {
        return USB_INVALID_HANDLE;
    }
    hid = (usb_dev_hid_t *)handle;

    if (hid->interruptOutPipeBusy) {
        return USB_BUSY;
    }
    err = usb_device_recv_req(hid->handle, ep, buffer, length);
    if (!err) {
        hid->interruptOutPipeBusy = 1;
    }
    return err;
}

#endif
