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
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#if MYNEWT_VAL(USB_DEVICE_CONFIG_CDC_ACM)
#include <cdc/cdc.h>

#include <hal_usb/hal_usb.h>

//FIXME: make dynamic
usb_dev_cdc_t g_cdc[USB_DEVICE_CONFIG_CDC_ACM_MAX_INSTANCE];

/* full speed device */
#define DEVICE_RELEASE                                 0x0101
#define USB_DEVICE_CLASS                               0x02
#define USB_DEVICE_SUBCLASS                            0x00
#define USB_DEVICE_PROTOCOL                            0x00

uint8_t g_cdc_device_descriptor[DESC_LEN_DEVICE] = {
    DESC_LEN_DEVICE,
    DESC_TYPE_DEVICE,
    USB_SHORT_GET_LOW(USB_SPEC_RELEASE),
    USB_SHORT_GET_HIGH(USB_SPEC_RELEASE),
    USB_DEVICE_CLASS,
    USB_DEVICE_SUBCLASS,
    USB_DEVICE_PROTOCOL,
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
    USB_SHORT_GET_LOW(DEVICE_RELEASE),
    USB_SHORT_GET_HIGH(DEVICE_RELEASE),
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
usb_dev_cdc_alloc_handle(usb_dev_cdc_t **handle)
{
    int32_t i;

    for (i = 0; i < USB_DEVICE_CONFIG_CDC_ACM_MAX_INSTANCE; i++) {
        if (!g_cdc[i].handle) {
            *handle = &g_cdc[i];
            return 0;
        }
    }

    return USB_BUSY;
}

static int
usb_dev_cdc_free_handle(usb_dev_cdc_t *handle)
{
    handle->handle = NULL;
    handle->config = NULL;
    handle->config_num = 0;
    handle->alternate = 0;
    return 0;
}

static int
usb_dev_cdc_interrupt_in(usb_device_handle handle, usb_dev_ep_cb_msg_t *msg,
        void *param)
{
    usb_dev_cdc_t *cdc;

    cdc = (usb_dev_cdc_t *)param;
    if (!cdc) {
        return USB_INVALID_HANDLE;
    }

    cdc->intr_in.is_busy = false;
    if (cdc->config && cdc->config->cb) {
        return cdc->config->cb((class_handle_t)cdc, CDC_EVT_SERIAL_STATE_NOTIF, msg);
    }

    return USB_ERR;
}

static int
usb_dev_cdc_bulk_in(usb_device_handle handle, usb_dev_ep_cb_msg_t *msg,
        void *param)
{
    usb_dev_cdc_t *cdc;

    cdc = (usb_dev_cdc_t *)param;
    if (!cdc) {
        return USB_INVALID_HANDLE;
    }

    cdc->bulk_in.is_busy = false;
    if (cdc->config && cdc->config->cb) {
        return cdc->config->cb((class_handle_t)cdc, CDC_EVT_SEND_RESPONSE, msg);
    }

    return USB_ERR;
}

static int
usb_dev_cdc_bulk_out(usb_device_handle handle, usb_dev_ep_cb_msg_t *msg,
        void *param)
{
    usb_dev_cdc_t *cdc;

    cdc = (usb_dev_cdc_t *)param;
    if (!cdc) {
        return USB_INVALID_HANDLE;
    }

    cdc->bulk_out.is_busy = false;
    if (cdc->config && cdc->config->cb) {
        return cdc->config->cb((class_handle_t)cdc, CDC_EVT_RECV_RESPONSE, msg);
    }

    return USB_ERR;
}

static int
usb_dev_cdc_endpoints_init(usb_dev_cdc_t *cdc)
{
    usb_device_interface_list_t *interfaceList;
    usb_dev_itf_t *interface = NULL;
    int i, j;
    uint8_t dir;
    int err = USB_ERR;

    if (!cdc) {
        return err;
    }

    if (cdc->config_num == 0 || cdc->config_num > cdc->config->info->configurations) {
        return err;
    }

    interfaceList = &cdc->config->info->interfaceList[cdc->config_num - 1];

    for (i = 0; i < interfaceList->count; i++) {
        if (interfaceList->itfs[i].classCode == USB_DEVICE_CONFIG_CDC_COMM_CLASS_CODE) {
            for (j = 0; j < interfaceList->itfs[i].count; j++) {
                if (interfaceList->itfs[i].itf[j].alternateSetting == cdc->alternate) {
                    interface = &interfaceList->itfs[i].itf[j];
                    break;
                }
            }
            cdc->itf_num = interfaceList->itfs[i].itf_num;
            break;
        }
    }
    if (!interface) {
        return err;
    }
    cdc->comm_itf = interface;
    for (i = 0; i < interface->eps.count; i++) {
        usb_dev_ep_init_t epInitStruct;
        usb_dev_ep_cb_t ep_cb;
        epInitStruct.zlt = 0;
        epInitStruct.endpointAddress = interface->eps.ep[i].ep_addr;
        epInitStruct.maxPacketSize = interface->eps.ep[i].maxPacketSize;
        epInitStruct.transferType = interface->eps.ep[i].transferType;

        dir = USB_EP_DIR(epInitStruct.endpointAddress);
        if (dir == USB_IN && epInitStruct.transferType == USB_ENDPOINT_INTERRUPT) {
            cdc->intr_in.ep = USB_EP_NUMBER(epInitStruct.endpointAddress);
            cdc->intr_in.is_busy = false;
            ep_cb.fn = usb_dev_cdc_interrupt_in;
        }

        ep_cb.param = cdc;

        err = usb_dev_ep_init(cdc->handle, &epInitStruct, &ep_cb);
    }

    for (i = 0; i < interfaceList->count; i++) {
        if (USB_DEVICE_CONFIG_CDC_DATA_CLASS_CODE ==
            interfaceList->itfs[i].classCode) {
            for (j = 0; j < interfaceList->itfs[i].count; j++) {
                if (interfaceList->itfs[i].itf[j].alternateSetting == cdc->alternate) {
                    interface = &interfaceList->itfs[i].itf[j];
                    break;
                }
            }
            break;
        }
    }

    cdc->data_itf = interface;

    for (i = 0; i < interface->eps.count; i++) {
        usb_dev_ep_init_t epInitStruct;
        usb_dev_ep_cb_t ep_cb;
        epInitStruct.zlt = 0;
        epInitStruct.endpointAddress = interface->eps.ep[i].ep_addr;
        epInitStruct.maxPacketSize = interface->eps.ep[i].maxPacketSize;
        epInitStruct.transferType = interface->eps.ep[i].transferType;

        dir = USB_EP_DIR(epInitStruct.endpointAddress);
        if (dir == USB_IN && epInitStruct.transferType == USB_ENDPOINT_BULK) {
            cdc->bulk_in.ep = USB_EP_NUMBER(epInitStruct.endpointAddress);
            cdc->bulk_in.is_busy = false;
            ep_cb.fn = usb_dev_cdc_bulk_in;
        } else if (dir == USB_OUT && epInitStruct.transferType == USB_ENDPOINT_BULK) {
            cdc->bulk_out.ep = USB_EP_NUMBER(epInitStruct.endpointAddress);
            cdc->bulk_out.is_busy = false;
            ep_cb.fn = usb_dev_cdc_bulk_out;
        }
        ep_cb.param = cdc;
        err = usb_dev_ep_init(cdc->handle, &epInitStruct, &ep_cb);
    }

    return err;
}

static int
usb_dev_cdc_endpoints_deinit(usb_dev_cdc_t *handle)
{
    int err = USB_ERR;
    int i;

    if (!handle->comm_itf || !handle->data_itf) {
        return err;
    }

    for (i = 0; i < handle->comm_itf->eps.count; i++) {
        err = usb_dev_ep_deinit(handle->handle, handle->comm_itf->eps.ep[i].ep_addr);
    }

    for (i = 0; i < handle->data_itf->eps.count; i++) {
        err = usb_dev_ep_deinit(handle->handle, handle->data_itf->eps.ep[i].ep_addr);
    }

    handle->comm_itf = handle->data_itf = NULL;

    return err;
}

int
usb_dev_cdc_event(void *handle, uint32_t event, void *param)
{
    usb_dev_cdc_t *cdc;
    usb_dev_cdc_req_param_t reqParam;
    uint16_t interfaceAlternate;
    uint8_t *temp8;
    uint8_t alternate;
    int i;
    int err = USB_ERR;

    if (!param || !handle) {
        return USB_INVALID_HANDLE;
    }

    cdc = (usb_dev_cdc_t *)handle;

    switch (event) {
    case kUSB_DeviceClassEventDeviceReset:
        cdc->config_num = 0;
        break;
    case kUSB_DeviceClassEventSetConfiguration:
        temp8 = (uint8_t *)param;
        if (!cdc->config) {
            break;
        }
        if (*temp8 == cdc->config_num) {
            break;
        }

        err = usb_dev_cdc_endpoints_deinit(cdc);
        cdc->config_num = *temp8;
        cdc->alternate = 0;
        err = usb_dev_cdc_endpoints_init(cdc);
        if (err) {
            //usb_echo(
            //    "kUSB_DeviceClassEventSetConfiguration, usb_dev_ep_init fail\n");
        }
        break;
    case kUSB_DeviceClassEventSetInterface:
        if (!cdc->config) {
            break;
        }

        interfaceAlternate = *((uint16_t *)param);
        alternate = interfaceAlternate & 0xFF;

        if (cdc->itf_num != (uint8_t)(interfaceAlternate >> 8)) {
            break;
        }
        if (alternate == cdc->alternate) {
            break;
        }
        err = usb_dev_cdc_endpoints_deinit(cdc);
        cdc->alternate = alternate;
        err = usb_dev_cdc_endpoints_init(cdc);
        if (err) {
            //TODO
            //usb_echo(
            //    "kUSB_DeviceClassEventSetInterface, usb_dev_ep_init fail\n");
        }
        break;
    case kUSB_DeviceClassEventSetEndpointHalt:
        if (!cdc->config || !cdc->comm_itf || !cdc->data_itf) {
            break;
        }
        temp8 = (uint8_t *)param;
        for (i = 0; i < cdc->comm_itf->eps.count; i++) {
            if (*temp8 == cdc->comm_itf->eps.ep[i].ep_addr) {
                err = usb_dev_ep_stall(cdc->handle, *temp8);
            }
        }
        for (i = 0; i < cdc->data_itf->eps.count; i++) {
            if (*temp8 == cdc->data_itf->eps.ep[i].ep_addr) {
                err = usb_dev_ep_stall(cdc->handle, *temp8);
            }
        }
        break;
    case kUSB_DeviceClassEventClearEndpointHalt:
        if (!cdc->config || !cdc->comm_itf || !cdc->data_itf) {
            break;
        }
        temp8 = (uint8_t *)param;
        for (i = 0; i < cdc->comm_itf->eps.count; i++) {
            if (*temp8 == cdc->comm_itf->eps.ep[i].ep_addr) {
                err = usb_dev_ep_unstall(cdc->handle, *temp8);
            }
        }
        for (i = 0; i < cdc->data_itf->eps.count; i++) {
            if (*temp8 == cdc->data_itf->eps.ep[i].ep_addr) {
                err = usb_dev_ep_unstall(cdc->handle, *temp8);
            }
        }
        break;
    case kUSB_DeviceClassEventClassRequest:
        if (param) {
            usb_dev_ctrl_req_t *req = (usb_dev_ctrl_req_t *)param;

            if ((req->setup->wIndex & 0xff) != cdc->itf_num) {
                break;
            }
            /* Standard CDC request */
            if (USB_REQ_TYPE_TYPE_CLASS ==
                (req->setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
                reqParam.buffer = &req->buf;
                reqParam.length = &req->len;
                reqParam.interfaceIndex = req->setup->wIndex;
                reqParam.setupValue = req->setup->wValue;
                reqParam.isSetup = req->is_setup;
                switch (req->setup->bRequest) {
                case USB_DEVICE_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND: /* fallthrough */
                case USB_DEVICE_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE: /* fallthrough */
                case USB_DEVICE_CDC_REQUEST_SET_COMM_FEATURE:          /* fallthrough */
                case USB_DEVICE_CDC_REQUEST_GET_COMM_FEATURE:          /* fallthrough */
                case USB_DEVICE_CDC_REQUEST_CLEAR_COMM_FEATURE:        /* fallthrough */
                case USB_DEVICE_CDC_REQUEST_GET_LINE_CODING:           /* fallthrough */
                case USB_DEVICE_CDC_REQUEST_SET_LINE_CODING:           /* fallthrough */
                case USB_DEVICE_CDC_REQUEST_SET_CONTROL_LINE_STATE:    /* fallthrough */
                case USB_DEVICE_CDC_REQUEST_SEND_BREAK:
                    err = cdc->config->cb((class_handle_t)cdc, req->setup->bRequest, &reqParam);
                    break;
                default:
                    err = USB_INVALID_REQ;
                    break;
                }
            }
        }
        break;
    }

    return err;
}

int
usb_dev_cdc_init(uint8_t ctrl_id, usb_dev_class_config_t *config,
                 class_handle_t *handle)
{
    usb_dev_cdc_t *cdc;
    int err = USB_ERR;

    err = usb_dev_cdc_alloc_handle(&cdc);

    if (err) {
        return err;
    }

    err = usb_device_class_get_handle(ctrl_id, &cdc->handle);
    if (err) {
        return err;
    }

    if (!cdc->handle) {
        return USB_INVALID_HANDLE;
    }
    cdc->config = config;
    cdc->config_num = 0;
    cdc->alternate = 0xFF;

#if 0
    if (USB_OsaMutexCreate(&cdc->bulk_in.mutex)) {
        usb_echo("mutex create error!");
    }
    if (USB_OsaMutexCreate(&cdc->bulk_out.mutex)) {
        usb_echo("mutex create error!");
    }
    if (USB_OsaMutexCreate(&cdc->intr_in.mutex)) {
        usb_echo("mutex create error!");
    }
#endif
    *handle = (class_handle_t)cdc;
    return err;
}

int
usb_dev_cdc_deinit(class_handle_t handle)
{
    usb_dev_cdc_t *cdc;
    int err = USB_ERR;

    cdc = (usb_dev_cdc_t *)handle;

    if (!cdc) {
        return USB_INVALID_HANDLE;
    }

    //FIXME: seems like mutexes are not really needed...
#if 0
    //FIXME: log errors below
    if (USB_OsaMutexDestroy(cdc->bulk_in.mutex)) {
        usb_echo("mutex destroy error!");
    }
    if (USB_OsaMutexDestroy(cdc->bulk_out.mutex)) {
        usb_echo("mutex destroy error!");
    }
    if (USB_OsaMutexDestroy(cdc->intr_in.mutex)) {
        usb_echo("mutex destroy error!");
    }
#endif

    err = usb_dev_cdc_endpoints_deinit(cdc);
    usb_dev_cdc_free_handle(cdc);
    return err;
}

int
usb_dev_cdc_send(class_handle_t handle, uint8_t ep, uint8_t *buf, uint32_t len)
{
    usb_dev_cdc_t *cdc;
    int err = USB_ERR;
    usb_dev_cdc_pipe_t *pipe = NULL;
    os_sr_t sr;

    if (!handle) {
        return USB_INVALID_HANDLE;
    }

    cdc = (usb_dev_cdc_t *)handle;

    if (cdc->bulk_in.ep == ep) {
        pipe = &cdc->bulk_in;
    } else if (cdc->intr_in.ep == ep) {
        pipe = &cdc->intr_in;
    }

    if (pipe) {
        if (pipe->is_busy) {
            return USB_BUSY;
        }

        OS_ENTER_CRITICAL(sr);
        err = usb_device_send_req(cdc->handle, ep, buf, len);
        if (!err) {
            pipe->is_busy = true;
        }
        OS_EXIT_CRITICAL(sr);
    }

    return err;
}

int
usb_dev_cdc_recv(class_handle_t handle, uint8_t ep, uint8_t *buf, uint32_t len)
{
    usb_dev_cdc_t *cdc;
    int err = USB_ERR;
    os_sr_t sr;

    if (!handle) {
        return USB_INVALID_HANDLE;
    }

    cdc = (usb_dev_cdc_t *)handle;
    if (cdc->bulk_out.is_busy) {
        return USB_BUSY;
    }

    OS_ENTER_CRITICAL(sr);
    err = usb_device_recv_req(cdc->handle, ep, buf, len);
    if (!err) {
        cdc->bulk_out.is_busy = true;
    }
    OS_EXIT_CRITICAL(sr);

    return err;
}

#endif /* USB_DEVICE_CONFIG_CDC_ACM */
