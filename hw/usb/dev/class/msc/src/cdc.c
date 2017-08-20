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
#include <device/device.h>
#include <device/class.h>

#if MYNEWT_VAL(USB_DEVICE_CONFIG_CDC_ACM)
#include <cdc/cdc.h>

//FIXME: platform specific should be moved...

#include <hal_usb/hal_usb.h>

//FIXME:
//USB_GLOBAL usb_device_cdc_acm_struct_t g_cdcAcmHandle[USB_DEVICE_CONFIG_CDC_ACM_MAX_INSTANCE];
usb_device_cdc_acm_struct_t g_cdcAcmHandle[USB_DEVICE_CONFIG_CDC_ACM_MAX_INSTANCE];

static usb_status_t
usb_dev_cdc_alloc_handle(usb_device_cdc_acm_struct_t * *handle)
{
    int32_t i;

    for (i = 0; i < USB_DEVICE_CONFIG_CDC_ACM_MAX_INSTANCE; i++) {
        if (!g_cdcAcmHandle[i].handle) {
            *handle = &g_cdcAcmHandle[i];
            return kStatus_USB_Success;
        }
    }

    return kStatus_USB_Busy;
}

static usb_status_t
usb_dev_cdc_free_handle(usb_device_cdc_acm_struct_t *handle)
{
    handle->handle = NULL;
    handle->configStruct = NULL;
    handle->configuration = 0;
    handle->alternate = 0;
    return kStatus_USB_Success;
}

/*!
 * This function responds to the interrupt in endpoint event.
 *
 * @param handle The device handle of the CDC ACM device.
 * @param msg The pointer to the message of the endpoint callback.
 * @param callbackParam The pointer to the parameter of the callback.
 */
static usb_status_t
usb_dev_cdc_interrupt_in(usb_device_handle handle,
                         usb_dev_ep_cb_msg_t *msg,
                         void *callbackParam)
{
    usb_device_cdc_acm_struct_t *cdc;

    cdc = (usb_device_cdc_acm_struct_t *)callbackParam;
    if (!cdc) {
        return kStatus_USB_InvalidHandle;
    }

    cdc->interruptIn.isBusy = 0;
    if (cdc->configStruct && cdc->configStruct->classCallback) {
        return cdc->configStruct->classCallback(
            (class_handle_t)cdc, kUSB_DeviceCdcEventSerialStateNotif, msg);
    }

    return kStatus_USB_Error;
}

/*!
 * This function responds to the bulk in endpoint event.
 *
 * @param handle The device handle of the CDC ACM device.
 * @param msg The pointer to the message of the endpoint callback.
 * @param callbackParam The pointer to the parameter of the callback.
 */
static usb_status_t
usb_dev_cdc_bulk_in(usb_device_handle handle,
                    usb_dev_ep_cb_msg_t *msg,
                    void *callbackParam)
{
    usb_device_cdc_acm_struct_t *cdc;

    cdc = (usb_device_cdc_acm_struct_t *)callbackParam;
    if (!cdc) {
        return kStatus_USB_InvalidHandle;
    }

    cdc->bulkIn.isBusy = 0;
    if (cdc->configStruct && cdc->configStruct->classCallback) {
        return cdc->configStruct->classCallback(
            (class_handle_t) cdc, kUSB_DeviceCdcEventSendResponse, msg);
    }

    return kStatus_USB_Error;
}

/*!
 * This function responds to the bulk out endpoint event.
 *
 * @param handle The device handle of the CDC ACM device.
 * @param msg The pointer to the message of the endpoint callback.
 * @param callbackParam The pointer to the parameter of the callback.
 */
static usb_status_t
usb_dev_cdc_bulk_out(usb_device_handle handle,
                     usb_dev_ep_cb_msg_t *msg,
                     void *callbackParam)
{
    usb_device_cdc_acm_struct_t *cdc;

    cdc = (usb_device_cdc_acm_struct_t *)callbackParam;
    if (!cdc) {
        return kStatus_USB_InvalidHandle;
    }

    cdc->bulkOut.isBusy = 0;
    if (cdc->configStruct && cdc->configStruct->classCallback) {
        return cdc->configStruct->classCallback(
            (class_handle_t)cdc, kUSB_DeviceCdcEventRecvResponse, msg);
    }

    return kStatus_USB_Error;
}

/*!
 * This function initializes the endpoints in CDC ACM class.
 *
 * @param cdc The class handle of the CDC ACM class.
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t
usb_dev_cdc_endpoints_init(usb_device_cdc_acm_struct_t *cdc)
{
    usb_device_interface_list_t *interfaceList;
    usb_device_interface_struct_t *interface = NULL;
    usb_status_t err = kStatus_USB_Error;
    int i;
    uint8_t dir;

    if (!cdc) {
        return err;
    }

    /*
     * return error when configuration is invalid (0 or more than the
     * configuration number)
     */
    if (cdc->configuration == 0 ||
        cdc->configuration > cdc->configStruct->classInfomation->configurations) {
        return err;
    }

    interfaceList =
        &cdc->configStruct->classInfomation->interfaceList[
            cdc->configuration - 1];

    for (i = 0; i < interfaceList->count; i++) {
        if (USB_DEVICE_CONFIG_CDC_COMM_CLASS_CODE ==
            interfaceList->interfaces[i].classCode) {
            for (int index = 0; index < interfaceList->interfaces[i].count;
                 index++) {
                if (interfaceList->interfaces[i].interface[index].
                    alternateSetting == cdc->alternate) {
                    interface =
                        &interfaceList->interfaces[i].interface[index];
                    break;
                }
            }
            cdc->interfaceNumber = interfaceList->interfaces[i].interfaceNumber;
            break;
        }
    }
    if (!interface) {
        return err;
    }
    cdc->commInterfaceHandle = interface;
    for (i = 0; i < interface->endpointList.count; i++) {
        usb_device_endpoint_init_struct_t epInitStruct;
        usb_device_endpoint_callback_struct_t epCallback;
        epInitStruct.zlt = 0;
        epInitStruct.endpointAddress =
            interface->endpointList.endpoint[i].endpointAddress;
        epInitStruct.maxPacketSize =
            interface->endpointList.endpoint[i].maxPacketSize;
        epInitStruct.transferType =
            interface->endpointList.endpoint[i].transferType;

        dir = USB_EP_DIR(epInitStruct.endpointAddress);
        if (dir == USB_IN && epInitStruct.transferType == USB_ENDPOINT_INTERRUPT) {
            cdc->interruptIn.ep = USB_EP_NUMBER(epInitStruct.endpointAddress);
            cdc->interruptIn.isBusy = 0;
            epCallback.callbackFn = usb_dev_cdc_interrupt_in;
        }

        epCallback.callbackParam = cdc;

        err = usb_dev_ep_init(cdc->handle, &epInitStruct, &epCallback);
    }

    for (i = 0; i < interfaceList->count; i++) {
        if (USB_DEVICE_CONFIG_CDC_DATA_CLASS_CODE ==
            interfaceList->interfaces[i].classCode) {
            for (int index = 0; index < interfaceList->interfaces[i].count;
                 index++) {
                if (interfaceList->interfaces[i].interface[index].
                    alternateSetting == cdc->alternate) {
                    interface = &interfaceList->interfaces[i].interface[index];
                    break;
                }
            }
            break;
        }
    }

    cdc->dataInterfaceHandle = interface;

    for (i = 0; i < interface->endpointList.count; i++) {
        usb_device_endpoint_init_struct_t epInitStruct;
        usb_device_endpoint_callback_struct_t epCallback;
        epInitStruct.zlt = 0;
        epInitStruct.endpointAddress = interface->endpointList.endpoint[i].endpointAddress;
        epInitStruct.maxPacketSize = interface->endpointList.endpoint[i].maxPacketSize;
        epInitStruct.transferType = interface->endpointList.endpoint[i].transferType;

        dir = USB_EP_DIR(epInitStruct.endpointAddress);
        if (dir == USB_IN && epInitStruct.transferType == USB_ENDPOINT_BULK) {
            cdc->bulkIn.ep = USB_EP_NUMBER(epInitStruct.endpointAddress);
            cdc->bulkIn.isBusy = 0;
            epCallback.callbackFn = usb_dev_cdc_bulk_in;
        } else if (dir == USB_OUT && epInitStruct.transferType == USB_ENDPOINT_BULK) {
            cdc->bulkOut.ep = USB_EP_NUMBER(epInitStruct.endpointAddress);
            cdc->bulkOut.isBusy = 0;
            epCallback.callbackFn = usb_dev_cdc_bulk_out;
        }
        epCallback.callbackParam = cdc;
        err = usb_dev_ep_init(cdc->handle, &epInitStruct, &epCallback);
    }

    return err;
}

/*!
 * This function de-initializes the endpoints in CDC ACM class.
 *
 * @param handle The class handle of the CDC ACM class.
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t
usb_dev_cdc_eps_deinit(usb_device_cdc_acm_struct_t *handle)
{
    usb_status_t err = kStatus_USB_Error;
    int i;

    if (!handle->commInterfaceHandle || !handle->dataInterfaceHandle) {
        return err;
    }

    for (i = 0; i < handle->commInterfaceHandle->endpointList.count; i++) {
        err = usb_dev_ep_deinit(handle->handle,
            handle->commInterfaceHandle->endpointList.endpoint[i].endpointAddress);
    }

    for (i = 0; i < handle->dataInterfaceHandle->endpointList.count; i++) {
        err = usb_dev_ep_deinit(handle->handle,
            handle->dataInterfaceHandle->endpointList.endpoint[i].endpointAddress);
    }

    handle->commInterfaceHandle = handle->dataInterfaceHandle = NULL;

    return err;
}

usb_status_t
usb_dev_cdc_event(void *handle, uint32_t event, void *param)
{
    usb_device_cdc_acm_struct_t *cdcAcmHandle;
    usb_device_cdc_acm_request_param_struct_t reqParam;
    usb_status_t err = kStatus_USB_Error;
    uint16_t interfaceAlternate;
    uint8_t *temp8;
    uint8_t alternate;
    int i;

    if (!param || !handle) {
        return kStatus_USB_InvalidHandle;
    }

    cdcAcmHandle = (usb_device_cdc_acm_struct_t *) handle;

    switch (event) {
    case kUSB_DeviceClassEventDeviceReset:
        /* Bus reset, clear the configuration. */
        cdcAcmHandle->configuration = 0;
        break;
    case kUSB_DeviceClassEventSetConfiguration:
        temp8 = ((uint8_t *)param);
        if (!cdcAcmHandle->configStruct) {
            break;
        }
        if (*temp8 == cdcAcmHandle->configuration) {
            break;
        }

        err = usb_dev_cdc_eps_deinit(cdcAcmHandle);
        cdcAcmHandle->configuration = *temp8;
        cdcAcmHandle->alternate = 0;
        err = usb_dev_cdc_endpoints_init(cdcAcmHandle);
        if (kStatus_USB_Success != err) {
            //usb_echo(
            //    "kUSB_DeviceClassEventSetConfiguration, usb_dev_ep_init fail\n");
        }
        break;
    case kUSB_DeviceClassEventSetInterface:
        if (!cdcAcmHandle->configStruct) {
            break;
        }

        interfaceAlternate = *((uint16_t *)param);
        alternate = (uint8_t)(interfaceAlternate & 0xFF);

        if (cdcAcmHandle->interfaceNumber != (uint8_t)(interfaceAlternate >> 8)) {
            break;
        }
        if (alternate == cdcAcmHandle->alternate) {
            break;
        }
        err = usb_dev_cdc_eps_deinit(cdcAcmHandle);
        cdcAcmHandle->alternate = alternate;
        err = usb_dev_cdc_endpoints_init(cdcAcmHandle);
        if (kStatus_USB_Success != err) {
            //usb_echo(
            //    "kUSB_DeviceClassEventSetInterface, usb_dev_ep_init fail\n");
        }
        break;
    case kUSB_DeviceClassEventSetEndpointHalt:
        if ((!cdcAcmHandle->configStruct) ||
            (!cdcAcmHandle->commInterfaceHandle) ||
            (!cdcAcmHandle->dataInterfaceHandle)) {
            break;
        }
        temp8 = (uint8_t *)param;
        for (i = 0; i < cdcAcmHandle->commInterfaceHandle->endpointList.count; i++) {
            if (*temp8 ==
                    cdcAcmHandle->commInterfaceHandle->endpointList.endpoint[i].
                        endpointAddress) {
                err = usb_dev_ep_stall(cdcAcmHandle->handle, *temp8);
            }
        }
        for (i = 0; i < cdcAcmHandle->dataInterfaceHandle->endpointList.count; i++) {
            if (*temp8 ==
                    cdcAcmHandle->dataInterfaceHandle->endpointList.endpoint[i].
                        endpointAddress) {
                err = usb_dev_ep_stall(cdcAcmHandle->handle, *temp8);
            }
        }
        break;
    case kUSB_DeviceClassEventClearEndpointHalt:
        if ((!cdcAcmHandle->configStruct) ||
            (!cdcAcmHandle->commInterfaceHandle) ||
            (!cdcAcmHandle->dataInterfaceHandle)) {
            break;
        }
        temp8 = ((uint8_t *)param);
        for (i = 0; i < cdcAcmHandle->commInterfaceHandle->endpointList.count; i++) {
            if (*temp8 ==
                    cdcAcmHandle->commInterfaceHandle->endpointList.endpoint[i].
                        endpointAddress) {
                err = usb_dev_ep_unstall(cdcAcmHandle->handle, *temp8);
            }
        }
        for (i = 0; i < cdcAcmHandle->dataInterfaceHandle->endpointList.count; i++) {
            if (*temp8 ==
                    cdcAcmHandle->dataInterfaceHandle->endpointList.endpoint[i].
                        endpointAddress) {
                err = usb_dev_ep_unstall(cdcAcmHandle->handle, *temp8);
            }
        }
        break;
    case kUSB_DeviceClassEventClassRequest:
        if (param) {
            usb_device_control_request_struct_t *controlRequest =
                (usb_device_control_request_struct_t *)param;

            if ((controlRequest->setup->wIndex & 0xff) !=
                cdcAcmHandle->interfaceNumber) {
                break;
            }
            /* Standard CDC request */
            if (USB_REQUEST_TYPE_TYPE_CLASS ==
                (controlRequest->setup->bmRequestType &
                 USB_REQUEST_TYPE_TYPE_MASK)) {
                reqParam.buffer = &(controlRequest->buffer);
                reqParam.length = &(controlRequest->length);
                reqParam.interfaceIndex = controlRequest->setup->wIndex;
                reqParam.setupValue = controlRequest->setup->wValue;
                reqParam.isSetup = controlRequest->isSetup;
                switch (controlRequest->setup->bRequest) {
                case USB_DEVICE_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND:
                    err = cdcAcmHandle->configStruct->classCallback(
                        (class_handle_t)cdcAcmHandle,
                        kUSB_DeviceCdcEventSendEncapsulatedCommand,
                        &reqParam);
                    break;
                case USB_DEVICE_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE:
                    err = cdcAcmHandle->configStruct->classCallback(
                        (class_handle_t)cdcAcmHandle,
                        kUSB_DeviceCdcEventGetEncapsulatedResponse,
                        &reqParam);
                    break;
                case USB_DEVICE_CDC_REQUEST_SET_COMM_FEATURE:
                    err = cdcAcmHandle->configStruct->classCallback(
                        (class_handle_t)cdcAcmHandle,
                        kUSB_DeviceCdcEventSetCommFeature, &reqParam);
                    break;
                case USB_DEVICE_CDC_REQUEST_GET_COMM_FEATURE:
                    err = cdcAcmHandle->configStruct->classCallback(
                        (class_handle_t)cdcAcmHandle,
                        kUSB_DeviceCdcEventGetCommFeature, &reqParam);
                    break;
                case USB_DEVICE_CDC_REQUEST_CLEAR_COMM_FEATURE:
                    err = cdcAcmHandle->configStruct->classCallback(
                        (class_handle_t)cdcAcmHandle,
                        kUSB_DeviceCdcEventClearCommFeature,
                        &reqParam);
                    break;
                case USB_DEVICE_CDC_REQUEST_GET_LINE_CODING:
                    err = cdcAcmHandle->configStruct->classCallback(
                        (class_handle_t)cdcAcmHandle,
                        kUSB_DeviceCdcEventGetLineCoding, &reqParam);
                    break;
                case USB_DEVICE_CDC_REQUEST_SET_LINE_CODING:
                    err = cdcAcmHandle->configStruct->classCallback(
                        (class_handle_t)cdcAcmHandle,
                        kUSB_DeviceCdcEventSetLineCoding, &reqParam);
                    break;
                case USB_DEVICE_CDC_REQUEST_SET_CONTROL_LINE_STATE:
                    err = cdcAcmHandle->configStruct->classCallback(
                        (class_handle_t)cdcAcmHandle,
                        kUSB_DeviceCdcEventSetControlLineState,
                        &reqParam);
                    break;
                case USB_DEVICE_CDC_REQUEST_SEND_BREAK:
                    err = cdcAcmHandle->configStruct->classCallback(
                        (class_handle_t)cdcAcmHandle,
                        kUSB_DeviceCdcEventSendBreak,
                        &reqParam);
                    break;
                default:
                    err = kStatus_USB_InvalidRequest;
                    break;
                }
            }
        }
        break;
    }

    return err;
}

usb_status_t
usb_dev_cdc_init(uint8_t controllerId,
                 usb_dev_class_config_t *config,
                 class_handle_t *handle)
{
    usb_device_cdc_acm_struct_t *cdcAcmHandle;
    usb_status_t err = kStatus_USB_Error;

    err = usb_dev_cdc_alloc_handle(&cdcAcmHandle);

    if (kStatus_USB_Success != err) {
        return err;
    }

    err = usb_device_class_get_handle(controllerId, &cdcAcmHandle->handle);
    if (kStatus_USB_Success != err) {
        return err;
    }

    if (!cdcAcmHandle->handle) {
        return kStatus_USB_InvalidHandle;
    }
    cdcAcmHandle->configStruct = config;
    cdcAcmHandle->configuration = 0;
    cdcAcmHandle->alternate = 0xFF;

#if 0
    if (kStatus_USB_OSA_Success !=
        USB_OsaMutexCreate(&(cdcAcmHandle->bulkIn.mutex))) {
        usb_echo("mutex create error!");
    }
    if (kStatus_USB_OSA_Success !=
        USB_OsaMutexCreate(&(cdcAcmHandle->bulkOut.mutex))) {
        usb_echo("mutex create error!");
    }
    if (kStatus_USB_OSA_Success !=
        USB_OsaMutexCreate(&(cdcAcmHandle->interruptIn.mutex))) {
        usb_echo("mutex create error!");
    }
#endif
    *handle = (class_handle_t)cdcAcmHandle;
    return err;
}

usb_status_t
usb_dev_cdc_deinit(class_handle_t handle)
{
    usb_device_cdc_acm_struct_t *cdc;
    usb_status_t err = kStatus_USB_Error;

    cdc = (usb_device_cdc_acm_struct_t *)handle;

    if (!cdc) {
        return kStatus_USB_InvalidHandle;
    }

    //FIXME: seems like mutexes are not really needed...
#if 0
    //FIXME: log errors below
    if (USB_OsaMutexDestroy(cdc->bulkIn.mutex) != kStatus_USB_OSA_Success) {
        usb_echo("mutex destroy error!");
    }
    if (USB_OsaMutexDestroy(cdc->bulkOut.mutex) != kStatus_USB_OSA_Success) {
        usb_echo("mutex destroy error!");
    }
    if (USB_OsaMutexDestroy(cdc->interruptIn.mutex) != kStatus_USB_OSA_Success) {
        usb_echo("mutex destroy error!");
    }
#endif

    err = usb_dev_cdc_eps_deinit(cdc);
    usb_dev_cdc_free_handle(cdc);
    return err;
}

usb_status_t
usb_dev_cdc_send(class_handle_t handle, uint8_t ep, uint8_t *buffer,
                 uint32_t length)
{
    usb_device_cdc_acm_struct_t *cdc;
    usb_status_t err = kStatus_USB_Error;
    usb_device_cdc_acm_pipe_t *pipe = NULL;
    os_sr_t sr;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    cdc = (usb_device_cdc_acm_struct_t *)handle;

    if (cdc->bulkIn.ep == ep) {
        pipe = &cdc->bulkIn;
    } else if (cdc->interruptIn.ep == ep) {
        pipe = &cdc->interruptIn;
    }

    if (pipe) {
        if (pipe->isBusy) {
            return kStatus_USB_Busy;
        }

        OS_ENTER_CRITICAL(sr);
        err = usb_device_send_req(cdc->handle, ep, buffer, length);
        if (err == kStatus_USB_Success) {
            pipe->isBusy = 1;
        }
        OS_EXIT_CRITICAL(sr);
    }

    return err;
}

usb_status_t
usb_dev_cdc_recv(class_handle_t handle, uint8_t ep, uint8_t *buffer,
                 uint32_t length)
{
    usb_device_cdc_acm_struct_t *cdc;
    usb_status_t err = kStatus_USB_Error;
    os_sr_t sr;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    cdc = (usb_device_cdc_acm_struct_t *)handle;
    if (cdc->bulkOut.isBusy == 1) {
        return kStatus_USB_Busy;
    }

    OS_ENTER_CRITICAL(sr);
    err = usb_device_recv_req(cdc->handle, ep, buffer, length);
    if (kStatus_USB_Success == err) {
        cdc->bulkOut.isBusy = 1;
    }
    OS_EXIT_CRITICAL(sr);

    return err;
}

#endif /* USB_DEVICE_CONFIG_CDC_ACM */
