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

static usb_status_t USB_DeviceHidAllocateHandle(usb_device_hid_struct_t **handle);
static usb_status_t USB_DeviceHidFreeHandle(usb_device_hid_struct_t *handle);
static usb_status_t USB_DeviceHidInterruptIn(usb_device_handle handle,
                                             usb_dev_ep_cb_msg_t *message,
                                             void *callbackParam);
static usb_status_t USB_DeviceHidInterruptOut(usb_device_handle handle,
                                              usb_dev_ep_cb_msg_t *message,
                                              void *callbackParam);
static usb_status_t USB_DeviceHidEndpointsInit(usb_device_hid_struct_t *hidHandle);
static usb_status_t USB_DeviceHidEndpointsDeinit(usb_device_hid_struct_t *hidHandle);

static usb_device_hid_struct_t s_UsbDeviceHidHandle[MYNEWT_VAL(USB_DEVICE_CONFIG_HID)];

static usb_status_t
USB_DeviceHidAllocateHandle(usb_device_hid_struct_t **handle)
{
    int i;
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_HID); i++) {
        if (!s_UsbDeviceHidHandle[i].handle) {
            *handle = &s_UsbDeviceHidHandle[i];
            return kStatus_USB_Success;
        }
    }

    return kStatus_USB_Busy;
}

static usb_status_t
USB_DeviceHidFreeHandle(usb_device_hid_struct_t *handle)
{
    handle->handle = NULL;
    handle->configStruct = (usb_dev_class_config_t *)NULL;
    handle->configuration = 0;
    handle->alternate = 0;
    return kStatus_USB_Success;
}

static usb_status_t
USB_DeviceHidInterruptIn(usb_device_handle handle,
                         usb_dev_ep_cb_msg_t *message,
                         void *callbackParam)
{
    usb_device_hid_struct_t *hidHandle;
    usb_status_t error = kStatus_USB_Error;

    hidHandle = (usb_device_hid_struct_t *)callbackParam;

    if (!hidHandle) {
        return kStatus_USB_InvalidHandle;
    }
    hidHandle->interruptInPipeBusy = 0;
    if (hidHandle->configStruct && hidHandle->configStruct->classCallback) {
        error =
            hidHandle->configStruct->classCallback((class_handle_t)hidHandle, kUSB_DeviceHidEventSendResponse, message);
    }

    return error;
}

static usb_status_t
USB_DeviceHidInterruptOut(usb_device_handle handle,
                          usb_dev_ep_cb_msg_t *message,
                          void *callbackParam)
{
    usb_device_hid_struct_t *hidHandle;
    usb_status_t error = kStatus_USB_Error;

    hidHandle = (usb_device_hid_struct_t *)callbackParam;

    if (!hidHandle) {
        return kStatus_USB_InvalidHandle;
    }
    hidHandle->interruptOutPipeBusy = 0;
    if (hidHandle->configStruct && hidHandle->configStruct->classCallback) {
        error =
            hidHandle->configStruct->classCallback((class_handle_t)hidHandle, kUSB_DeviceHidEventRecvResponse, message);
    }

    return error;
}

static usb_status_t
USB_DeviceHidEndpointsInit(usb_device_hid_struct_t *hidHandle)
{
    usb_device_interface_list_t *interfaceList;
    usb_device_interface_struct_t *interface = (usb_device_interface_struct_t *)NULL;
    usb_status_t error = kStatus_USB_Error;
    int i;

    if (!hidHandle->configuration) {
        return error;
    }

    if (hidHandle->configuration > hidHandle->configStruct->classInfomation->configurations) {
        return error;
    }

    if (!hidHandle->configStruct->classInfomation->interfaceList) {
        return error;
    }
    interfaceList = &hidHandle->configStruct->classInfomation->interfaceList[hidHandle->configuration - 1];

    for (i = 0; i < interfaceList->count; i++) {
        if (USB_DEVICE_CONFIG_HID_CLASS_CODE == interfaceList->interfaces[i].classCode) {
            for (int index = 0; index < interfaceList->interfaces[i].count; index++) {
                if (interfaceList->interfaces[i].interface[index].alternateSetting == hidHandle->alternate) {
                    interface = &interfaceList->interfaces[i].interface[index];
                    break;
                }
            }
            hidHandle->interfaceNumber = interfaceList->interfaces[i].interfaceNumber;
            break;
        }
    }
    if (!interface) {
        return error;
    }

    hidHandle->interfaceHandle = interface;

    for (i = 0; i < interface->endpointList.count; i++) {
        usb_device_endpoint_init_struct_t epInitStruct;
        usb_device_endpoint_callback_struct_t ep_callback;
        epInitStruct.zlt = 0;
        epInitStruct.endpointAddress = interface->endpointList.endpoint[i].endpointAddress;
        epInitStruct.maxPacketSize = interface->endpointList.endpoint[i].maxPacketSize;
        epInitStruct.transferType = interface->endpointList.endpoint[i].transferType;

        if (USB_IN == ((epInitStruct.endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) >>
                       USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT)) {
            ep_callback.callbackFn = USB_DeviceHidInterruptIn;
        } else {
            ep_callback.callbackFn = USB_DeviceHidInterruptOut;
        }
        ep_callback.callbackParam = hidHandle;

        error = usb_dev_ep_init(hidHandle->handle, &epInitStruct, &ep_callback);
    }
    return error;
}

static usb_status_t
USB_DeviceHidEndpointsDeinit(usb_device_hid_struct_t *hidHandle)
{
    usb_status_t error = kStatus_USB_Error;
    int i;

    if (!hidHandle->interfaceHandle) {
        return error;
    }

    for (i = 0; i < hidHandle->interfaceHandle->endpointList.count; i++)
    {
        error = usb_dev_ep_deinit(hidHandle->handle,
                                  hidHandle->interfaceHandle->endpointList.endpoint[i].endpointAddress);
    }
    hidHandle->interfaceHandle = NULL;
    return error;
}

usb_status_t usb_dev_hid_event(void *handle, uint32_t event, void *param)
{
    usb_device_hid_struct_t *hidHandle;
    usb_device_hid_report_struct_t report;
    usb_status_t error = kStatus_USB_Error;
    uint16_t interfaceAlternate;
    uint8_t *temp8;
    uint8_t alternate;

    if (!param || !handle) {
        return kStatus_USB_InvalidHandle;
    }

    hidHandle = (usb_device_hid_struct_t *)handle;

    switch (event)
    {
    case kUSB_DeviceClassEventDeviceReset:
        hidHandle->configuration = 0;
        hidHandle->interruptInPipeBusy = 0;
        hidHandle->interruptOutPipeBusy = 0;
        hidHandle->interfaceHandle = NULL;
        break;
    case kUSB_DeviceClassEventSetConfiguration:
        temp8 = (uint8_t *)param;
        if (!hidHandle->configStruct) {
            break;
        }
        if (*temp8 == hidHandle->configuration) {
            break;
        }

        if (hidHandle->configuration) {
            error = USB_DeviceHidEndpointsDeinit(hidHandle);
        }

        hidHandle->configuration = *temp8;
        hidHandle->alternate = 0;

        error = USB_DeviceHidEndpointsInit(hidHandle);
        break;
    case kUSB_DeviceClassEventSetInterface:
        if (!hidHandle->configStruct) {
            break;
        }

        interfaceAlternate = *((uint16_t *)param);
        alternate = (uint8_t)(interfaceAlternate & 0xFF);

        if (hidHandle->interfaceNumber != ((uint8_t)(interfaceAlternate >> 8))) {
            break;
        }

        if (alternate == hidHandle->alternate) {
            break;
        }

        error = USB_DeviceHidEndpointsDeinit(hidHandle);
        hidHandle->alternate = alternate;

        error = USB_DeviceHidEndpointsInit(hidHandle);
        break;
    case kUSB_DeviceClassEventSetEndpointHalt:
        if ((!hidHandle->configStruct) || (!hidHandle->interfaceHandle)) {
            break;
        }
        /* Get the endpoint address */
        temp8 = ((uint8_t *)param);
        for (int count = 0U; count < hidHandle->interfaceHandle->endpointList.count; count++)
        {
            if (*temp8 == hidHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress)
            {
                /* Only stall the endpoint belongs to the class */
                error = usb_dev_ep_stall(hidHandle->handle, *temp8);
            }
        }
        break;
    case kUSB_DeviceClassEventClearEndpointHalt:
        if ((!hidHandle->configStruct) || (!hidHandle->interfaceHandle))
        {
            break;
        }
        /* Get the endpoint address */
        temp8 = ((uint8_t *)param);
        for (int count = 0U; count < hidHandle->interfaceHandle->endpointList.count; count++)
        {
            if (*temp8 == hidHandle->interfaceHandle->endpointList.endpoint[count].endpointAddress)
            {
                /* Only un-stall the endpoint belongs to the class */
                error = usb_dev_ep_unstall(hidHandle->handle, *temp8);
            }
        }
        break;
    case kUSB_DeviceClassEventClassRequest:
        if (param)
        {
            /* Handle the hid class specific request. */
            usb_device_control_request_struct_t *controlRequest = (usb_device_control_request_struct_t *)param;

            if ((controlRequest->setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) !=
                USB_REQUEST_TYPE_RECIPIENT_INTERFACE)
            {
                break;
            }

            if ((controlRequest->setup->wIndex & 0xFFU) != hidHandle->interfaceNumber)
            {
                break;
            }

            switch (controlRequest->setup->bRequest)
            {
                case USB_DEVICE_HID_REQUEST_GET_REPORT:
                    /* Get report request */
                    report.reportType = (controlRequest->setup->wValue & 0xFF00U) >> 0x08U;
                    report.reportId = (controlRequest->setup->wValue & 0x00FFU);
                    error = hidHandle->configStruct->classCallback((class_handle_t)hidHandle,
                                                                   kUSB_DeviceHidEventGetReport, &report);
                    controlRequest->buffer = report.reportBuffer;
                    controlRequest->length = report.reportLength;
                    break;
                case USB_DEVICE_HID_REQUEST_GET_IDLE:
                    /* Get idle request */
                    error = hidHandle->configStruct->classCallback(
                        (class_handle_t)hidHandle, kUSB_DeviceHidEventGetIdle, &hidHandle->idleRate);
                    controlRequest->buffer = &hidHandle->idleRate;
                    break;
                case USB_DEVICE_HID_REQUEST_GET_PROTOCOL:
                    /* Get protocol request */
                    error = hidHandle->configStruct->classCallback(
                        (class_handle_t)hidHandle, kUSB_DeviceHidEventGetIdle, &hidHandle->protocol);
                    controlRequest->buffer = &hidHandle->protocol;
                    break;
                case USB_DEVICE_HID_REQUEST_SET_REPORT:
                    /* Set report request */
                    report.reportType = (controlRequest->setup->wValue & 0xFF00U) >> 0x08U;
                    report.reportId = (controlRequest->setup->wValue & 0x00FFU);
                    if (controlRequest->isSetup)
                    {
                        report.reportLength = controlRequest->length;
                        error = hidHandle->configStruct->classCallback(
                            (class_handle_t)hidHandle, kUSB_DeviceHidEventRequestReportBuffer, &report);
                        controlRequest->buffer = report.reportBuffer;
                        controlRequest->length = report.reportLength;
                    }
                    else
                    {
                        report.reportBuffer = controlRequest->buffer;
                        report.reportLength = controlRequest->length;
                        error = hidHandle->configStruct->classCallback((class_handle_t)hidHandle,
                                                                       kUSB_DeviceHidEventSetReport, &report);
                    }
                    break;
                case USB_DEVICE_HID_REQUEST_SET_IDLE:
                    {
                        hidHandle->idleRate = (controlRequest->setup->wValue & 0xFF00U) >> 0x08U;
                        error = hidHandle->configStruct->classCallback(
                            (class_handle_t)hidHandle, kUSB_DeviceHidEventSetIdle, &controlRequest->setup->wValue);
                    }
                    break;
                case USB_DEVICE_HID_REQUEST_SET_PROTOCOL:
                    /* Set protocol request */
                    {
                        hidHandle->protocol = (controlRequest->setup->wValue & 0x00FFU);
                        error = hidHandle->configStruct->classCallback(
                            (class_handle_t)hidHandle, kUSB_DeviceHidEventSetProtocol, &hidHandle->protocol);
                    }
                    break;
                default:
                    error = kStatus_USB_InvalidRequest;
                    break;
            }
        }
        break;
    default:
        break;
    }
    return error;
}

usb_status_t
usb_dev_hid_init(uint8_t controllerId,
                 usb_dev_class_config_t *config,
                 class_handle_t *handle)
{
    usb_device_hid_struct_t *hidHandle;
    usb_status_t error = kStatus_USB_Error;

    error = USB_DeviceHidAllocateHandle(&hidHandle);

    if (kStatus_USB_Success != error) {
        return error;
    }

    error = usb_device_class_get_handle(controllerId, &hidHandle->handle);

    if (kStatus_USB_Success != error) {
        return error;
    }

    if (!hidHandle->handle) {
        return kStatus_USB_InvalidHandle;
    }

    hidHandle->configStruct = config;
    hidHandle->configuration = 0;
    hidHandle->alternate = 0xff;

    *handle = (class_handle_t)hidHandle;
    return error;
}

usb_status_t
usb_dev_hid_deinit(class_handle_t handle)
{
    usb_device_hid_struct_t *hidHandle;
    usb_status_t error = kStatus_USB_Error;

    hidHandle = (usb_device_hid_struct_t *)handle;

    if (!hidHandle) {
        return kStatus_USB_InvalidHandle;
    }

    error = USB_DeviceHidEndpointsDeinit(hidHandle);
    USB_DeviceHidFreeHandle(hidHandle);
    return error;
}

usb_status_t
usb_dev_hid_send(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length)
{
    usb_device_hid_struct_t *hidHandle;
    usb_status_t error = kStatus_USB_Error;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }
    hidHandle = (usb_device_hid_struct_t *)handle;

    if (hidHandle->interruptInPipeBusy) {
        return kStatus_USB_Busy;
    }
    error = usb_device_send_req(hidHandle->handle, ep, buffer, length);
    if (kStatus_USB_Success == error) {
        hidHandle->interruptInPipeBusy = 1;
    }
    return error;
}

usb_status_t
usb_dev_hid_recv(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length)
{
    usb_device_hid_struct_t *hidHandle;
    usb_status_t error = kStatus_USB_Error;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }
    hidHandle = (usb_device_hid_struct_t *)handle;

    if (hidHandle->interruptOutPipeBusy) {
        return kStatus_USB_Busy;
    }
    error = usb_device_recv_req(hidHandle->handle, ep, buffer, length);
    if (kStatus_USB_Success == error) {
        hidHandle->interruptOutPipeBusy = 1;
    }
    return error;
}

#endif
