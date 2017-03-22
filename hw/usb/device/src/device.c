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

#include <usb/usb.h>
#include <device/device.h>
#include <device/ch9.h>

#include <hal_usb/hal_usb.h>

#include <os/os.h>

static usb_device_struct_t s_UsbDevice[MYNEWT_VAL(USB_DEVICE_CONFIG_NUM)];
static struct os_eventq notification_queue;
static struct os_event ev;
static usb_device_callback_message_struct_t message;

//FIXME
//static int mhead = 0;
//static int mtail = 0;
//static usb_device_callback_message_struct_t messages[16];

/*!
 * This function allocates a device handle.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param handle          It is out parameter, is used to return pointer of the device handle to the caller.
 */
static usb_status_t
_usb_device_alloc_handle(uint8_t controllerId, usb_device_struct_t * *handle)
{
    uint32_t i;
    os_sr_t sr;
    usb_status_t status = kStatus_USB_Busy;

    OS_ENTER_CRITICAL(sr);

    /* Check the controller is initialized or not. */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDevice[i].controllerHandle &&
                s_UsbDevice[i].controllerId == controllerId) {
            status = kStatus_USB_Error;
            goto done;
        }
    }

    /* Get a free device handle. */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (!s_UsbDevice[i].controllerHandle) {
            s_UsbDevice[i].controllerId = controllerId;
            *handle = &s_UsbDevice[i];
            status = kStatus_USB_Success;
            goto done;
        }
    }

done:
    OS_EXIT_CRITICAL(sr);
    return status;
}

/*!
 * This function frees a device handle.
 *
 * @param handle          The device handle.
 */
static usb_status_t
_usb_device_free_handle(usb_device_struct_t *handle)
{
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);
    handle->controllerHandle = NULL;
    handle->controllerId = 0;
    OS_EXIT_CRITICAL(sr);

    return kStatus_USB_Success;
}

/*!
 * This function is used to get the controller interface handle.
 *
 * @param controllerId          The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param controllerInterface   It is out parameter, is used to return pointer of the device controller handle to the
 * caller.
 */

#if 0
static usb_status_t
_usb_device_get_controller_interface(uint8_t controllerId,
                                     const usb_device_controller_interface_struct_t * *controllerInterface)
{
    return error;
}
#endif

/*!
 * @brief Start a new transfer.
 *
 * This function is used to start a new transfer.
 *
 * @param endpointAddress       Endpoint address. Bit7 is direction, 0U - USB_OUT, 1U - USB_IN.
 * @param buffer                 The memory address to be transferred, or the memory address to hold the data need to be
 * sent.
 * @param length                 The length of the data.
 *
 * @retval kStatus_USB_Success              Get a device handle successfully.
 * @retval kStatus_USB_InvalidHandle        The device handle is invalided.
 * @retval kStatus_USB_ControllerNotFound   The controller interface is not found.
 * @retval kStatus_USB_Error                The device is doing reset.
 */
static usb_status_t
_usb_device_transfer(usb_device_handle handle,
                     uint8_t endpointAddress,
                     uint8_t *buffer,
                     uint32_t length)
{
    usb_device_struct_t *deviceHandle = (usb_device_struct_t *)handle;
    usb_status_t error = kStatus_USB_Error;
    uint8_t endpoint = USB_EP_NUMBER(endpointAddress);
    uint8_t direction = USB_EP_DIR(endpointAddress);

    if (!deviceHandle) {
        return kStatus_USB_InvalidHandle;
    }

    if (deviceHandle->controllerInterface) {
        if (deviceHandle->endpointCallback[(uint8_t)((uint32_t)endpoint <<
            1U) | direction].isBusy) {
            return kStatus_USB_Busy;
        }
        deviceHandle->endpointCallback[(uint8_t)((uint32_t)endpoint << 1U) |
                                       direction].isBusy = 1U;
        if (endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) {
#if (defined(USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE) && \
            (USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE > 0U))
            if (length) {
                USB_CacheFlushLines((void *)buffer, length);
            }
#endif
            /* Call the controller send interface. */
            error = deviceHandle->controllerInterface->deviceSend(
                deviceHandle->controllerHandle, endpointAddress,
                buffer,
                length);
        } else {
#if (defined(USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE) && \
            (USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE > 0))
            if (length) {
                USB_CacheInvalidateLines((void *)buffer, length);
            }
#endif
            /* Call the controller receive interface. */
            error = deviceHandle->controllerInterface->deviceRecv(
                deviceHandle->controllerHandle, endpointAddress,
                buffer,
                length);
        }
    } else {
        error = kStatus_USB_ControllerNotFound;
    }
    return error;
}

static usb_status_t
_usb_device_control(usb_device_handle handle, usb_device_control_type_t type, void *param)
{
    usb_device_struct_t *deviceHandle = (usb_device_struct_t *)handle;

    if (!deviceHandle) {
        return kStatus_USB_InvalidHandle;
    }

    if (!deviceHandle->controllerInterface) {
        return kStatus_USB_ControllerNotFound;
    }

    return deviceHandle->controllerInterface->deviceControl(
        deviceHandle->controllerHandle, type, param);
}

/*!
 * @brief Handle the reset notification.
 *
 * This function is used to handle the reset notification.
 *
 * @param message                The device callback message handle.
 *
 * @retval kStatus_USB_Success              Get a device handle successfully.
 */
static usb_status_t
_usb_device_reset_notification(usb_device_struct_t *handle,
                              usb_device_callback_message_struct_t *message)
{
    int i;

    handle->isResetting = 1;

#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    /* Clear remote wakeup feature */
    handle->remotewakeup = 0;
#endif
    /* Set the controller to default status. */
    _usb_device_control(handle, USB_DEV_CTRL_SET_DFLT_STATUS, NULL);

    handle->state = kUSB_DeviceStateDefault;
    handle->deviceAddress = 0;

    for (i = 0; i < (MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS) * 2); i++) {
        handle->endpointCallback[i].callbackFn = NULL;
        handle->endpointCallback[i].callbackParam = NULL;
        handle->endpointCallback[i].isBusy = 0;
    }

    handle->deviceCallback(handle, kUSB_DeviceEventBusReset, NULL);

    handle->isResetting = 0;
    return kStatus_USB_Success;
}

#if MYNEWT_VAL(USB_DEVICE_CONFIG_LOW_POWER_MODE)
static usb_status_t
_usb_device_suspend_notification(usb_device_struct_t *handle,
                                 usb_device_callback_message_struct_t *message)
{
    return handle->deviceCallback(handle, kUSB_DeviceEventSuspend, NULL);
}

static usb_status_t
_usb_device_resume_notification(usb_device_struct_t *handle,
                                usb_device_callback_message_struct_t *message)
{
    return handle->deviceCallback(handle, kUSB_DeviceEventResume, NULL);
}
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_DETACH_ENABLE)
static usb_status_t
_usb_device_detach_notification(usb_device_struct_t *handle,
                                usb_device_callback_message_struct_t *message)
{
    return handle->deviceCallback(handle, kUSB_DeviceEventDetach, NULL);
}

static usb_status_t
_usb_device_attach_notification(usb_device_struct_t *handle,
                                usb_device_callback_message_struct_t *message)
{
    return handle->deviceCallback(handle, kUSB_DeviceEventAttach, NULL);
}
#endif

static usb_status_t
_usb_device_notification(usb_device_struct_t *handle,
                         usb_device_callback_message_struct_t *message)
{
    uint8_t endpoint = USB_EP_NUMBER(message->code);
    uint8_t direction = USB_EP_DIR(message->code);
    usb_status_t error = kStatus_USB_Error;
    uint8_t epidx;

    //printf("_usb_device_notification: message->code=%d\n", message->code);
    switch (message->code) {
    case kUSB_DeviceNotifyBusReset:
        error = _usb_device_reset_notification(handle, message);
        break;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_LOW_POWER_MODE)
    case kUSB_DeviceNotifySuspend:
        error = _usb_device_suspend_notification(handle, message);
        break;
    case kUSB_DeviceNotifyResume:
        error = _usb_device_resume_notification(handle, message);
        break;
#endif

        //FIXME
#if (defined(USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING) && \
        USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING) || \
        (defined(USB_DEVICE_CONFIG_EHCI_ERROR_HANDLING) && \
        USB_DEVICE_CONFIG_EHCI_ERROR_HANDLING)
    case kUSB_DeviceNotifyError:
        error = USB_DeviceErrorNotification(handle, message);
        break;
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_DETACH_ENABLE)
    case kUSB_DeviceNotifyDetach:
        error = _usb_device_detach_notification(handle, message);
        break;
    case kUSB_DeviceNotifyAttach:
        error = _usb_device_attach_notification(handle, message);
        break;
#endif

    default:
        if (endpoint < MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
            epidx = (uint8_t)((uint32_t)endpoint<< 1) | direction;
            if (handle->endpointCallback[epidx].callbackFn) {
                usb_device_endpoint_callback_message_struct_t
                    endpointCallbackMessage;
                endpointCallbackMessage.buffer = message->buffer;
                endpointCallbackMessage.length = message->length;
                endpointCallbackMessage.isSetup = message->isSetup;
                if (message->isSetup) {
                    handle->endpointCallback[0].isBusy = 0;
                    handle->endpointCallback[1].isBusy = 0;
                } else {
                    handle->endpointCallback[epidx].isBusy = 0;
                }
                error = handle->endpointCallback[epidx].callbackFn(
                    handle,
                    &endpointCallbackMessage,
                    handle->endpointCallback[epidx].callbackParam);
            }
        }
        break;
    }
    return error;
}

/*!
 * @brief Notify the device that the controller status changed.
 *
 * This function is used to notify the device that the controller status changed.
 *
 * @param message                The device callback message handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t
usb_dev_notify(void *handle, void *msg)
{
    usb_device_struct_t *deviceHandle = (usb_device_struct_t *)handle;
    //usb_device_callback_message_struct_t *message = (usb_device_callback_message_struct_t *)msg;

    memcpy(&message, (usb_device_callback_message_struct_t *) msg, sizeof message);

    if (!msg || !handle) {
        return kStatus_USB_InvalidHandle;
    }

    if (!deviceHandle->deviceCallback) {
        return kStatus_USB_Error;
    }

    if (deviceHandle->isResetting) {
        if (USB_EP_NUMBER(message.code) && !(message.code & 0x70)) {
            return _usb_device_notification(deviceHandle, &message);
        }
    }

    //FIXME: are events copied???
    ev.ev_arg = &message;
    os_eventq_put(deviceHandle->notificationQueue, &ev);

    return kStatus_USB_Success;
}

/*!
 * This function initializes the USB device module specified by the controllerId.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param deviceCallback Function pointer of the device callback.
 * @param handle          It is out parameter, is used to return pointer of the device handle to the caller.
 *
 * @retval kStatus_USB_Success              The device is initialized successfully.
 * @retval kStatus_USB_InvalidHandle        The handle is a NULL pointer.
 * @retval kStatus_USB_Busy                 Cannot allocate a device handle.
 * @retval kStatus_USB_ControllerNotFound   Cannot find the controller according to the controller id.
 * @retval kStatus_USB_InvalidControllerInterface  The controller driver interfaces is invaild, There is an empty
 * interface entity.
 * @retval kStatus_USB_Error                The macro USB_DEVICE_CONFIG_ENDPOINTS is more than IP's endpoint number.
 *                                          Or, the device has been initialized.
 *                                          Or, the message queue is created failed.
 */
usb_status_t
usb_device_init(uint8_t controllerId, usb_device_callback_t deviceCallback,
                usb_device_handle *handle)
{
    usb_device_struct_t *deviceHandle = NULL;
    usb_status_t error;
    uint32_t count;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    /* Allocate a device handle by using the controller id. */
    error = _usb_device_alloc_handle(controllerId, &deviceHandle);

    if (error != kStatus_USB_Success) {
        return error;
    }

    deviceHandle->deviceCallback = deviceCallback;
    deviceHandle->controllerId = controllerId;
    deviceHandle->deviceAddress = 0;
    deviceHandle->isResetting = 0;

    for (count = 0; count < (MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS) * 2); count++) {
        deviceHandle->endpointCallback[count].callbackFn = NULL;
        deviceHandle->endpointCallback[count].callbackParam = NULL;
        deviceHandle->endpointCallback[count].isBusy = 0;
    }

    deviceHandle->controllerInterface = usb_hal_controller_interface();
    if (!deviceHandle->controllerInterface) {
        _usb_device_free_handle(deviceHandle);
        return kStatus_USB_ControllerNotFound;
    }
    if (!deviceHandle->controllerInterface->deviceInit   ||
        !deviceHandle->controllerInterface->deviceDeinit ||
        !deviceHandle->controllerInterface->deviceSend   ||
        !deviceHandle->controllerInterface->deviceRecv   ||
        !deviceHandle->controllerInterface->deviceCancel ||
        !deviceHandle->controllerInterface->deviceControl) {
        _usb_device_free_handle(deviceHandle);
        return kStatus_USB_InvalidControllerInterface;
    }

    os_eventq_init(&notification_queue);
    deviceHandle->notificationQueue = &notification_queue;

    /* Initialize the controller */
    error = deviceHandle->controllerInterface->deviceInit(controllerId,
                                                          deviceHandle,
                                                          &deviceHandle->controllerHandle);
    if (error != kStatus_USB_Success) {
        usb_device_deinit(deviceHandle);
        return error;
    }

    deviceHandle->state = kUSB_DeviceStateDefault;
    *handle = deviceHandle;

    return error;
}

usb_status_t
usb_device_run(usb_device_handle handle)
{
    return _usb_device_control(handle, USB_DEV_CTRL_RUN, NULL);
}

usb_status_t
usb_device_stop(usb_device_handle handle)
{
    return _usb_device_control(handle, USB_DEV_CTRL_STOP, NULL);
}

usb_status_t
usb_device_deinit(usb_device_handle handle)
{
    usb_device_struct_t *deviceHandle = (usb_device_struct_t *)handle;

    if (!deviceHandle) {
        return kStatus_USB_InvalidHandle;
    }

    if (deviceHandle->controllerInterface) {
        deviceHandle->controllerInterface->deviceDeinit(deviceHandle->controllerHandle);
        deviceHandle->controllerInterface = NULL;
    }

    if (deviceHandle->notificationQueue) {
        deviceHandle->notificationQueue = NULL;
    }

    _usb_device_free_handle(deviceHandle);
    return kStatus_USB_Success;
}

/*!
 * @brief Send data through a specified endpoint.
 *
 * The function is used to send data through a specified endpoint.
 *
 * @param endpointAddress Endpoint index.
 * @param buffer The memory address to hold the data need to be sent.
 * @param length The data length need to be sent.
 *
 * @retval kStatus_USB_Success              The send request is sent successfully.
 * @retval kStatus_USB_InvalidHandle        The handle is a NULL pointer. Or the controller handle is invalid.
 * @retval kStatus_USB_Busy                 Cannot allocate dtds for current tansfer in EHCI driver.
 * @retval kStatus_USB_ControllerNotFound   Cannot find the controller.
 * @retval kStatus_USB_Error                The device is doing reset.
 *
 * @note The return value just means if the sending request is successful or not; the transfer done is notified by the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t
usb_device_send_req(usb_device_handle handle, uint8_t endpointAddress,
                    uint8_t *buffer, uint32_t length)
{
    return _usb_device_transfer(handle, USB_EP_NUMBER(endpointAddress) |
            (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT),
        buffer, length);
}

/*!
 * @brief Receive data through a specified endpoint.
 *
 * The function is used to receive data through a specified endpoint.
 *
 * @param endpointAddress Endpoint index.
 * @param buffer The memory address to save the received data.
 * @param length The data length want to be received.
 *
 * @retval kStatus_USB_Success              The receive request is sent successfully.
 * @retval kStatus_USB_InvalidHandle        The handle is a NULL pointer. Or the controller handle is invalid.
 * @retval kStatus_USB_Busy                 Cannot allocate dtds for current tansfer in EHCI driver.
 * @retval kStatus_USB_ControllerNotFound   Cannot find the controller.
 * @retval kStatus_USB_Error                The device is doing reset.
 *
 * @note The return value just means if the receiving request is successful or not; the transfer done is notified by the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t
usb_device_recv_req(usb_device_handle handle, uint8_t endpointAddress,
                    uint8_t *buffer, uint32_t length)
{
    return _usb_device_transfer(handle, USB_EP_NUMBER(endpointAddress) |
            (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT),
        buffer, length);
}

usb_status_t
usb_device_cancel(usb_device_handle handle, uint8_t ep)
{
    usb_device_struct_t *dev = (usb_device_struct_t *)handle;

    if (!dev) {
        return kStatus_USB_InvalidHandle;
    }

    if (!dev->controllerInterface) {
        return kStatus_USB_ControllerNotFound;
    }

    return dev->controllerInterface->deviceCancel(dev->controllerHandle, ep);
}

usb_status_t
usb_dev_ep_init(usb_device_handle handle,
                usb_device_endpoint_init_struct_t *ep_init,
                usb_device_endpoint_callback_struct_t *ep_cb)
{
    usb_device_struct_t *dev = (usb_device_struct_t *)handle;
    uint8_t endpoint;
    uint8_t direction;
    uint8_t epidx;

    if (!dev) {
        return kStatus_USB_InvalidHandle;
    }

    if (!ep_init || !ep_cb) {
        return kStatus_USB_InvalidParameter;
    }

    endpoint = USB_EP_NUMBER(ep_init->endpointAddress);
    direction = USB_EP_DIR(ep_init->endpointAddress);

    if (endpoint >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return kStatus_USB_InvalidParameter;
    }

    epidx = (uint8_t)((uint32_t)endpoint << 1) | direction;
    dev->endpointCallback[epidx].callbackFn = ep_cb->callbackFn;
    dev->endpointCallback[epidx].callbackParam = ep_cb->callbackParam;
    dev->endpointCallback[epidx].isBusy = 0;

    return _usb_device_control(handle, USB_DEV_CTRL_EP_INIT, ep_init);
}

usb_status_t
usb_dev_ep_deinit(usb_device_handle handle, uint8_t endpointAddress)
{
    usb_device_struct_t *dev = (usb_device_struct_t *)handle;
    uint8_t endpoint = USB_EP_NUMBER(endpointAddress);
    uint8_t direction = USB_EP_DIR(endpointAddress);
    uint8_t epidx;
    usb_status_t error = kStatus_USB_Error;

    if (!dev) {
        return kStatus_USB_InvalidHandle;
    }

    error = _usb_device_control(handle, USB_DEV_CTRL_EP_DEINIT, &endpointAddress);
    if (endpoint >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return kStatus_USB_InvalidParameter;
    }

    epidx = (uint8_t) ((uint32_t)endpoint << 1) | direction;
    dev->endpointCallback[epidx].callbackFn = NULL;
    dev->endpointCallback[epidx].callbackParam = NULL;
    dev->endpointCallback[epidx].isBusy = 0;

    return error;
}

usb_status_t
usb_dev_ep_stall(usb_device_handle handle, uint8_t ep_addr)
{
    if (USB_EP_NUMBER(ep_addr) >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return kStatus_USB_InvalidParameter;
    }

    return _usb_device_control(handle, USB_DEV_CTRL_EP_STALL, &ep_addr);
}

usb_status_t
usb_dev_ep_unstall(usb_device_handle handle, uint8_t ep_addr)
{
    if (USB_EP_NUMBER(ep_addr) >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return kStatus_USB_InvalidParameter;
    }

    return _usb_device_control(handle, USB_DEV_CTRL_EP_UNSTALL, &ep_addr);
}

/*!
 * The function is used to get the status of the selected item.
 *
 * @param type   The selected item. Please refer to the structure usb_device_status_t.
 * @param param  The param type is determined by the selected item.
 *
 * @retval kStatus_USB_Success              Get status successfully.
 * @retval kStatus_USB_InvalidHandle        The handle is a NULL pointer. Or the controller handle is invalid.
 * @retval kStatus_USB_InvalidParameter     The param is NULL pointer.
 * @retval kStatus_USB_ControllerNotFound   Cannot find the controller.
 * @retval kStatus_USB_Error                Unsupported type.
 */
usb_status_t
usb_dev_get_status(usb_device_handle handle, usb_device_status_t type, void *param)
{
    uint8_t *temp8;
    usb_status_t error = kStatus_USB_Error;

    if (!param) {
        return kStatus_USB_InvalidParameter;
    }

    switch (type) {
        case kUSB_DeviceStatusSpeed:
            error =
                _usb_device_control(handle, USB_DEV_CTRL_GET_SPEED, param);
            break;
        case kUSB_DeviceStatusOtg:
            error = _usb_device_control(handle, USB_DEV_CTRL_GET_OTG_STATUS, param);
            break;
        case kUSB_DeviceStatusDeviceState:
            temp8 = (uint8_t *)param;
            error = kStatus_USB_Success;
            *temp8 = ((usb_device_struct_t *)handle)->state;
            break;
        case kUSB_DeviceStatusAddress:
            temp8 = (uint8_t *)param;
            error = kStatus_USB_Success;
            *temp8 = ((usb_device_struct_t *)handle)->deviceAddress;
            break;
        case kUSB_DeviceStatusDevice:
            error = _usb_device_control(handle, USB_DEV_CTRL_GET_STATUS, param);
            break;
        case kUSB_DeviceStatusEndpoint:
            error = _usb_device_control(handle, USB_DEV_CTRL_GET_EP_STATUS, param);
            break;
        case kUSB_DeviceStatusSynchFrame:
            error = _usb_device_control(handle, USB_DEV_CTRL_GET_SYNCF, param);
            break;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
        case kUSB_DeviceStatusRemoteWakeup:
            temp8 = (uint8_t *)param;
            error = kStatus_USB_Success;
            *temp8 = ((usb_device_struct_t *)handle)->remotewakeup;
            break;
#endif
        default:
            break;
    }
    return error;
}

/*!
 * The function is used to set the status of the selected item.
 *
 * @param type The selected item. Please refer to the structure usb_device_status_t.
 * @param param The param type is determined by the selected item.
 *
 * @retval kStatus_USB_Success              Set status successfully.
 * @retval kStatus_USB_InvalidHandle        The handle is a NULL pointer. Or the controller handle is invalid.
 * @retval kStatus_USB_ControllerNotFound   Cannot find the controller.
 * @retval kStatus_USB_Error                Unsupported type, or the param is NULL pointer.
 */
usb_status_t
usb_dev_set_status(usb_device_handle handle, usb_device_status_t type, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    switch (type) {
//FIXME
#if MYNEWT_VAL(USB_DEVICE_CONFIG_EHCI) && MYNEWT_VAL(USB_DEVICE_CONFIG_EHCI_TEST_MODE)
        case kUSB_DeviceStatusTestMode:
            error = _usb_device_control(handle, USB_DEV_CTRL_SET_TEST_MODE, param);
            break;
#endif
        case kUSB_DeviceStatusOtg:
            error = _usb_device_control(handle, USB_DEV_CTRL_SET_OTG_STATUS, param);
            break;
        case kUSB_DeviceStatusDeviceState:
            if (NULL != param) {
                error = kStatus_USB_Success;
                ((usb_device_struct_t *)handle)->state =
                    (uint8_t)(*(uint8_t *)param);
            }
            break;
        case kUSB_DeviceStatusAddress:
            if (kUSB_DeviceStateAddressing !=
                ((usb_device_struct_t *)handle)->state) {
                if (NULL != param) {
                    error = kStatus_USB_Success;
                    ((usb_device_struct_t *)handle)->deviceAddress =
                        (uint8_t)(*(uint8_t *)param);
                    ((usb_device_struct_t *)handle)->state =
                        kUSB_DeviceStateAddressing;
                }
            } else {
                error = _usb_device_control(handle, USB_DEV_CTRL_SET_ADDR,
                                            &((usb_device_struct_t *)handle)->deviceAddress);
            }
            break;
        case kUSB_DeviceStatusBusResume:
            error = _usb_device_control(handle, USB_DEV_CTRL_RESUME, param);
            break;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
        case kUSB_DeviceStatusRemoteWakeup:
            if (NULL != param) {
                error = kStatus_USB_Success;
                ((usb_device_struct_t *)handle)->remotewakeup =
                    (uint8_t)(*(uint8_t *)param);
            }
            break;
#endif
        case kUSB_DeviceStatusBusSuspend:
            error =
                _usb_device_control(handle, USB_DEV_CTRL_SUSPEND, param);
            break;
        default:
            break;
    }
    return error;
}

/*
 * This function is used to handle controller message.
 * This function should not be called in application directly.
 */
void
usb_device_task_fn(void *deviceHandle)
{
    usb_device_struct_t *handle = (usb_device_struct_t *)deviceHandle;
    struct os_event *ev;

    if (handle) {
        ev = os_eventq_get(handle->notificationQueue);
        //printf("new usb event\n");
        _usb_device_notification(handle, ev->ev_arg);
    }
}

/*!
 * @brief Get dvice stack version function.
 *
 * The function is used to get dvice stack version.
 *
 * @param[out] version The version structure pointer to keep the device stack version.
 *
 */
void
usb_device_get_version(uint32_t *version)
{
    if (version) {
        *version =
            (uint32_t)USB_MAKE_VERSION(USB_STACK_VERSION_MAJOR,
                                       USB_STACK_VERSION_MINOR,
                                       USB_STACK_VERSION_BUGFIX);
    }
}

#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
usb_status_t
usb_device_update_hw_tick(usb_device_handle handle, uint64_t tick)
{
    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    ((usb_device_struct_t *) handle)->hwTick = tick;
    return kStatus_USB_Success;
}
#endif
