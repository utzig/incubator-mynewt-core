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

#ifndef DEV_H
#define DEV_H

#include <stdint.h>
#include <usb/usb.h>

typedef enum _usb_device_status
{
    kUSB_DeviceStatusTestMode = 1,  /*!< Test mode */
    kUSB_DeviceStatusSpeed,         /*!< Current speed */
    kUSB_DeviceStatusOtg,           /*!< OTG status */
    kUSB_DeviceStatusDevice,        /*!< Device status */
    kUSB_DeviceStatusEndpoint,      /*!< Endpoint state usb_device_endpoint_status_t */
    kUSB_DeviceStatusDeviceState,   /*!< Device state */
    kUSB_DeviceStatusAddress,       /*!< Device address */
    kUSB_DeviceStatusSynchFrame,    /*!< Current frame */
    kUSB_DeviceStatusBus,           /*!< Bus status */
    kUSB_DeviceStatusBusSuspend,    /*!< Bus suspend */
    kUSB_DeviceStatusBusResume,     /*!< Bus resume */
    kUSB_DeviceStatusRemoteWakeup,  /*!< Remote wakeup state */
} usb_device_status_t;

typedef enum _usb_device_state
{
    kUSB_DeviceStateConfigured = 0,  /*!< Device state, Configured*/
    kUSB_DeviceStateAddress,         /*!< Device state, Address*/
    kUSB_DeviceStateDefault,         /*!< Device state, Default*/
    kUSB_DeviceStateAddressing,      /*!< Device state, Address setting*/
    kUSB_DeviceStateTestMode,        /*!< Device state, Test mode*/
} usb_device_state_t;

typedef enum _usb_endpoint_status
{
    kUSB_DeviceEndpointStateIdle = 0,  /*!< Endpoint state, idle*/
    kUSB_DeviceEndpointStateStalled,   /*!< Endpoint state, stalled*/
} usb_device_endpoint_status_t;

#define USB_CONTROL_ENDPOINT                                    0
#define USB_CONTROL_MAX_PACKET_SIZE                             64

#define USB_SETUP_PACKET_SIZE                                   8
#define USB_ENDPOINT_NUMBER_MASK                                0x0F

/*! @brief Default invalid value or the endpoint callback length of cancelled transfer */
#define USB_UNINITIALIZED_VAL_32                                0xFFFFFFFF

/*! @brief Available common EVENT types in device callback */
typedef enum _usb_device_event
{
    kUSB_DeviceEventBusReset = 1,                 /*!< USB bus reset signal detected */
    kUSB_DeviceEventSuspend,                      /*!< USB bus suspend signal detected */
    kUSB_DeviceEventResume,                       /*!< USB bus resume signal detected. The resume signal is driven by itself or a host */
    kUSB_DeviceEventError,                        /*!< An error is happened in the bus. */
    kUSB_DeviceEventDetach,                       /*!< USB device is disconnected from a host. */
    kUSB_DeviceEventAttach,                       /*!< USB device is connected to a host. */
    kUSB_DeviceEventSetConfiguration,             /*!< Set configuration. */
    kUSB_DeviceEventSetInterface,                 /*!< Set interface. */
    kUSB_DeviceEventGetDeviceDescriptor,          /*!< Get device descriptor. */
    kUSB_DeviceEventGetConfigurationDescriptor,   /*!< Get configuration descriptor. */
    kUSB_DeviceEventGetStringDescriptor,          /*!< Get string descriptor. */
    kUSB_DeviceEventGetHidDescriptor,             /*!< Get HID descriptor. */
    kUSB_DeviceEventGetHidReportDescriptor,       /*!< Get HID report descriptor. */
    kUSB_DeviceEventGetHidPhysicalDescriptor,     /*!< Get HID physical descriptor. */
    kUSB_DeviceEventGetDeviceQualifierDescriptor, /*!< Get device qualifier descriptor. */
    kUSB_DeviceEventVendorRequest,                /*!< Vendor request. */
    kUSB_DeviceEventSetRemoteWakeup,              /*!< Enable or disable remote wakeup function. */
    kUSB_DeviceEventGetConfiguration,             /*!< Get current configuration index */
    kUSB_DeviceEventGetInterface,                 /*!< Get current interface alternate setting value */
    kUSB_DeviceEventSetBHNPEnable,
} usb_device_event_t;

typedef struct _usb_device_endpoint_callback_message_struct
{
    uint8_t  *buffer;                             /*!< Transferred buffer */
    uint32_t length;                              /*!< Transferred data length */
    uint8_t  isSetup;                             /*!< Is in a setup phase */
} usb_dev_ep_cb_msg_t;

/*
 * This callback function is used to notify the upper layer what the transfer
 * result is. his callback pointer is passed when a specified endpoint is
 * initialized by calling API #USB_DeviceInitEndpoint.
 */
typedef usb_status_t (*usb_device_endpoint_callback_t)(usb_device_handle handle,
                                                       usb_dev_ep_cb_msg_t *msg,
                                                       void *callbackParam);

/*
 * This callback function is used to notify the upper layer that the device
 * status has changed.
 */
typedef usb_status_t (*usb_device_callback_t)(usb_device_handle handle,
                                              uint32_t callbackEvent,
                                              void *eventParam);

typedef struct _usb_device_endpoint_callback_struct
{
    usb_device_endpoint_callback_t callbackFn;
    void                           *callbackParam;
    uint8_t                        isBusy;
} usb_device_endpoint_callback_struct_t;

typedef struct _usb_device_endpoint_init_struct
{
    uint16_t maxPacketSize;
    uint8_t  endpointAddress;
    uint8_t  transferType;
    uint8_t  zlt;
} usb_device_endpoint_init_struct_t;

typedef struct _usb_device_endpoint_status_struct
{
    uint8_t  endpointAddress;
    uint16_t endpointStatus;
} usb_device_endpoint_status_struct_t;

#define usb_device_controller_handle usb_device_handle

typedef enum _usb_device_notification
{
    kUSB_DeviceNotifyBusReset = 0x10,  /*!< Reset signal detected */
    kUSB_DeviceNotifySuspend,          /*!< Suspend signal detected */
    kUSB_DeviceNotifyResume,           /*!< Resume signal detected */
    kUSB_DeviceNotifyError,            /*!< Errors happened in bus */
    kUSB_DeviceNotifyDetach,           /*!< Device disconnected from a host */
    kUSB_DeviceNotifyAttach,           /*!< Device connected to a host */
} usb_device_notification_t;

typedef struct _usb_device_callback_message_struct
{
    uint8_t  *buffer; /*!< Transferred buffer */
    uint32_t length; /*!< Transferred data length */
    uint8_t  code;   /*!< Notification code */
    uint8_t  isSetup; /*!< Is in a setup phase */
} usb_device_callback_message_struct_t;

/*
 * Device controller interface call table functions
 */

typedef enum _usb_device_control_type
{
    USB_DEV_CTRL_RUN,
    USB_DEV_CTRL_STOP,
    USB_DEV_CTRL_EP_INIT,
    USB_DEV_CTRL_EP_DEINIT,
    USB_DEV_CTRL_EP_STALL,
    USB_DEV_CTRL_EP_UNSTALL,
    USB_DEV_CTRL_GET_STATUS,
    USB_DEV_CTRL_GET_EP_STATUS,
    USB_DEV_CTRL_SET_ADDR,
    USB_DEV_CTRL_GET_SYNCF,
    USB_DEV_CTRL_RESUME,
    USB_DEV_CTRL_SUSPEND,
    USB_DEV_CTRL_SET_DFLT_STATUS,
    USB_DEV_CTRL_GET_SPEED,
    USB_DEV_CTRL_GET_OTG_STATUS,
    USB_DEV_CTRL_SET_OTG_STATUS,
    USB_DEV_CTRL_SET_TEST_MODE,
} usb_device_control_type_t;

typedef usb_status_t (*usb_device_controller_init_t)(uint8_t controllerId,
                                                     usb_device_handle handle,
                                                     usb_device_controller_handle
                                                     *controllerHandle);

typedef usb_status_t (*usb_device_controller_deinit_t)(
    usb_device_controller_handle controllerHandle);

typedef usb_status_t (*usb_device_controller_send_t)(
    usb_device_controller_handle controllerHandle,
    uint8_t endpointAddress,
    uint8_t *buffer,
    uint32_t length);

typedef usb_status_t (*usb_device_controller_recv_t)(
    usb_device_controller_handle controllerHandle,
    uint8_t endpointAddress,
    uint8_t *buffer,
    uint32_t length);

typedef usb_status_t (*usb_device_controller_cancel_t)(
    usb_device_controller_handle controllerHandle,
    uint8_t endpointAddress);

typedef usb_status_t (*usb_device_controller_control_t)(
    usb_device_controller_handle controllerHandle,
    usb_device_control_type_t
    command,
    void *param);

typedef struct _usb_device_controller_interface_struct
{
    usb_device_controller_init_t    deviceInit;
    usb_device_controller_deinit_t  deviceDeinit;
    usb_device_controller_send_t    deviceSend;
    usb_device_controller_recv_t    deviceRecv;
    usb_device_controller_cancel_t  deviceCancel;
    usb_device_controller_control_t deviceControl;
} usb_device_controller_interface_struct_t;

typedef struct
{
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    volatile uint64_t                              hwTick;
#endif
    usb_device_controller_handle                   controllerHandle;
    const usb_device_controller_interface_struct_t *controllerInterface;
    struct os_eventq                               *notificationQueue;
    usb_device_callback_t                          devcb;
    usb_device_endpoint_callback_struct_t          epcbs[MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS) << 1];
    uint8_t                                        deviceAddress;
    uint8_t                                        controllerId;
    uint8_t                                        state;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    uint8_t                                        remotewakeup;
#endif
    uint8_t                                        isResetting;
} usb_dev_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * This function initializes the USB device module specified by the controllerId.
 */
usb_status_t usb_dev_init(uint8_t controllerId,
                          usb_device_callback_t devcb,
                          usb_device_handle *handle);

/*
 * This function enables the device functionality, so that the device can be
 * recognized by the host when the device detects that it has been connected to
 * a host.
 */
usb_status_t usb_device_run(usb_device_handle handle);

/*
 * This function disables the device functionality. After this function called,
 * even if the device is detached to the host, it can't work.
 */
usb_status_t usb_device_stop(usb_device_handle handle);

/*
 * This function de-initializes the device controller specified by the handle.
 */
usb_status_t usb_device_deinit(usb_device_handle handle);

/*!
 * The function is used to send data through a specified endpoint.
 *
 * @param[in] endpointAddress Endpoint index.
 * @param[in] buffer The memory address to hold the data need to be sent. The function is not reentrant.
 * @param[in] length The data length need to be sent.
 *
 * @note The return value indicates whether the sending request is successful or
 * not. The transfer done is notified by the corresponding callback function.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for
 * one specific endpoint, the application should implement a queue on the
 * application level. The subsequent transfer can begin only when the previous
 * transfer is done (get notification through the endpoint callback).
 */
usb_status_t usb_device_send_req(usb_device_handle handle,
                                 uint8_t endpointAddress,
                                 uint8_t *buffer,
                                 uint32_t length);

/*!
 * @brief Receives data through a specified endpoint.
 *
 * The function is used to receive data through a specified endpoint. The function is not reentrant.
 *
 * @param[in] endpointAddress Endpoint index.
 * @param[in] buffer The memory address to save the received data.
 * @param[in] length The data length want to be received.
 *
 * @retval kStatus_USB_Success              The receive request is sent successfully.
 * @retval kStatus_USB_InvalidHandle        The handle is a NULL pointer. Or the controller handle is invalid.
 * @retval kStatus_USB_Busy                 Cannot allocate DTDS for current transfer in EHCI driver.
 * @retval kStatus_USB_ControllerNotFound   Cannot find the controller.
 * @retval kStatus_USB_Error                The device is doing reset.
 *
 * @note The return value indicates whether the receiving request is successful or not. The transfer done is notified by
 * the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue on the application level.
 * The subsequent transfer can begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
usb_status_t usb_device_recv_req(usb_device_handle handle,
                                 uint8_t endpointAddress,
                                 uint8_t *buffer,
                                 uint32_t length);

usb_status_t usb_device_cancel(usb_device_handle handle,
                               uint8_t endpointAddress);

usb_status_t usb_dev_ep_init(usb_device_handle handle,
                             usb_device_endpoint_init_struct_t *epInit,
                             usb_device_endpoint_callback_struct_t *epcbs);

usb_status_t usb_dev_ep_deinit(usb_device_handle handle,
                               uint8_t endpointAddress);

usb_status_t usb_dev_ep_stall(usb_device_handle handle, uint8_t ep_addr);

usb_status_t usb_dev_ep_unstall(usb_device_handle handle, uint8_t ep_addr);

usb_status_t usb_dev_get_status(usb_device_handle handle,
                                usb_device_status_t type,
                                void *param);

usb_status_t usb_dev_set_status(usb_device_handle handle,
                                usb_device_status_t type,
                                void *param);

usb_status_t usb_dev_notify(void *handle, void *msg);

/*
 * The function is used to handle the controller message.
 * This function should not be called in the application directly.
 */
void usb_device_task_fn(void *deviceHandle);

void usb_device_get_version(uint32_t *version);

#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
usb_status_t usb_device_update_hw_tick(usb_device_handle handle, uint64_t tick);
#endif

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

#endif /* DEV_H */
