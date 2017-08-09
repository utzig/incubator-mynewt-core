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
#ifndef _USB_CLASS_CDC_H_
#define _USB_CLASS_CDC_H_

#include <os/os.h>

#define USB_DEVICE_CONFIG_CDC_ACM_MAX_INSTANCE                  1
#define USB_DEVICE_CONFIG_CDC_COMM_CLASS_CODE                   0x02
#define USB_DEVICE_CONFIG_CDC_DATA_CLASS_CODE                   0x0A

#define USB_DEVICE_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND        0x00
#define USB_DEVICE_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE        0x01
#define USB_DEVICE_CDC_REQUEST_SET_COMM_FEATURE                 0x02
#define USB_DEVICE_CDC_REQUEST_GET_COMM_FEATURE                 0x03
#define USB_DEVICE_CDC_REQUEST_CLEAR_COMM_FEATURE               0x04
#define USB_DEVICE_CDC_REQUEST_SET_AUX_LINE_STATE               0x10
#define USB_DEVICE_CDC_REQUEST_SET_HOOK_STATE                   0x11
#define USB_DEVICE_CDC_REQUEST_PULSE_SETUP                      0x12
#define USB_DEVICE_CDC_REQUEST_SEND_PULSE                       0x13
#define USB_DEVICE_CDC_REQUEST_SET_PULSE_TIME                   0x14
#define USB_DEVICE_CDC_REQUEST_RING_AUX_JACK                    0x15
#define USB_DEVICE_CDC_REQUEST_SET_LINE_CODING                  0x20
#define USB_DEVICE_CDC_REQUEST_GET_LINE_CODING                  0x21
#define USB_DEVICE_CDC_REQUEST_SET_CONTROL_LINE_STATE           0x22
#define USB_DEVICE_CDC_REQUEST_SEND_BREAK                       0x23
#define USB_DEVICE_CDC_REQUEST_SET_RINGER_PARAMS                0x30
#define USB_DEVICE_CDC_REQUEST_GET_RINGER_PARAMS                0x31
#define USB_DEVICE_CDC_REQUEST_SET_OPERATION_PARAM              0x32
#define USB_DEVICE_CDC_REQUEST_GET_OPERATION_PARAM              0x33
#define USB_DEVICE_CDC_REQUEST_SET_LINE_PARAMS                  0x34
#define USB_DEVICE_CDC_REQUEST_GET_LINE_PARAMS                  0x35
#define USB_DEVICE_CDC_REQUEST_DIAL_DIGITS                      0x36
#define USB_DEVICE_CDC_REQUEST_SET_UNIT_PARAMETER               0x37
#define USB_DEVICE_CDC_REQUEST_GET_UNIT_PARAMETER               0x38
#define USB_DEVICE_CDC_REQUEST_CLEAR_UNIT_PARAMETER             0x39
#define USB_DEVICE_CDC_REQUEST_SET_ETHERNET_MULTICAST_FILTERS   0x40
#define USB_DEVICE_CDC_REQUEST_SET_ETHERNET_POW_PATTER_FILTER   0x41
#define USB_DEVICE_CDC_REQUEST_GET_ETHERNET_POW_PATTER_FILTER   0x42
#define USB_DEVICE_CDC_REQUEST_SET_ETHERNET_PACKET_FILTER       0x43
#define USB_DEVICE_CDC_REQUEST_GET_ETHERNET_STATISTIC           0x44
#define USB_DEVICE_CDC_REQUEST_SET_ATM_DATA_FORMAT              0x50
#define USB_DEVICE_CDC_REQUEST_GET_ATM_DEVICE_STATISTICS        0x51
#define USB_DEVICE_CDC_REQUEST_SET_ATM_DEFAULT_VC               0x52
#define USB_DEVICE_CDC_REQUEST_GET_ATM_VC_STATISTICS            0x53
#define USB_DEVICE_CDC_REQUEST_MDLM_SPECIFIC_REQUESTS_MASK      0x7F

#define USB_DEVICE_CDC_NOTIF_NETWORK_CONNECTION                 0x00
#define USB_DEVICE_CDC_NOTIF_RESPONSE_AVAIL                     0x01
#define USB_DEVICE_CDC_NOTIF_AUX_JACK_HOOK_STATE                0x08
#define USB_DEVICE_CDC_NOTIF_RING_DETECT                        0x09
#define USB_DEVICE_CDC_NOTIF_SERIAL_STATE                       0x20
#define USB_DEVICE_CDC_NOTIF_CALL_STATE_CHANGE                  0x28
#define USB_DEVICE_CDC_NOTIF_LINE_STATE_CHANGE                  0x29
#define USB_DEVICE_CDC_NOTIF_CONNECTION_SPEED_CHANGE            0x2A

#define USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE                   0x01
#define USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING                  0x02

#define USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION    0x02
#define USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE          0x01
#define USB_DEVICE_CDC_UART_STATE_RX_CARRIER                    0x01
#define USB_DEVICE_CDC_UART_STATE_TX_CARRIER                    0x02
#define USB_DEVICE_CDC_UART_STATE_BREAK                         0x04
#define USB_DEVICE_CDC_UART_STATE_RING_SIGNAL                   0x08
#define USB_DEVICE_CDC_UART_STATE_FRAMING                       0x10
#define USB_DEVICE_CDC_UART_STATE_PARITY                        0x20
#define USB_DEVICE_CDC_UART_STATE_OVERRUN                       0x40

typedef enum
{
    kUSB_DeviceCdcEventSendResponse = 0x01,
    kUSB_DeviceCdcEventRecvResponse,
    kUSB_DeviceCdcEventSerialStateNotif,
    kUSB_DeviceCdcEventSendEncapsulatedCommand,
    kUSB_DeviceCdcEventGetEncapsulatedResponse,
    kUSB_DeviceCdcEventSetCommFeature,
    kUSB_DeviceCdcEventGetCommFeature,
    kUSB_DeviceCdcEventClearCommFeature,
    kUSB_DeviceCdcEventGetLineCoding,
    kUSB_DeviceCdcEventSetLineCoding,
    kUSB_DeviceCdcEventSetControlLineState,
    kUSB_DeviceCdcEventSendBreak,
} usb_device_cdc_acm_event_t;

typedef struct
{
    uint8_t  **buffer;       /*!< The pointer to the address of the buffer for CDC class request. */
    uint32_t *length;        /*!< The pointer to the length of the buffer for CDC class request. */
    uint16_t interfaceIndex; /*!< The interface index of the setup packet. */
    uint16_t setupValue;     /*!< The wValue field of the setup packet. */
    uint8_t  isSetup;        /*!< The flag indicates if it is a setup packet, 1: yes, 0: no. */
} usb_device_cdc_acm_request_param_struct_t;

typedef struct
{
    struct os_mutex mutex;      /*!< The mutex of the pipe. */
    uint8_t ep;                 /*!< The endpoint number of the pipe. */
    uint8_t isBusy;             /*!< 1: The pipe is transferring packet, 0: The pipe is idle. */
} usb_device_cdc_acm_pipe_t;

typedef struct
{
    usb_device_handle handle;                           /*!< The handle of the USB device. */
    usb_dev_class_config_t *configStruct;               /*!< The class configure structure. */
    usb_device_interface_struct_t *commInterfaceHandle; /*!< The CDC communication interface handle. */
    usb_device_interface_struct_t *dataInterfaceHandle; /*!< The CDC data interface handle. */
    usb_device_cdc_acm_pipe_t bulkIn;                   /*!< The bulk in pipe for sending packet to host. */
    usb_device_cdc_acm_pipe_t bulkOut;                  /*!< The bulk out pipe for receiving packet from host. */
    usb_device_cdc_acm_pipe_t interruptIn; /*!< The interrupt in pipe for notifying the device state to host. */
    uint8_t configuration;                 /*!< The current configuration value. */
    uint8_t interfaceNumber;               /*!< The current interface number. */
    uint8_t alternate;                     /*!< The alternate setting value of the interface. */
    uint8_t hasSentState; /*!< 1: The device has primed the state in interrupt pipe, 0: Not primed the state. */
} usb_device_cdc_acm_struct_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name USB CDC ACM Class Driver
 * @{
 */
/*!
 * This function obtains a USB device handle according to the controller ID,
 * initializes the CDC ACM class with the class configure parameters and
 * creates the mutex for each pipe.
 *
 * @param controllerId The ID of the controller. The value can be chosen from the
 *      kUSB_ControllerKhci0, kUSB_ControllerKhci1, kUSB_ControllerEhci0, or
 *      kUSB_ControllerEhci1.
 * @param config The user configuration structure of type usb_dev_class_config_t.
 *      The user populates the members of this structure and passes the pointer
 *      of this structure into this function.
 */
usb_status_t usb_dev_cdc_init(uint8_t controllerId,
                              usb_dev_class_config_t *config,
                              class_handle_t *handle);

/*!
 * This function destroys the mutex for each pipe, deinitializes each endpoint
 * of the CDC ACM class and frees the CDC ACM class handle.
 */
usb_status_t usb_dev_cdc_deinit(class_handle_t handle);

/*!
 * This function responds to various events including the common device events
 * and the class-specific events. For class-specific events, it calls the class
 * callback defined in the application to deal with the class-specific event.
 */
usb_status_t usb_dev_cdc_event(void *handle, uint32_t event, void *param);

/*!
 * This function checks whether the endpoint is sending packet, then it primes
 * the endpoint with the buffer address and the buffer length if the pipe is not
 * busy. Otherwise, it ignores this transfer by returning an error code.
 *
 * @param handle The class handle of the CDC ACM class.
 * @param ep The endpoint number of the transfer.
 * @param buffer The pointer to the buffer to be transferred.
 * @param length The length of the buffer to be transferred.
 */
usb_status_t usb_dev_cdc_send(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);

/*!
 * This function checks whether the endpoint is receiving packet, then it primes
 * the endpoint with the buffer address and the buffer length if the pipe is not
 * busy. Otherwise, it ignores this transfer by returning an error code.
 *
 * @param handle The class handle of the CDC ACM class.
 * @param ep The endpoint number of the transfer.
 * @param buffer The pointer to the buffer to be transferred.
 * @param length The length of the buffer to be transferred.
 */
usb_status_t usb_dev_cdc_recv(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);

/*! @}*/

#if defined(__cplusplus)
}
#endif

#endif /* _USB_CLASS_CDC_H_ */
