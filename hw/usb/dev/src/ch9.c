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

#include <dev/dev.h>
#include <dev/class.h>
#include <dev/ch9.h>

#include <stdbool.h>

#if MYNEWT_VAL(USB_DEVICE_CONFIG_NUM)

/* TODO: helpers, might move later... */

static inline bool
is_req_type_class(uint8_t reqtype)
{
    return ((reqtype & USB_REQ_TYPE_TYPE_CLASS) == USB_REQ_TYPE_TYPE_CLASS);
}

static inline bool
is_req_type_std(uint8_t reqtype)
{
    return ((reqtype & USB_REQ_TYPE_TYPE_MASK) == USB_REQ_TYPE_TYPE_STANDARD);
}

static inline bool
is_req_type_vendor(uint8_t reqtype)
{
    return ((reqtype & USB_REQ_TYPE_TYPE_VENDOR) == USB_REQ_TYPE_TYPE_VENDOR);
}

static inline bool
is_req_type_out(uint8_t reqtype)
{
    return ((reqtype & USB_REQ_TYPE_DIR_MASK) == USB_REQ_TYPE_DIR_OUT);
}

static inline bool
is_req_type_in(uint8_t reqtype)
{
    return ((reqtype & USB_REQ_TYPE_DIR_MASK) == USB_REQ_TYPE_DIR_IN);
}

static inline bool
is_recipient_device(uint8_t reqtype)
{
    return ((reqtype & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_DEVICE);
}

static inline bool
is_recipient_interface(uint8_t reqtype)
{
    return ((reqtype & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE);
}

static inline bool
is_recipient_endpoint(uint8_t reqtype)
{
    return ((reqtype & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_ENDPOINT);
}

typedef usb_status_t (*usb_standard_request_callback_t)(
    usb_device_common_class_t *class,
    usb_setup_struct_t *setup,
    uint8_t * *buffer,
    uint32_t *length);

#define DECLARE_STD_REQ_CB(name) static usb_status_t name(       \
    usb_device_common_class_t *, usb_setup_struct_t *,           \
    uint8_t **, uint32_t *)

DECLARE_STD_REQ_CB(_usb_device_get_status);
DECLARE_STD_REQ_CB(_usb_device_set_clear_feature);
DECLARE_STD_REQ_CB(_usb_device_set_address);
DECLARE_STD_REQ_CB(_usb_device_get_descriptor);
DECLARE_STD_REQ_CB(_usb_device_get_configuration);
DECLARE_STD_REQ_CB(_usb_device_set_configuration);
DECLARE_STD_REQ_CB(_usb_device_get_interface);
DECLARE_STD_REQ_CB(_usb_device_set_interface);
DECLARE_STD_REQ_CB(_usb_device_synch_frame);

static const usb_standard_request_callback_t _std_req_table[] = {
    _usb_device_get_status,
    _usb_device_set_clear_feature,
    NULL,
    _usb_device_set_clear_feature,
    NULL,
    _usb_device_set_address,
    _usb_device_get_descriptor,
    NULL,
    _usb_device_get_configuration,
    _usb_device_set_configuration,
    _usb_device_get_interface,
    _usb_device_set_interface,
    _usb_device_synch_frame,
};

static usb_status_t
_usb_device_get_status(usb_device_common_class_t *class,
                       usb_setup_struct_t *setup,
                       uint8_t * *buffer,
                       uint32_t *length)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t state;

    usb_dev_get_status(class->handle, kUSB_DeviceStatusDeviceState,
                          &state);

    if (state != kUSB_DeviceStateAddress && state != kUSB_DeviceStateConfigured) {
        return error;
    }

    if (is_recipient_device(setup->bmRequestType)) {
#if defined(USB_DEVICE_CONFIG_OTG)
        if (setup->wIndex == USB_REQ_STD_GET_STATUS_OTG_STATUS_SELECTOR) {
            error = usb_dev_get_status(class->handle, kUSB_DeviceStatusOtg,
                                       &class->std_transact_buf);
            class->std_transact_buf = USB_SHORT_TO_LITTLE_ENDIAN(class->std_transact_buf);
            *length = 1;
        } else {   /* Get the device status */
#endif
        error = usb_dev_get_status(class->handle,
                                   kUSB_DeviceStatusDevice,
                                   &class->std_transact_buf);
        class->std_transact_buf = class->std_transact_buf & USB_GET_STATUS_DEVICE_MASK;
        class->std_transact_buf = USB_SHORT_TO_LITTLE_ENDIAN(class->std_transact_buf);
        *length = USB_DEVICE_STATUS_SIZE;
#if defined(USB_DEVICE_CONFIG_OTG)
    }
#endif
    } else if (is_recipient_interface(setup->bmRequestType)) {
        error = kStatus_USB_Success;
        class->std_transact_buf = 0;
        *length = USB_INTERFACE_STATUS_SIZE;
    } else if (is_recipient_endpoint(setup->bmRequestType)) {
        usb_dev_ep_status_t ep_status;
        ep_status.addr = (uint8_t)setup->wIndex;
        ep_status.status = kUSB_DeviceEndpointStateIdle;
        error = usb_dev_get_status(class->handle,
                                   kUSB_DeviceStatusEndpoint,
                                   &ep_status);
        class->std_transact_buf = ep_status.status & USB_GET_STATUS_ENDPOINT_MASK;
        class->std_transact_buf = USB_SHORT_TO_LITTLE_ENDIAN(class->std_transact_buf);
        *length = USB_ENDPOINT_STATUS_SIZE;
    }
    *buffer = (uint8_t *)&class->std_transact_buf;

    return error;
}

static usb_status_t
_usb_device_set_clear_feature(usb_device_common_class_t *class,
                              usb_setup_struct_t *setup,
                              uint8_t **buffer,
                              uint32_t *length)
{
    usb_status_t err = kStatus_USB_InvalidRequest;
    uint8_t state;
    uint8_t isSet = 0;

    usb_dev_get_status(class->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if (state != kUSB_DeviceStateAddress && state != kUSB_DeviceStateConfigured) {
        return err;
    }

    if (setup->bRequest == USB_REQ_STD_SET_FEATURE) {
        isSet = 1;
    }

    if (is_recipient_device(setup->bmRequestType)) {
        /* Set or Clear the device featrue. */
        if (setup->wValue == USB_REQ_STD_FEATURE_SELECTOR_DEVICE_REMOTE_WAKEUP) {
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
            usb_dev_set_status(class->handle,
                               kUSB_DeviceStatusRemoteWakeup, &isSet);
#endif
            err = usb_device_class_cb(class->handle,
                                      kUSB_DeviceEventSetRemoteWakeup,
                                      &isSet);
        }

#if (defined(USB_DEVICE_CONFIG_OTG) && (USB_DEVICE_CONFIG_OTG))
        else if (setup->wValue == USB_REQ_STD_FEATURE_SELECTOR_B_HNP_ENABLE) {
            err = usb_device_class_cb(class->handle,
                                      kUSB_DeviceEventSetBHNPEnable,
                                      &isSet);
        }
#endif

    } else if (is_recipient_endpoint(setup->bmRequestType)) {
        if (setup->wValue == USB_REQ_STD_FEATURE_SELECTOR_ENDPOINT_HALT) {
            if (USB_CONTROL_ENDPOINT == USB_EP_NUMBER(setup->wIndex)) {
                if (isSet) {
                    usb_dev_ep_stall(class->handle,
                                        (uint8_t)setup->wIndex);
                } else {
                    usb_dev_ep_unstall(class->handle,
                                       (uint8_t)setup->wIndex);
                }
            }

            /* Set or Clear the endpoint status featrue. */
            if (isSet) {
                err = usb_device_class_event(class->handle,
                          kUSB_DeviceClassEventSetEndpointHalt,
                          &setup->wIndex);
            } else {
                err = usb_device_class_event(class->handle,
                          kUSB_DeviceClassEventClearEndpointHalt,
                          &setup->wIndex);
            }
        }
    }

    return err;
}

static usb_status_t
_usb_device_set_address(usb_device_common_class_t *classHandle,
                        usb_setup_struct_t *setup,
                        uint8_t * *buffer,
                        uint32_t *length)
{
    usb_status_t err = kStatus_USB_InvalidRequest;
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState, &state);

    switch (state) {
    case kUSB_DeviceStateAddressing: /* fallthrough */
    case kUSB_DeviceStateAddress:    /* fallthrough */
    case kUSB_DeviceStateDefault:    /* fallthrough */
    case kUSB_DeviceStateConfigured:
        break;
    default:
        return err;
    }

    if (state != kUSB_DeviceStateAddressing) {
        state = setup->wValue & 0xFF;
        err = usb_dev_set_status(classHandle->handle,
                                 kUSB_DeviceStatusAddress, &state);
    } else {
        err = usb_dev_set_status(classHandle->handle,
                                 kUSB_DeviceStatusAddress, NULL);

        if (!err) {
            state = kUSB_DeviceStateAddress;
            err = usb_dev_set_status(classHandle->handle,
                                     kUSB_DeviceStatusDeviceState, &state);
        }
    }

    return err;
}

static usb_status_t
_usb_device_get_descriptor(usb_device_common_class_t *classHandle,
                           usb_setup_struct_t *setup,
                           uint8_t **buffer,
                           uint32_t *length)
{
    usb_device_get_descriptor_common_union_t commonDescriptor;
    uint8_t state;
    uint8_t type = (uint8_t)((setup->wValue & 0xFF00) >> 8);
    uint8_t index = (uint8_t)(setup->wValue & 0x00FF);
    usb_status_t err = kStatus_USB_InvalidRequest;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState, &state);

    //printf("state=%d\n", state);
    switch (state) {
    case kUSB_DeviceStateAddress:
    case kUSB_DeviceStateConfigured:
    case kUSB_DeviceStateDefault:
        break;
    default:
        return err;
    }

    //printf("type=%d, index=%d\n", type, index);

    commonDescriptor.commonDescriptor.length = setup->wLength;
    switch (type) {
    case USB_DESCRIPTOR_TYPE_DEVICE:
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetDeviceDescriptor,
                                  &commonDescriptor.deviceDescriptor);
        break;
    case USB_DESCRIPTOR_TYPE_CONFIGURE:
        commonDescriptor.configurationDescriptor.configuration = index;
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetConfigurationDescriptor,
                                  &commonDescriptor.configurationDescriptor);
        break;
    case USB_DESCRIPTOR_TYPE_STRING:
        commonDescriptor.stringDescriptor.stringIndex = index;
        commonDescriptor.stringDescriptor.languageId = setup->wIndex;
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetStringDescriptor,
                                  &commonDescriptor.stringDescriptor);
        break;

#if MYNEWT_VAL(USB_DEVICE_CONFIG_HID)
    case USB_DESCRIPTOR_TYPE_HID:
        commonDescriptor.hidDescriptor.interfaceNumber = setup->wIndex;
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetHidDescriptor,
                                  &commonDescriptor.hidDescriptor);
        break;
    case USB_DESCRIPTOR_TYPE_HID_REPORT:
        commonDescriptor.hidReportDescriptor.interfaceNumber = setup->wIndex;
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetHidReportDescriptor,
                                  &commonDescriptor.hidReportDescriptor);
        break;
    case USB_DESCRIPTOR_TYPE_HID_PHYSICAL:
        commonDescriptor.hidPhysicalDescriptor.index = index;
        commonDescriptor.hidPhysicalDescriptor.interfaceNumber = setup->wIndex;
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetHidPhysicalDescriptor,
                                  &commonDescriptor.hidPhysicalDescriptor);
        break;
#endif

    default:
        break;
    }

    *buffer = commonDescriptor.commonDescriptor.buffer;
    *length = commonDescriptor.commonDescriptor.length;

    return err;
}

static usb_status_t
_usb_device_get_configuration(usb_device_common_class_t *classHandle,
                              usb_setup_struct_t *setup,
                              uint8_t **buffer,
                              uint32_t *length)
{
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState, &state);

    if (state != kUSB_DeviceStateAddress && state != kUSB_DeviceStateConfigured) {
        return kStatus_USB_InvalidRequest;
    }

    *length = USB_CONFIGURE_SIZE;
    *buffer = (uint8_t *)&classHandle->std_transact_buf;

    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventGetConfiguration,
                               &classHandle->std_transact_buf);
}

static usb_status_t
_usb_device_set_configuration(usb_device_common_class_t *classHandle,
                              usb_setup_struct_t *setup,
                              uint8_t * *buffer,
                              uint32_t *length)
{
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if (state != kUSB_DeviceStateAddress && state != kUSB_DeviceStateConfigured) {
        return kStatus_USB_InvalidRequest;
    }

    state = kUSB_DeviceStateConfigured;
    usb_dev_set_status(classHandle->handle, kUSB_DeviceStatusDeviceState, &state);
    if (!setup->wValue) {
        state = kUSB_DeviceStateAddress;
        usb_dev_set_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                           &state);
    }

    usb_device_class_event(classHandle->handle,
                           kUSB_DeviceClassEventSetConfiguration,
                           &setup->wValue);

    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventSetConfiguration,
                               &setup->wValue);
}

static usb_status_t
_usb_device_get_interface(usb_device_common_class_t *classHandle,
                          usb_setup_struct_t *setup,
                          uint8_t **buffer,
                          uint32_t *length)
{
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState, &state);

    if (state != kUSB_DeviceStateConfigured) {
        return kStatus_USB_InvalidRequest;
    }

    *length = USB_INTERFACE_SIZE;
    *buffer = (uint8_t *)&classHandle->std_transact_buf;
    classHandle->std_transact_buf = setup->wIndex & 0xFF;

    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventGetInterface,
                               &classHandle->std_transact_buf);
}

static usb_status_t
_usb_device_set_interface(usb_device_common_class_t *classHandle,
                          usb_setup_struct_t *setup,
                          uint8_t **buffer,
                          uint32_t *length)
{
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if (state != kUSB_DeviceStateConfigured) {
        return kStatus_USB_InvalidRequest;
    }
    classHandle->std_transact_buf = ((setup->wIndex & 0xFF) << 8) | (setup->wValue & 0xFF);

    usb_device_class_event(classHandle->handle,
                           kUSB_DeviceClassEventSetInterface,
                           &classHandle->std_transact_buf);

    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventSetInterface,
                               &classHandle->std_transact_buf);
}

static usb_status_t
_usb_device_synch_frame(usb_device_common_class_t *classHandle,
                        usb_setup_struct_t *setup,
                        uint8_t **buffer,
                        uint32_t *length)
{
    usb_status_t err = kStatus_USB_InvalidRequest;
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if (state != kUSB_DeviceStateConfigured) {
        return err;
    }

    classHandle->std_transact_buf = setup->wIndex;
    err = usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusSynchFrame,
                             &classHandle->std_transact_buf);

    *buffer = (uint8_t *)&classHandle->std_transact_buf;
    *length = sizeof(classHandle->std_transact_buf);

    return err;
}

/*
 * This function is used to send the reponse to the host.
 *
 * There are two cases this function will be called.
 * Case one when a setup packet is received in control endpoint callback function:
 *   1. If there is not data phase in the setup transfer, the function will prime
 *      an IN transfer with the data length is zero for status phase.
 *   2. If there is an IN data phase, the function will prime an OUT transfer with
 *      the actual length to need to send for data phase. And then prime an IN
 *      transfer with the data length is zero for status phase.
 *   3. If there is an OUT data phase, the function will prime an IN transfer with
 *      the actual length to want to receive for data phase.
 *
 * Case two when is not a setup packet received in control endpoint callback function:
 *   1. The function will prime an IN transfer with data length is zero for status
 *      phase.
 */
static usb_status_t
_usb_device_control_cb_feedback(usb_device_handle handle,
                                usb_setup_struct_t *setup,
                                usb_status_t err,
                                usb_device_control_read_write_sequence_t stage,
                                uint8_t * *buffer,
                                uint32_t *length)
{
    uint8_t dir = USB_REQ_TYPE_DIR_IN;

    if (err == kStatus_USB_InvalidRequest) {
        if (!is_req_type_std(setup->bmRequestType) &&
                is_req_type_out(setup->bmRequestType) &&
                setup->wLength && stage == kUSB_DeviceControlPipeSetupStage) {
            dir = USB_REQ_TYPE_DIR_OUT;
        }
        err = usb_dev_ep_stall(handle, USB_CONTROL_ENDPOINT | dir);
    } else {
        if (*length > setup->wLength) {
            *length = setup->wLength;
        }

        err = usb_device_send_req(handle, USB_CONTROL_ENDPOINT, *buffer, *length);
        if (!err && is_req_type_in(setup->bmRequestType)) {
            err = usb_device_recv_req(handle, USB_CONTROL_ENDPOINT, NULL, 0);
        }
    }
    return err;
}

/*
 * This callback function is used to notify uplayer the result of a transfer.
 * This callback pointer is passed when a specified endpoint initialied by calling
 * API usb_dev_ep_init.
 */
usb_status_t
_usb_device_control_cb(usb_device_handle handle,
                       usb_dev_ep_cb_msg_t *message,
                       void *callbackParam)
{
    usb_setup_struct_t *deviceSetup;
    usb_device_common_class_t *classHandle;
    uint8_t *buffer = (uint8_t *)NULL;
    uint32_t length = 0;
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t state;
    usb_device_control_request_struct_t req;

    //printf("_usb_device_control_cb\n");

    if (message->length == ((uint32_t) -1) || !callbackParam) {
        return error;
    }

    classHandle = (usb_device_common_class_t *)callbackParam;
    deviceSetup = (usb_setup_struct_t *)&classHandle->setupBuffer[0];
    usb_dev_get_status(handle, kUSB_DeviceStatusDeviceState, &state);

    if (message->isSetup) {
        if (message->length != USB_SETUP_PACKET_SIZE || !message->buffer) {
            /* If a invalid setup is received, the control pipes should be de-init and init again.
             * Due to the IP can not meet this require, it is revesed for feature.
             */
            /*
               usb_dev_ep_deinit(handle, USB_CONTROL_ENDPOINT | 0x80);
               usb_dev_ep_deinit(handle, USB_CONTROL_ENDPOINT);
               usb_device_control_pipe_init(handle, callbackParam);
             */
            return error;
        }

        /* Receive a setup request */
        usb_setup_struct_t *setup = (usb_setup_struct_t *)(message->buffer);

        /* Copy the setup packet to the application buffer */
        deviceSetup->wValue = USB_SHORT_FROM_LITTLE_ENDIAN(setup->wValue);
        deviceSetup->wIndex = USB_SHORT_FROM_LITTLE_ENDIAN(setup->wIndex);
        deviceSetup->wLength = USB_SHORT_FROM_LITTLE_ENDIAN(setup->wLength);
        deviceSetup->bRequest = setup->bRequest;
        deviceSetup->bmRequestType = setup->bmRequestType;

        printf("bRequest=%d, bmRequestType=0x%02x, wLength=%d\n", setup->bRequest, setup->bmRequestType, deviceSetup->wLength);

        if (is_req_type_std(deviceSetup->bmRequestType)) {
            if (_std_req_table[deviceSetup->bRequest]) {
                error = _std_req_table[deviceSetup->bRequest](
                    classHandle, deviceSetup, &buffer, &length);
            }
        } else {
            if (deviceSetup->wLength && is_req_type_out(deviceSetup->bmRequestType)) {
                /* Class or vendor request with the OUT data phase. */
                if (deviceSetup->wLength && is_req_type_class(deviceSetup->bmRequestType)) {
                    req.buffer = NULL;
                    req.isSetup = 1;
                    req.setup = deviceSetup;
                    req.length = deviceSetup->wLength;
                    error = usb_device_class_event(handle,
                                                   kUSB_DeviceClassEventClassRequest,
                                                   &req);
                    length = req.length;
                    buffer = req.buffer;
                } else if (deviceSetup->wLength && is_req_type_vendor(deviceSetup->bmRequestType)) {
                    req.buffer = (uint8_t *)NULL;
                    req.isSetup = 1;
                    req.setup = deviceSetup;
                    req.length = deviceSetup->wLength;
                    error = usb_device_class_cb(handle,
                                                kUSB_DeviceEventVendorRequest,
                                                &req);
                    length = req.length;
                    buffer = req.buffer;
                }
                if (error == kStatus_USB_Success) {
                    /* Prime an OUT transfer */
                    error = usb_device_recv_req(handle, USB_CONTROL_ENDPOINT,
                                                buffer,
                                                deviceSetup->wLength);
                    return error;
                }
            } else {
                /* Class or vendor request with the IN data phase. */
                if (is_req_type_class(deviceSetup->bmRequestType)) {
                    req.buffer = NULL;
                    req.isSetup = 1;
                    req.setup = deviceSetup;
                    req.length = deviceSetup->wLength;
                    error = usb_device_class_event(handle,
                                                   kUSB_DeviceClassEventClassRequest,
                                                   &req);
                    length = req.length;
                    buffer = req.buffer;
                } else if (is_req_type_vendor(deviceSetup->bmRequestType)) {
                    req.buffer = NULL;
                    req.isSetup = 1;
                    req.setup = deviceSetup;
                    req.length = deviceSetup->wLength;
                    error = usb_device_class_cb(handle,
                                                kUSB_DeviceEventVendorRequest,
                                                &req);
                    length = req.length;
                    buffer = req.buffer;
                }
            }
        }

        /* Send the reponse to the host. */
        error = _usb_device_control_cb_feedback(handle, deviceSetup, error,
                                                kUSB_DeviceControlPipeSetupStage, &buffer,
                                                &length);
    } else if (state == kUSB_DeviceStateAddressing) {
        /* Set the device address to controller. */
        error =
            _std_req_table[deviceSetup->bRequest](classHandle,
                                                  deviceSetup,
                                                  &buffer,
                                                  &length);
    } else if (message->length && deviceSetup->wLength && is_req_type_out(deviceSetup->bmRequestType)) {
        if (is_req_type_class(deviceSetup->bmRequestType)) {
            req.buffer = message->buffer;
            req.isSetup = 0;
            req.setup = deviceSetup;
            req.length = message->length;
            error = usb_device_class_event(handle,
                                           kUSB_DeviceClassEventClassRequest,
                                           &req);
        } else if (is_req_type_vendor(deviceSetup->bmRequestType)) {
            req.buffer = message->buffer;
            req.isSetup = 0;
            req.setup = deviceSetup;
            req.length = message->length;
            error = usb_device_class_cb(handle,
                                        kUSB_DeviceEventVendorRequest,
                                        &req);
        }
        /* Send the reponse to the host. */
        error = _usb_device_control_cb_feedback(handle, deviceSetup, error,
                                                kUSB_DeviceControlPipeDataStage, &buffer,
                                                &length);
    }
    return error;
}

usb_status_t
usb_device_control_pipe_init(usb_device_handle handle, void *param)
{
    usb_dev_ep_init_t epInitStruct;
    usb_dev_ep_cb_t ep_cb;
    usb_status_t err;

    ep_cb.fn = _usb_device_control_cb;
    ep_cb.param = param;

    epInitStruct.zlt = 1;
    epInitStruct.transferType = USB_ENDPOINT_CONTROL;
    epInitStruct.maxPacketSize = USB_CONTROL_MAX_PACKET_SIZE;

    epInitStruct.endpointAddress = USB_CONTROL_ENDPOINT | 0x80;

    err = usb_dev_ep_init(handle, &epInitStruct, &ep_cb);
    if (err != kStatus_USB_Success) {
        return err;
    }

    epInitStruct.endpointAddress = USB_CONTROL_ENDPOINT;

    err = usb_dev_ep_init(handle, &epInitStruct, &ep_cb);
    if (err != kStatus_USB_Success) {
        usb_dev_ep_deinit(handle, USB_CONTROL_ENDPOINT | 0x80);
        return err;
    }

    return kStatus_USB_Success;
}
#endif /* MYNEWT_VAL(USB_DEVICE_CONFIG_NUM) */
