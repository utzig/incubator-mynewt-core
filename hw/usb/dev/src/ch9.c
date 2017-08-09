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

#if MYNEWT_VAL(USB_DEVICE_CONFIG_NUM)

typedef usb_status_t (*usb_standard_request_callback_t)(
    usb_device_common_class_t *class,
    usb_setup_struct_t *setup,
    uint8_t * *buffer,
    uint32_t *length);

#if 1
#define DECLARE_STD_REQ_CB(name)  static usb_status_t name(       \
    usb_device_common_class_t *, usb_setup_struct_t *,            \
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
#endif

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

/*!
 * This function is used to handle get status request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 */
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

    if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) ==
        USB_REQUEST_TYPE_RECIPIENT_DEVICE) {
#if (defined(USB_DEVICE_CONFIG_OTG) && (USB_DEVICE_CONFIG_OTG))
        if (setup->wIndex ==
            USB_REQUEST_STANDARD_GET_STATUS_OTG_STATUS_SELECTOR) {
            error =
                usb_dev_get_status(class->handle, kUSB_DeviceStatusOtg,
                                   &class->standardTranscationBuffer);
            class->standardTranscationBuffer =
                USB_SHORT_TO_LITTLE_ENDIAN(
                    class->standardTranscationBuffer);
            /* The device status length must be USB_DEVICE_STATUS_SIZE. */
            *length = 1;
        } else {   /* Get the device status */
#endif
        error = usb_dev_get_status(class->handle,
                                   kUSB_DeviceStatusDevice,
                                   &class->standardTranscationBuffer);
        class->standardTranscationBuffer =
            class->standardTranscationBuffer &
            USB_GET_STATUS_DEVICE_MASK;
        class->standardTranscationBuffer = USB_SHORT_TO_LITTLE_ENDIAN(
            class->standardTranscationBuffer);
        /* The device status length must be USB_DEVICE_STATUS_SIZE. */
        *length = USB_DEVICE_STATUS_SIZE;
#if (defined(USB_DEVICE_CONFIG_OTG) && (USB_DEVICE_CONFIG_OTG))
    }
#endif
    } else if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) ==
               USB_REQUEST_TYPE_RECIPIENT_INTERFACE) {
        /* Get the interface status */
        error = kStatus_USB_Success;
        class->standardTranscationBuffer = 0;
        /* The interface status length must be USB_INTERFACE_STATUS_SIZE. */
        *length = USB_INTERFACE_STATUS_SIZE;
    } else if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) ==
               USB_REQUEST_TYPE_RECIPIENT_ENDPOINT) {
        /* Get the endpoint status */
        usb_device_endpoint_status_struct_t endpointStatus;
        endpointStatus.endpointAddress = (uint8_t)setup->wIndex;
        endpointStatus.endpointStatus = kUSB_DeviceEndpointStateIdle;
        error = usb_dev_get_status(class->handle,
                                   kUSB_DeviceStatusEndpoint,
                                   &endpointStatus);
        class->standardTranscationBuffer =
            endpointStatus.endpointStatus & USB_GET_STATUS_ENDPOINT_MASK;
        class->standardTranscationBuffer = USB_SHORT_TO_LITTLE_ENDIAN(
            class->standardTranscationBuffer);
        /* The endpoint status length must be USB_INTERFACE_STATUS_SIZE. */
        *length = USB_ENDPOINT_STATUS_SIZE;
    } else {
    }
    *buffer = (uint8_t *)&class->standardTranscationBuffer;

    return error;
}

/*!
 * @brief Handle set or clear device feature request.
 *
 * This function is used to handle set or clear device feature request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @retval kStatus_USB_Success              The requst is handled successfully.
 * @retval kStatus_USB_InvalidRequest       The request can not be handle in current device state,
 *                                          or, the request is unsupported.
 */
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

    if (setup->bRequest == USB_REQUEST_STANDARD_SET_FEATURE) {
        isSet = 1;
    }

    if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) ==
        USB_REQUEST_TYPE_RECIPIENT_DEVICE) {
        /* Set or Clear the device featrue. */
        if (USB_REQUEST_STANDARD_FEATURE_SELECTOR_DEVICE_REMOTE_WAKEUP ==
            setup->wValue) {
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
            usb_dev_set_status(class->handle,
                               kUSB_DeviceStatusRemoteWakeup, &isSet);
#endif
            /* Set or Clear the device remote wakeup featrue. */
            err = usb_device_class_cb(class->handle,
                                      kUSB_DeviceEventSetRemoteWakeup,
                                      &isSet);
        }

#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) && \
        (defined(USB_DEVICE_CONFIG_EHCI_TEST_MODE) && \
        (USB_DEVICE_CONFIG_EHCI_TEST_MODE > 0U))
        else if (USB_REQUEST_STANDARD_FEATURE_SELECTOR_DEVICE_TEST_MODE ==
                 setup->wValue) {
            state = kUSB_DeviceStateTestMode;
            err = usb_dev_set_status(class->handle,
                                     kUSB_DeviceStatusDeviceState, &state);
        }
#endif

#if (defined(USB_DEVICE_CONFIG_OTG) && (USB_DEVICE_CONFIG_OTG))
        else if (USB_REQUEST_STANDARD_FEATURE_SELECTOR_B_HNP_ENABLE ==
                 setup->wValue) {
            err = usb_device_class_cb(class->handle,
                                      kUSB_DeviceEventSetBHNPEnable,
                                      &isSet);
        }
#endif

    } else if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) ==
               USB_REQUEST_TYPE_RECIPIENT_ENDPOINT) {
        /* Set or Clear the endpoint featrue. */
        if (USB_REQUEST_STANDARD_FEATURE_SELECTOR_ENDPOINT_HALT ==
            setup->wValue) {
            if (USB_CONTROL_ENDPOINT == USB_EP_NUMBER(setup->wIndex)) {
                /* Set or Clear the control endpoint status(halt or not). */
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

/*!
 * @brief Handle set address request.
 *
 * This function is used to handle set address request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @retval kStatus_USB_Success              The requst is handled successfully.
 * @retval kStatus_USB_InvalidRequest       The request can not be handle in current device state.
 */
static usb_status_t
_usb_device_set_address(usb_device_common_class_t *classHandle,
                        usb_setup_struct_t *setup,
                        uint8_t * *buffer,
                        uint32_t *length)
{
    usb_status_t err = kStatus_USB_InvalidRequest;
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if (state != kUSB_DeviceStateAddressing &&
        state != kUSB_DeviceStateAddress &&
        state != kUSB_DeviceStateDefault &&
        state != kUSB_DeviceStateConfigured) {
        return err;
    }

    if (kUSB_DeviceStateAddressing != state) {
        /* If the device address is not setting, pass the address and the device state will change to
         * kUSB_DeviceStateAddressing internally. */
        state = setup->wValue & 0xFFU;
        err = usb_dev_set_status(classHandle->handle,
                                 kUSB_DeviceStatusAddress, &state);
    } else {
        /* If the device address is setting, set device address and the address will be write into the controller
         * internally. */
        err = usb_dev_set_status(classHandle->handle,
                                 kUSB_DeviceStatusAddress, NULL);
        /* And then change the device state to kUSB_DeviceStateAddress. */
        if (err == kStatus_USB_Success) {
            state = kUSB_DeviceStateAddress;
            err = usb_dev_set_status(classHandle->handle,
                                     kUSB_DeviceStatusDeviceState, &state);
        }
    }

    return err;
}

/*!
 * @brief Handle get descriptor request.
 *
 * This function is used to handle get descriptor request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @retval kStatus_USB_Success              The requst is handled successfully.
 * @retval kStatus_USB_InvalidRequest       The request can not be handle in current device state,
 *                                          or, the request is unsupported.
 */
static usb_status_t
_usb_device_get_descriptor(usb_device_common_class_t *classHandle,
                           usb_setup_struct_t *setup,
                           uint8_t **buffer,
                           uint32_t *length)
{
    usb_device_get_descriptor_common_union_t commonDescriptor;
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t state;
    uint8_t descriptorType = (uint8_t)((setup->wValue & 0xFF00U) >> 8U);
    uint8_t descriptorIndex = (uint8_t)((setup->wValue & 0x00FFU));

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if ((kUSB_DeviceStateAddress != state) &&
        (kUSB_DeviceStateConfigured != state) &&
        (kUSB_DeviceStateDefault != state)) {
        return error;
    }
    commonDescriptor.commonDescriptor.length = setup->wLength;
    if (USB_DESCRIPTOR_TYPE_DEVICE == descriptorType) {
        /* Get the device descriptor */
        error = usb_device_class_cb(classHandle->handle,
                                    kUSB_DeviceEventGetDeviceDescriptor,
                                    &commonDescriptor.deviceDescriptor);
    } else if (USB_DESCRIPTOR_TYPE_CONFIGURE == descriptorType) {
        /* Get the configuration descriptor */
        commonDescriptor.configurationDescriptor.configuration =
            descriptorIndex;
        error = usb_device_class_cb(classHandle->handle,
                                    kUSB_DeviceEventGetConfigurationDescriptor,
                                    &commonDescriptor.configurationDescriptor);
    } else if (USB_DESCRIPTOR_TYPE_STRING == descriptorType) {
        /* Get the string descriptor */
        commonDescriptor.stringDescriptor.stringIndex = descriptorIndex;
        commonDescriptor.stringDescriptor.languageId = setup->wIndex;
        error = usb_device_class_cb(classHandle->handle,
                                    kUSB_DeviceEventGetStringDescriptor,
                                    &commonDescriptor.stringDescriptor);
    }
#if (defined(USB_DEVICE_CONFIG_HID) && (USB_DEVICE_CONFIG_HID > 0U))
    else if (USB_DESCRIPTOR_TYPE_HID == descriptorType) {
        /* Get the hid descriptor */
        commonDescriptor.hidDescriptor.interfaceNumber = setup->wIndex;
        error = usb_device_class_cb(classHandle->handle,
                                    kUSB_DeviceEventGetHidDescriptor,
                                    &commonDescriptor.hidDescriptor);
    } else if (USB_DESCRIPTOR_TYPE_HID_REPORT == descriptorType) {
        /* Get the hid report descriptor */
        commonDescriptor.hidReportDescriptor.interfaceNumber = setup->wIndex;
        error = usb_device_class_cb(classHandle->handle,
                                    kUSB_DeviceEventGetHidReportDescriptor,
                                    &commonDescriptor.hidReportDescriptor);
    } else if (USB_DESCRIPTOR_TYPE_HID_PHYSICAL == descriptorType) {
        /* Get the hid physical descriptor */
        commonDescriptor.hidPhysicalDescriptor.index = descriptorIndex;
        commonDescriptor.hidPhysicalDescriptor.interfaceNumber = setup->wIndex;
        error = usb_device_class_cb(classHandle->handle,
                                    kUSB_DeviceEventGetHidPhysicalDescriptor,
                                    &commonDescriptor.hidPhysicalDescriptor);
    }
#endif
#if (defined(USB_DEVICE_CONFIG_EHCI_TEST_MODE) && \
    (USB_DEVICE_CONFIG_EHCI_TEST_MODE > 0U))
    else if (USB_DESCRIPTOR_TYPE_DEVICE_QUALITIER == descriptorType) {
        /* Get the device descriptor */
        error = usb_device_class_cb(classHandle->handle,
                                    kUSB_DeviceEventGetDeviceQualifierDescriptor,
                                    &commonDescriptor.deviceDescriptor);
    }
#endif
    else {
    }
    *buffer = commonDescriptor.commonDescriptor.buffer;
    *length = commonDescriptor.commonDescriptor.length;
    return error;
}

static usb_status_t
_usb_device_get_configuration(usb_device_common_class_t *classHandle,
                              usb_setup_struct_t *setup,
                              uint8_t **buffer,
                              uint32_t *length)
{
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if ((kUSB_DeviceStateAddress != state) &&
        ((kUSB_DeviceStateConfigured != state))) {
        return kStatus_USB_InvalidRequest;
    }

    *length = USB_CONFIGURE_SIZE;
    *buffer = (uint8_t *)&classHandle->standardTranscationBuffer;
    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventGetConfiguration,
                               &classHandle->standardTranscationBuffer);
}

/*!
 * @brief Handle set current configuration request.
 *
 * This function is used to handle set current configuration request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @retval kStatus_USB_Success              The requst is handled successfully.
 * @retval kStatus_USB_InvalidRequest       The request can not be handle in current device state,
 *                                          or, the request is unsupported.
 */
static usb_status_t
_usb_device_set_configuration(usb_device_common_class_t *classHandle,
                              usb_setup_struct_t *setup,
                              uint8_t * *buffer,
                              uint32_t *length)
{
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if ((kUSB_DeviceStateAddress != state) &&
        (kUSB_DeviceStateConfigured != state)) {
        return kStatus_USB_InvalidRequest;
    }

    /* The device state is changed to kUSB_DeviceStateConfigured */
    state = kUSB_DeviceStateConfigured;
    usb_dev_set_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);
    if (!setup->wValue) {
        /* If the new configuration is zero, the device state is changed to kUSB_DeviceStateAddress */
        state = kUSB_DeviceStateAddress;
        usb_dev_set_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                           &state);
    }

    /* Notify the class layer the configuration is changed */
    usb_device_class_event(classHandle->handle,
                           kUSB_DeviceClassEventSetConfiguration,
                           &setup->wValue);
    /* Notify the application the configuration is changed */
    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventSetConfiguration,
                               &setup->wValue);
}

/*!
 * @brief Handle get the alternate setting of a interface request.
 *
 * This function is used to handle get the alternate setting of a interface request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @retval kStatus_USB_Success              The requst is handled successfully.
 * @retval kStatus_USB_InvalidRequest       The request can not be handle in current device state,
 *                                          or, the request is unsupported.
 */
static usb_status_t
_usb_device_get_interface(usb_device_common_class_t *classHandle,
                          usb_setup_struct_t *setup,
                          uint8_t **buffer,
                          uint32_t *length)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if (state != kUSB_DeviceStateConfigured) {
        return error;
    }
    *length = USB_INTERFACE_SIZE;
    *buffer = (uint8_t *)&classHandle->standardTranscationBuffer;
    classHandle->standardTranscationBuffer = setup->wIndex & 0xFFU;
    /* The Bit[15~8] is used to save the interface index, and the alternate setting will be saved in Bit[7~0] by
     * application. */
    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventGetInterface,
                               &classHandle->standardTranscationBuffer);
}

/*!
 * @brief Handle set the alternate setting of a interface request.
 *
 * This function is used to handle set the alternate setting of a interface request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @retval kStatus_USB_Success              The requst is handled successfully.
 * @retval kStatus_USB_InvalidRequest       The request can not be handle in current device state,
 *                                          or, the request is unsupported.
 */
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
    classHandle->standardTranscationBuffer = ((setup->wIndex & 0xFFU) << 8U) |
                                             (setup->wValue & 0xFFU);
    /* Notify the class driver the alternate setting of the interface is changed. */
    /* The Bit[15~8] is used to save the interface index, and the alternate setting is saved in Bit[7~0]. */
    usb_device_class_event(classHandle->handle,
                           kUSB_DeviceClassEventSetInterface,
                           &classHandle->standardTranscationBuffer);
    /* Notify the application the alternate setting of the interface is changed. */
    /* The Bit[15~8] is used to save the interface index, and the alternate setting will is saved in Bit[7~0]. */
    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventSetInterface,
                               &classHandle->standardTranscationBuffer);
}

/*!
 * @brief Handle get sync frame request.
 *
 * This function is used to handle get sync frame request.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @retval kStatus_USB_Success              The requst is handled successfully.
 * @retval kStatus_USB_InvalidRequest       The request can not be handle in current device state,
 *                                          or, the request is unsupported.
 */
static usb_status_t
_usb_device_synch_frame(usb_device_common_class_t *classHandle,
                        usb_setup_struct_t *setup,
                        uint8_t **buffer,
                        uint32_t *length)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if (state != kUSB_DeviceStateConfigured) {
        return error;
    }

    classHandle->standardTranscationBuffer = setup->wIndex;
    /* Get the sync frame value */
    error =
        usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusSynchFrame,
                           &classHandle->standardTranscationBuffer);
    *buffer = (uint8_t *)&classHandle->standardTranscationBuffer;
    *length = sizeof(classHandle->standardTranscationBuffer);

    return error;
}

/*!
 * @brief Send the reponse to the host.
 *
 * This function is used to send the reponse to the host.
 *
 * There are two cases this function will be called.
 * Case one when a setup packet is received in control endpoint callback function:
 *        1. If there is not data phase in the setup transfer, the function will prime an IN transfer with the data
 * length is zero for status phase.
 *        2. If there is an IN data phase, the function will prime an OUT transfer with the actual length to need to
 * send for data phase. And then prime an IN transfer with the data length is zero for status phase.
 *        3. If there is an OUT data phase, the function will prime an IN transfer with the actual length to want to
 * receive for data phase.
 *
 * Case two when is not a setup packet received in control endpoint callback function:
 *        1. The function will prime an IN transfer with data length is zero for status phase.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param setup           The pointer of the setup packet.
 * @param error           The error code returned from the standard request fucntion.
 * @param stage           The stage of the control transfer.
 * @param buffer          It is an out parameter, is used to save the buffer address to response the host's request.
 * @param length          It is an out parameter, the data length.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t
_usb_device_control_cb_feedback(usb_device_handle handle,
                                usb_setup_struct_t *setup,
                                usb_status_t error,
                                usb_device_control_read_write_sequence_t stage,
                                uint8_t * *buffer,
                                uint32_t *length)
{
    usb_status_t errorCode = kStatus_USB_Error;
    uint8_t direction = USB_IN;

    if (kStatus_USB_InvalidRequest == error) {
        /* Stall the control pipe when the request is unsupported. */
        if ((!((setup->bmRequestType & USB_REQUEST_TYPE_TYPE_MASK) ==
               USB_REQUEST_TYPE_TYPE_STANDARD)) &&
            ((setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) ==
             USB_REQUEST_TYPE_DIR_OUT) && (setup->wLength) &&
            (kUSB_DeviceControlPipeSetupStage == stage)) {
            direction = USB_OUT;
        }
        errorCode = usb_dev_ep_stall(handle,
            (USB_CONTROL_ENDPOINT) |
            (uint8_t)((uint32_t)direction <<
                USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
    } else {
        if (*length > setup->wLength) {
            *length = setup->wLength;
        }
        errorCode = usb_device_send_req(handle, (USB_CONTROL_ENDPOINT),
                                        *buffer, *length);

        if ((kStatus_USB_Success == errorCode) &&
            (USB_REQUEST_TYPE_DIR_IN ==
             (setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK))) {
            errorCode =
                usb_device_recv_req(handle, (USB_CONTROL_ENDPOINT), NULL, 0);
        }
    }
    return errorCode;
}

/*!
 * @brief Control endpoint callback function.
 *
 * This callback function is used to notify uplayer the tranfser result of a transfer.
 * This callback pointer is passed when a specified endpoint initialied by calling API usb_dev_ep_init.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param message         The result of a transfer, includes transfer buffer, transfer length and whether is in setup
 * phase for control pipe.
 * @param callbackParam  The paramter for this callback. It is same with
 * usb_device_endpoint_callback_struct_t::callbackParam.
 *
 * @return A USB error code or kStatus_USB_Success.
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
               usb_dev_ep_deinit(handle,
                         USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
               usb_dev_ep_deinit(handle,
                         USB_CONTROL_ENDPOINT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
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

        if ((deviceSetup->bmRequestType & USB_REQUEST_TYPE_TYPE_MASK) ==
            USB_REQUEST_TYPE_TYPE_STANDARD) {
            if (_std_req_table[deviceSetup->bRequest]) {
                error = _std_req_table[deviceSetup->bRequest](
                    classHandle, deviceSetup, &buffer, &length);
            }
        } else {
            if (deviceSetup->wLength &&
                ((deviceSetup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) ==
                 USB_REQUEST_TYPE_DIR_OUT)) {
                /* Class or vendor request with the OUT data phase. */
                if ((deviceSetup->wLength) &&
                    ((deviceSetup->bmRequestType &
                      USB_REQUEST_TYPE_TYPE_CLASS) ==
                     USB_REQUEST_TYPE_TYPE_CLASS)) {
                    req.buffer = (uint8_t *)NULL;
                    req.isSetup = 1;
                    req.setup = deviceSetup;
                    req.length = deviceSetup->wLength;
                    error = usb_device_class_event(handle,
                                                   kUSB_DeviceClassEventClassRequest,
                                                   &req);
                    length = req.length;
                    buffer = req.buffer;
                } else if (deviceSetup->wLength &&
                           (deviceSetup->bmRequestType &
                            USB_REQUEST_TYPE_TYPE_VENDOR) ==
                           USB_REQUEST_TYPE_TYPE_VENDOR) {
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
                if (((deviceSetup->bmRequestType &
                      USB_REQUEST_TYPE_TYPE_CLASS) ==
                     USB_REQUEST_TYPE_TYPE_CLASS)) {
                    req.buffer = NULL;
                    req.isSetup = 1;
                    req.setup = deviceSetup;
                    req.length = deviceSetup->wLength;
                    error = usb_device_class_event(handle,
                                                   kUSB_DeviceClassEventClassRequest,
                                                   &req);
                    length = req.length;
                    buffer = req.buffer;
                } else if (((deviceSetup->bmRequestType &
                             USB_REQUEST_TYPE_TYPE_VENDOR) ==
                            USB_REQUEST_TYPE_TYPE_VENDOR)) {
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
    } else if (kUSB_DeviceStateAddressing == state) {
        /* Set the device address to controller. */
        error =
            _std_req_table[deviceSetup->bRequest](classHandle,
                                                  deviceSetup,
                                                  &buffer,
                                                  &length);
    }
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) && \
    (defined(USB_DEVICE_CONFIG_EHCI_TEST_MODE) && \
    (USB_DEVICE_CONFIG_EHCI_TEST_MODE > 0U))
    else if (kUSB_DeviceStateTestMode == state) {
        uint8_t portTestControl = (uint8_t)(deviceSetup->wIndex >> 8);
        /* Set the controller.into test mode. */
        error = usb_dev_set_status(handle, kUSB_DeviceStatusTestMode,
                                   &portTestControl);
    }
#endif
    else if ((message->length) && (deviceSetup->wLength) &&
             ((deviceSetup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) ==
              USB_REQUEST_TYPE_DIR_OUT)) {
        if (((deviceSetup->bmRequestType & USB_REQUEST_TYPE_TYPE_CLASS) ==
             USB_REQUEST_TYPE_TYPE_CLASS)) {
            req.buffer = message->buffer;
            req.isSetup = 0;
            req.setup = deviceSetup;
            req.length = message->length;
            error = usb_device_class_event(handle,
                                           kUSB_DeviceClassEventClassRequest,
                                           &req);
        } else if (((deviceSetup->bmRequestType &
                     USB_REQUEST_TYPE_TYPE_VENDOR) ==
                    USB_REQUEST_TYPE_TYPE_VENDOR)) {
            req.buffer = message->buffer;
            req.isSetup = 0;
            req.setup = deviceSetup;
            req.length = message->length;
            error = usb_device_class_cb(handle,
                                        kUSB_DeviceEventVendorRequest,
                                        &req);
        } else {
        }
        /* Send the reponse to the host. */
        error = _usb_device_control_cb_feedback(handle, deviceSetup, error,
                                                kUSB_DeviceControlPipeDataStage, &buffer,
                                                &length);
    }
    return error;
}

/*!
 * @brief Control endpoint initialization function.
 *
 * This callback function is used to initialize the control pipes.
 *
 * @param handle          The device handle. It equals the value returned from USB_DeviceInit.
 * @param param           The up layer handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t
usb_device_control_pipe_init(usb_device_handle handle, void *param)
{
    usb_device_endpoint_init_struct_t epInitStruct;
    usb_device_endpoint_callback_struct_t endpointCallback;
    usb_status_t err;

    endpointCallback.callbackFn = _usb_device_control_cb;
    endpointCallback.callbackParam = param;

    epInitStruct.zlt = 1;
    epInitStruct.transferType = USB_ENDPOINT_CONTROL;
    epInitStruct.endpointAddress = USB_CONTROL_ENDPOINT |
        (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
    epInitStruct.maxPacketSize = USB_CONTROL_MAX_PACKET_SIZE;

    /* Initialize the control IN pipe */
    err = usb_dev_ep_init(handle, &epInitStruct, &endpointCallback);

    if (err != kStatus_USB_Success) {
        return err;
    }

    epInitStruct.endpointAddress = USB_CONTROL_ENDPOINT |
        (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);

    /* Initialize the control OUT pipe */
    err = usb_dev_ep_init(handle, &epInitStruct, &endpointCallback);

    if (err != kStatus_USB_Success) {
        usb_dev_ep_deinit(handle, USB_CONTROL_ENDPOINT |
            (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
        return err;
    }

    return kStatus_USB_Success;
}
#endif /* MYNEWT_VAL(USB_DEVICE_CONFIG_NUM) */
