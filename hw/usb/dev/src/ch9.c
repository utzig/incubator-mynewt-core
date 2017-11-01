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

typedef int (*usb_standard_request_callback_t)(
    usb_device_common_class_t *, usb_setup_t *, uint8_t **, uint32_t *);

#define DECLARE_STD_REQ_CB(name) static int name( \
    usb_device_common_class_t *, usb_setup_t *, uint8_t **, uint32_t *)

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

static int
_usb_device_get_status(usb_device_common_class_t *class, usb_setup_t *setup,
        uint8_t **buffer, uint32_t *length)
{
    uint8_t state;
    int err = USB_INVALID_REQ;

    usb_dev_get_status(class->handle, kUSB_DeviceStatusDeviceState,
                          &state);

    if (state != kUSB_DeviceStateAddress && state != kUSB_DeviceStateConfigured) {
        return err;
    }

    if (is_recipient_device(setup->bmRequestType)) {
#if defined(USB_DEVICE_CONFIG_OTG)
        if (setup->wIndex == USB_REQ_STD_GET_STATUS_OTG_STATUS_SELECTOR) {
            error = usb_dev_get_status(class->handle, kUSB_DeviceStatusOtg,
                                       &class->std_transact_buf);
            class->std_transact_buf = USB_U16_TO_LE(class->std_transact_buf);
            *length = 1;
        } else {
#endif
        err = usb_dev_get_status(class->handle, kUSB_DeviceStatusDevice,
                                 &class->std_transact_buf);
        class->std_transact_buf = class->std_transact_buf & USB_GET_STATUS_DEVICE_MASK;
        class->std_transact_buf = USB_U16_TO_LE(class->std_transact_buf);
        *length = USB_DEVICE_STATUS_SIZE;
#if defined(USB_DEVICE_CONFIG_OTG)
    }
#endif
    } else if (is_recipient_interface(setup->bmRequestType)) {
        err = 0;
        class->std_transact_buf = 0;
        *length = USB_INTERFACE_STATUS_SIZE;
    } else if (is_recipient_endpoint(setup->bmRequestType)) {
        usb_dev_ep_status_t ep_status;
        ep_status.addr = (uint8_t)setup->wIndex;
        ep_status.status = kUSB_DeviceEndpointStateIdle;
        err = usb_dev_get_status(class->handle, kUSB_DeviceStatusEndpoint,
                                 &ep_status);
        class->std_transact_buf = ep_status.status & USB_GET_STATUS_ENDPOINT_MASK;
        class->std_transact_buf = USB_U16_TO_LE(class->std_transact_buf);
        *length = USB_ENDPOINT_STATUS_SIZE;
    }
    *buffer = (uint8_t *)&class->std_transact_buf;

    return err;
}

static int
_usb_device_set_clear_feature(usb_device_common_class_t *class,
        usb_setup_t *setup, uint8_t **buffer, uint32_t *length)
{
    uint8_t state;
    uint8_t isSet = 0;
    int err = USB_INVALID_REQ;

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

static int
_usb_device_set_address(usb_device_common_class_t *classHandle,
        usb_setup_t *setup, uint8_t **buffer, uint32_t *length)
{
    uint8_t state;
    int err = USB_INVALID_REQ;

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
        state = setup->wValue & 0xff;
        err = usb_dev_set_status(classHandle->handle, kUSB_DeviceStatusAddress,
                &state);
    } else {
        err = usb_dev_set_status(classHandle->handle, kUSB_DeviceStatusAddress,
                NULL);

        if (!err) {
            state = kUSB_DeviceStateAddress;
            err = usb_dev_set_status(classHandle->handle,
                                     kUSB_DeviceStatusDeviceState, &state);
        }
    }

    return err;
}

static int
_usb_device_get_descriptor(usb_device_common_class_t *classHandle,
        usb_setup_t *setup, uint8_t **buf, uint32_t *len)
{
    usb_desc_t desc;
    uint8_t state;
    uint8_t type = setup->wValue >> 8;
    uint8_t index = setup->wValue & 0xff;
    int err = USB_INVALID_REQ;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState, &state);

    switch (state) {
    case kUSB_DeviceStateAddress:
    case kUSB_DeviceStateConfigured:
    case kUSB_DeviceStateDefault:
        break;
    default:
        return err;
    }

    desc.common_desc.len = setup->wLength;
    switch (type) {
    case DESC_TYPE_DEVICE:
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetDeviceDescriptor,
                                  &desc.device_desc);
        break;
    case DESC_TYPE_CONFIGURE:
        desc.configuration_desc.config = index;
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetConfigurationDescriptor,
                                  &desc.configuration_desc);
        break;
    case DESC_TYPE_STRING:
        desc.string_desc.str_idx = index;
        desc.string_desc.lang_id = setup->wIndex;
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetStringDescriptor,
                                  &desc.string_desc);
        break;

#if MYNEWT_VAL(USB_DEVICE_CONFIG_HID)
    case DESC_TYPE_HID:
        desc.hid_desc.itf_num = setup->wIndex;
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetHidDescriptor,
                                  &desc.hid_desc);
        break;
    case DESC_TYPE_HID_REPORT:
        desc.hid_report_desc.itf_num = setup->wIndex;
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetHidReportDescriptor,
                                  &desc.hid_report_desc);
        break;
    case DESC_TYPE_HID_PHYSICAL:
        desc.hid_physical_desc.idx = index;
        desc.hid_physical_desc.itf_num = setup->wIndex;
        err = usb_device_class_cb(classHandle->handle,
                                  kUSB_DeviceEventGetHidPhysicalDescriptor,
                                  &desc.hid_physical_desc);
        break;
#endif

    default:
        break;
    }

    *buf = desc.common_desc.buf;
    *len = desc.common_desc.len;

    return err;
}

static int
_usb_device_get_configuration(usb_device_common_class_t *classHandle,
        usb_setup_t *setup, uint8_t **buf, uint32_t *len)
{
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState, &state);

    if (state != kUSB_DeviceStateAddress && state != kUSB_DeviceStateConfigured) {
        return USB_INVALID_REQ;
    }

    *len = USB_CONFIGURE_SIZE;
    *buf = (uint8_t *)&classHandle->std_transact_buf;

    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventGetConfiguration,
                               &classHandle->std_transact_buf);
}

static int
_usb_device_set_configuration(usb_device_common_class_t *classHandle,
        usb_setup_t *setup, uint8_t **buffer, uint32_t *length)
{
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if (state != kUSB_DeviceStateAddress && state != kUSB_DeviceStateConfigured) {
        return USB_INVALID_REQ;
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

static int
_usb_device_get_interface(usb_device_common_class_t *classHandle,
        usb_setup_t *setup, uint8_t **buf, uint32_t *len)
{
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState, &state);

    if (state != kUSB_DeviceStateConfigured) {
        return USB_INVALID_REQ;
    }

    *len = USB_INTERFACE_SIZE;
    *buf = (uint8_t *)&classHandle->std_transact_buf;
    classHandle->std_transact_buf = setup->wIndex & 0xFF;

    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventGetInterface,
                               &classHandle->std_transact_buf);
}

static int
_usb_device_set_interface(usb_device_common_class_t *classHandle,
        usb_setup_t *setup, uint8_t **buffer, uint32_t *length)
{
    uint8_t state;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if (state != kUSB_DeviceStateConfigured) {
        return USB_INVALID_REQ;
    }
    classHandle->std_transact_buf = ((setup->wIndex & 0xff) << 8) | (setup->wValue & 0xff);

    usb_device_class_event(classHandle->handle,
                           kUSB_DeviceClassEventSetInterface,
                           &classHandle->std_transact_buf);

    return usb_device_class_cb(classHandle->handle,
                               kUSB_DeviceEventSetInterface,
                               &classHandle->std_transact_buf);
}

static int
_usb_device_synch_frame(usb_device_common_class_t *classHandle,
        usb_setup_t *setup, uint8_t **buf, uint32_t *len)
{
    uint8_t state;
    int err;

    usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusDeviceState,
                       &state);

    if (state != kUSB_DeviceStateConfigured) {
        return USB_INVALID_REQ;
    }

    classHandle->std_transact_buf = setup->wIndex;
    err = usb_dev_get_status(classHandle->handle, kUSB_DeviceStatusSynchFrame,
                             &classHandle->std_transact_buf);

    *buf = (uint8_t *)&classHandle->std_transact_buf;
    *len = sizeof(classHandle->std_transact_buf);

    return err;
}

/*
 * This function is used to send the response to the host.
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
static int
_usb_device_control_cb_feedback(usb_device_handle handle, usb_setup_t *setup,
        int err, usb_dev_ctrl_rw_seq_t stage, uint8_t **buf, uint32_t *len)
{
    uint8_t dir = USB_REQ_TYPE_DIR_IN;

    if (err == USB_INVALID_REQ) {
        if (!is_req_type_std(setup->bmRequestType) &&
                is_req_type_out(setup->bmRequestType) && setup->wLength &&
                stage == kUSB_DeviceControlPipeSetupStage) {
            dir = USB_REQ_TYPE_DIR_OUT;
        }
        err = usb_dev_ep_stall(handle, USB_CONTROL_ENDPOINT | dir);
    } else {
        if (*len > setup->wLength) {
            *len = setup->wLength;
        }

        /* stage == kUSB_DeviceControlPipeSetupStage is setup (not in) */

        err = usb_device_send_req(handle, USB_CONTROL_ENDPOINT, *buf, *len);
        //printf("err=%d, bmRequestType=0x%02x\n", err, setup->bmRequestType);
#if 1 //FIXME: temporary test for stm32
        if (!err && is_req_type_in(setup->bmRequestType)) {
            err = usb_device_recv_req(handle, USB_CONTROL_ENDPOINT, NULL, 0);
            //printf("recv_req, err=%d\n", err);
        }
#endif
    }
    return err;
}

/*
 * This callback function is used to notify uplayer the result of a transfer.
 * This callback pointer is passed when a specified endpoint initialied by calling
 * API usb_dev_ep_init.
 */
static int
_usb_device_control_cb(usb_device_handle handle, usb_dev_ep_cb_msg_t *msg, void *param)
{
    usb_setup_t *setup = NULL;
    usb_device_common_class_t *class = NULL;
    uint8_t *buf = NULL;
    uint32_t len = 0;
    uint8_t state;
    usb_dev_ctrl_req_t req;
    int err = USB_INVALID_REQ;

    if (msg->len == USB_UNINITIALIZED_VAL_32 || !param) {
        return err;
    }

    class = (usb_device_common_class_t *)param;
    setup = (usb_setup_t *)&class->setupBuffer[0];
    usb_dev_get_status(handle, kUSB_DeviceStatusDeviceState, &state);

    if (msg->setup) {
        if (msg->len != USB_SETUP_PACKET_SIZE || !msg->buf) {
            /* If a invalid setup is received, the control pipes should be de-init and init again.
             * Due to the IP can not meet this require, it is revesed for feature.
             */
            /*
               usb_dev_ep_deinit(handle, USB_CONTROL_ENDPOINT | 0x80);
               usb_dev_ep_deinit(handle, USB_CONTROL_ENDPOINT);
               usb_device_control_pipe_init(handle, param);
             */
            return err;
        }

        /* Receive a setup request */
        usb_setup_t *st = (usb_setup_t *)msg->buf;
        setup->wValue = USB_U16_FROM_LE(st->wValue);
        setup->wIndex = USB_U16_FROM_LE(st->wIndex);
        setup->wLength = USB_U16_FROM_LE(st->wLength);
        setup->bRequest = st->bRequest;
        setup->bmRequestType = st->bmRequestType;

        //printf("bmRequestType=%02x, bRequest=%02x\n", setup->bmRequestType, setup->bRequest);
        if (is_req_type_std(setup->bmRequestType)) {
            if (_std_req_table[setup->bRequest]) {
                //printf("bRequest=%02x, wValue=%04x\n", setup->bRequest, setup->wValue);
                err = _std_req_table[setup->bRequest](class, setup, &buf, &len);
            }
        } else {
            if (setup->wLength && is_req_type_out(setup->bmRequestType)) {
                /* Class or vendor request with the OUT data phase. */
                if (setup->wLength && is_req_type_class(setup->bmRequestType)) {
                    req.buf = NULL;
                    req.is_setup = 1;
                    req.setup = setup;
                    req.len = setup->wLength;
                    err = usb_device_class_event(handle,
                            kUSB_DeviceClassEventClassRequest, &req);
                    len = req.len;
                    buf = req.buf;
                } else if (setup->wLength && is_req_type_vendor(setup->bmRequestType)) {
                    req.buf = NULL;
                    req.is_setup = 1;
                    req.setup = setup;
                    req.len = setup->wLength;
                    err = usb_device_class_cb(handle,
                            kUSB_DeviceEventVendorRequest, &req);
                    len = req.len;
                    buf = req.buf;
                }
                if (!err) {
                    /* Prime an OUT transfer */
                    err = usb_device_recv_req(handle, USB_CONTROL_ENDPOINT, buf,
                            setup->wLength);
                    return err;
                }
            } else {
                /* Class or vendor request with the IN data phase. */
                if (is_req_type_class(setup->bmRequestType)) {
                    req.buf = NULL;
                    req.is_setup = 1;
                    req.setup = setup;
                    req.len = setup->wLength;
                    err = usb_device_class_event(handle,
                            kUSB_DeviceClassEventClassRequest, &req);
                    len = req.len;
                    buf = req.buf;
                } else if (is_req_type_vendor(setup->bmRequestType)) {
                    req.buf = NULL;
                    req.is_setup = 1;
                    req.setup = setup;
                    req.len = setup->wLength;
                    err = usb_device_class_cb(handle,
                            kUSB_DeviceEventVendorRequest, &req);
                    len = req.len;
                    buf = req.buf;
                }
            }
        }

        /* TODO: utzig - this is where setup stage is handled! */
        /* Send the response to the host. */
        err = _usb_device_control_cb_feedback(handle, setup, err,
                kUSB_DeviceControlPipeSetupStage, &buf, &len);
    //} else if (msg->is_in) {
        /* TODO: utzig - this is where stage in is handled! */
    //    err = usb_device_recv_req(handle, USB_CONTROL_ENDPOINT, NULL, 0);
    } else if (state == kUSB_DeviceStateAddressing) {
        err = _std_req_table[setup->bRequest](class, setup, &buf, &len);
    //} else if (is_req_type_in(setup->bmRequestType)) {
    } else if (msg->len && setup->wLength && is_req_type_out(setup->bmRequestType)) {
        //printf("out: bmRequestType=0x%02x\n", setup->bmRequestType);
        if (is_req_type_class(setup->bmRequestType)) {
            req.buf = msg->buf;
            req.is_setup = 0;
            req.setup = setup;
            req.len = msg->len;
            err = usb_device_class_event(handle, kUSB_DeviceClassEventClassRequest, &req);
        } else if (is_req_type_vendor(setup->bmRequestType)) {
            req.buf = msg->buf;
            req.is_setup = 0;
            req.setup = setup;
            req.len = msg->len;
            err = usb_device_class_cb(handle, kUSB_DeviceEventVendorRequest, &req);
        }
        /* Send the reponse to the host. */
        err = _usb_device_control_cb_feedback(handle, setup, err,
                kUSB_DeviceControlPipeDataStage, &buf, &len);
    }
    return err;
}

int
usb_device_control_pipe_init(usb_device_handle handle, void *param)
{
    usb_dev_ep_init_t ep_init;
    usb_dev_ep_cb_t ep_cb;
    int err;

    ep_cb.fn = _usb_device_control_cb;
    ep_cb.param = param;

    ep_init.zlt = 1;
    ep_init.transferType = USB_ENDPOINT_CONTROL;
    ep_init.maxPacketSize = USB_CONTROL_MAX_PACKET_SIZE;
    ep_init.endpointAddress = 0x80;

    err = usb_dev_ep_init(handle, &ep_init, &ep_cb);
    if (err) {
        return err;
    }

    ep_init.zlt = 1;
    ep_init.transferType = USB_ENDPOINT_CONTROL;
    ep_init.maxPacketSize = USB_CONTROL_MAX_PACKET_SIZE;
    ep_init.endpointAddress = 0;

    err = usb_dev_ep_init(handle, &ep_init, &ep_cb);
    if (err) {
        usb_dev_ep_deinit(handle, 0x80);
        return err;
    }

    return 0;
}
#endif /* MYNEWT_VAL(USB_DEVICE_CONFIG_NUM) */
