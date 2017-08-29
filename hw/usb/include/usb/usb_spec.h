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

#ifndef __USB_SPEC_H__
#define __USB_SPEC_H__

#include <stdint.h>

#define USB_SPEED_FULL                                                  0x00
#define USB_SPEED_LOW                                                   0x01
#define USB_SPEED_HIGH                                                  0x02

typedef struct
{
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_struct_t;

#define USB_ENDPOINT_CONTROL                                             0x00
#define USB_ENDPOINT_ISOCHRONOUS                                         0x01
#define USB_ENDPOINT_BULK                                                0x02
#define USB_ENDPOINT_INTERRUPT                                           0x03

#define USB_OUT                                                          0
#define USB_IN                                                           1

/* USB standard descriptor length */
#define USB_DESCRIPTOR_LENGTH_DEVICE                                     0x12
#define USB_DESCRIPTOR_LENGTH_CONFIGURE                                  0x09
#define USB_DESCRIPTOR_LENGTH_INTERFACE                                  0x09
#define USB_DESCRIPTOR_LENGTH_ENDPOINT                                   0x07
#define USB_DESCRIPTOR_LENGTH_DEVICE_QUALITIER                           0x0A
#define USB_DESCRIPTOR_LENGTH_OTG_DESCRIPTOR                             5

/* USB standard descriptor type */
#define USB_DESCRIPTOR_TYPE_DEVICE                                       0x01
#define USB_DESCRIPTOR_TYPE_CONFIGURE                                    0x02
#define USB_DESCRIPTOR_TYPE_STRING                                       0x03
#define USB_DESCRIPTOR_TYPE_INTERFACE                                    0x04
#define USB_DESCRIPTOR_TYPE_ENDPOINT                                     0x05
#define USB_DESCRIPTOR_TYPE_DEVICE_QUALITIER                             0x06
#define USB_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION                    0x07
#define USB_DESCRIPTOR_TYPE_INTERFAACE_POWER                             0x08
#define USB_DESCRIPTOR_TYPE_OTG                                          0x09
#define USB_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION                        0x0B

#define USB_DESCRIPTOR_TYPE_HID                                          0x21
#define USB_DESCRIPTOR_TYPE_HID_REPORT                                   0x22
#define USB_DESCRIPTOR_TYPE_HID_PHYSICAL                                 0x23

/* USB standard request type */
#define USB_REQ_TYPE_DIR_MASK                                            0x80
#define USB_REQ_TYPE_DIR_SHIFT                                           7
#define USB_REQ_TYPE_DIR_OUT                                             0x00
#define USB_REQ_TYPE_DIR_IN                                              0x80

#define USB_REQ_TYPE_TYPE_MASK                                           0x60
#define USB_REQ_TYPE_TYPE_SHIFT                                          5
#define USB_REQ_TYPE_TYPE_STANDARD                                       0
#define USB_REQ_TYPE_TYPE_CLASS                                          0x20
#define USB_REQ_TYPE_TYPE_VENDOR                                         0x40

#define USB_REQ_TYPE_RECIPIENT_MASK                                      0x1F
#define USB_REQ_TYPE_RECIPIENT_SHIFT                                     0
#define USB_REQ_TYPE_RECIPIENT_DEVICE                                    0x00
#define USB_REQ_TYPE_RECIPIENT_INTERFACE                                 0x01
#define USB_REQ_TYPE_RECIPIENT_ENDPOINT                                  0x02
#define USB_REQ_TYPE_RECIPIENT_OTHER                                     0x03

/* USB standard request */
#define USB_REQ_STD_GET_STATUS                                           0x00
#define USB_REQ_STD_CLEAR_FEATURE                                        0x01
#define USB_REQ_STD_SET_FEATURE                                          0x03
#define USB_REQ_STD_SET_ADDRESS                                          0x05
#define USB_REQ_STD_GET_DESCRIPTOR                                       0x06
#define USB_REQ_STD_SET_DESCRIPTOR                                       0x07
#define USB_REQ_STD_GET_CONFIGURATION                                    0x08
#define USB_REQ_STD_SET_CONFIGURATION                                    0x09
#define USB_REQ_STD_GET_INTERFACE                                        0x0A
#define USB_REQ_STD_SET_INTERFACE                                        0x0B
#define USB_REQ_STD_SYNCH_FRAME                                          0x0C

/* USB standard request GET Status */
#define USB_REQ_STD_GET_STATUS_DEVICE_SELF_POWERED_SHIFT                 0
#define USB_REQ_STD_GET_STATUS_DEVICE_REMOTE_WARKUP_SHIFT                1

#define USB_REQ_STD_GET_STATUS_ENDPOINT_HALT_MASK                        0x01
#define USB_REQ_STD_GET_STATUS_ENDPOINT_HALT_SHIFT                       0

#define USB_REQ_STD_GET_STATUS_OTG_STATUS_SELECTOR                       0xF000

/* USB standard request CLEAR/SET feature */
#define USB_REQ_STD_FEATURE_SELECTOR_ENDPOINT_HALT                       0
#define USB_REQ_STD_FEATURE_SELECTOR_DEVICE_REMOTE_WAKEUP                1
#define USB_REQ_STD_FEATURE_SELECTOR_DEVICE_TEST_MODE                    2
#define USB_REQ_STD_FEATURE_SELECTOR_B_HNP_ENABLE                        3
#define USB_REQ_STD_FEATURE_SELECTOR_A_HNP_SUPPORT                       4
#define USB_REQ_STD_FEATURE_SELECTOR_A_ALT_HNP_SUPPORT                   5

/* USB standard descriptor configure bmAttributes */
#define USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK                       0x80
#define USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_SHIFT                      7

#define USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_MASK             0x40
#define USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_SHIFT            6

#define USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_MASK            0x20
#define USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_SHIFT           5

/* USB standard descriptor endpoint bmAttributes */
#define USB_DESC_EP_ADDR_DIR_MASK                                  0x80
#define USB_DESC_EP_ADDR_DIR_SHIFT                                 7
#define USB_DESC_EP_ADDR_DIR_OUT                                   0x00
#define USB_DESC_EP_ADDR_DIR_IN                                    0x80

#define USB_DESC_ENDPOINT_ADDRESS_NUMBER_MASK                      0x0F
#define USB_DESC_ENDPOINT_ADDRESS_NUMBER_SHFIT                     0

#define USB_EP_DIR(ep)                                   (((ep) & 0x80) >> 7)
#define USB_EP_NUMBER(ep)                                ((ep) & 0x0F)

#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_TYPE_MASK                      0x03
#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_NUMBER_SHFIT                   0

#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_SYNC_TYPE_MASK                 0x0C
#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_SYNC_TYPE_SHFIT                2
#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_SYNC_TYPE_NO_SYNC              0x00
#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_SYNC_TYPE_ASYNC                0x04
#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_SYNC_TYPE_ADAPTIVE             0x08
#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_SYNC_TYPE_SYNC                 0x0C

#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_USAGE_TYPE_MASK                0x30
#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_USAGE_TYPE_SHFIT               4
#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_USAGE_TYPE_DATA_ENDPOINT       0x00
#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_USAGE_TYPE_FEEDBACK_ENDPOINT   0x10
#define USB_DESCRIPTOR_ENDPOINT_ATTRIBUTE_USAGE_TYPE_IMPLICIT_FEEDBACK_DATA_ENDPOINT 0x20

#define USB_DESCRIPTOR_ENDPOINT_MAXPACKETSIZE_SIZE_MASK                  0x07FF
#define USB_DESCRIPTOR_ENDPOINT_MAXPACKETSIZE_MULT_TRANSACTIONS_MASK     0x1800
#define USB_DESCRIPTOR_ENDPOINT_MAXPACKETSIZE_MULT_TRANSACTIONS_SHFIT    11

/* USB standard descriptor otg bmAttributes */
#define USB_DESCRIPTOR_OTG_ATTRIBUTES_SRP_MASK                           0x01
#define USB_DESCRIPTOR_OTG_ATTRIBUTES_HNP_MASK                           0x02
#define USB_DESCRIPTOR_OTG_ATTRIBUTES_ADP_MASK                           0x04

typedef struct
{
    uint8_t  **string;
    uint32_t *length;
    uint16_t languageId;
} usb_language_t;

typedef struct
{
    uint8_t        *languageString;
    uint32_t       stringLength;
    usb_language_t *languageList;
    uint8_t        count;
} usb_language_list_t;

typedef struct
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bData[1];
} usb_descriptor_common_t;

typedef struct
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bcdUSB[2];
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint8_t idVendor[2];
    uint8_t idProduct[2];
    uint8_t bcdDevice[2];
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} usb_descriptor_device_t;

typedef struct
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t wTotalLength[2];
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
} usb_descriptor_configuration_t;

typedef struct
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} usb_descriptor_interface_t;

typedef struct
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint8_t wMaxPacketSize[2];
    uint8_t bInterval;
} usb_descriptor_endpoint_t;

typedef union
{
    usb_descriptor_common_t        common;
    usb_descriptor_device_t        device;
    usb_descriptor_configuration_t configuration;
    usb_descriptor_interface_t     interface;
    usb_descriptor_endpoint_t      endpoint;
} usb_descriptor_union_t;

#endif /* __USB_SPEC_H__ */
