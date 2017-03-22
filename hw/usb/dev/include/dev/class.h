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

#ifndef __USB_DEVICE_CLASS_H__
#define __USB_DEVICE_CLASS_H__

#include "dev.h"

#define class_handle_t uint32_t

typedef enum
{
    kUSB_DeviceClassTypeHid = 1,
    kUSB_DeviceClassTypeCdc,
    kUSB_DeviceClassTypeMsc,
    kUSB_DeviceClassTypeAudio,
    kUSB_DeviceClassTypePhdc,
    kUSB_DeviceClassTypeVideo,
    kUSB_DeviceClassTypePrinter,
    kUSB_DeviceClassTypeDfu,
    kUSB_DeviceClassTypeCcid,
} usb_dev_class_type_t;

typedef enum
{
    kUSB_DeviceClassEventClassRequest = 1,
    kUSB_DeviceClassEventDeviceReset,
    kUSB_DeviceClassEventSetConfiguration,
    kUSB_DeviceClassEventSetInterface,
    kUSB_DeviceClassEventSetEndpointHalt,
    kUSB_DeviceClassEventClearEndpointHalt,
} usb_device_class_event_t;

typedef struct
{
    uint8_t  ep_addr;
    uint8_t  transferType;
    uint16_t maxPacketSize;
} usb_dev_ep_t;

typedef struct
{
    uint8_t                      count;
    usb_dev_ep_t                 *ep;
} usb_dev_ep_list_t;

typedef struct
{
    uint8_t                    alternateSetting;
    usb_dev_ep_list_t          eps;
    void                       *classSpecific;
} usb_dev_itf_t;

typedef struct
{
    uint8_t                       classCode;
    uint8_t                       subclassCode;
    uint8_t                       protocolCode;
    uint8_t                       itf_num;
    usb_dev_itf_t                 *itf;
    uint8_t                       count;
} usb_dev_itfs_t;

typedef struct
{
    uint8_t                        count;
    usb_dev_itfs_t                 *itfs;
} usb_device_interface_list_t;

typedef struct
{
    usb_device_interface_list_t    *interfaceList;
    usb_dev_class_type_t           type;
    uint8_t                        configurations;
} usb_dev_class_t;

typedef int (*usb_dev_class_cb_fn)(class_handle_t class_handle,
        uint32_t cb_event, void *param);

typedef struct
{
    usb_dev_class_cb_fn    cb;
    class_handle_t         handle;
    usb_dev_class_t        *info;
} usb_dev_class_config_t;

/*!
 * Structure representing the device class configuration information.
 */
typedef struct
{
    usb_dev_class_config_t    *config;
    usb_device_callback_t     deviceCallback;
    uint8_t                   count;
} usb_dev_class_configs_t;

/*!
 * @brief Obtains the control request structure.
 *
 * This structure is used to pass the control request information.
 * The structure is used in following two cases.
 * 1. Case one, the host wants to send data to the device in the control data stage: @n
 *         a. If a setup packet is received, the structure is used to pass the
 *                setup packet data and wants to get the buffer to receive data
 *                sent from the host.
 *            The field isSetup is 1.
 *            The length is the requested buffer length.
 *            The buffer is filled by the class or application by using the valid
 *                buffer address.
 *            The setup is the setup packet address.
 *         b. If the data received is sent by the host, the structure is used to
 *            pass the data buffer address and the data length sent by the host.
 *            In this way, the field isSetup is 0.
 *            The buffer is the address of the data sent from the host.
 *            The length is the received data length.
 *            The setup is the setup packet address. @n
 * 2. Case two, the host wants to get data from the device in control data stage: @n
 *            If the setup packet is received, the structure is used to pass the setup packet data and wants to get the
 * data buffer address to send data to the host.
 *            The field isSetup is 1.
 *            The length is the requested data length.
 *            The buffer is filled by the class or application by using the valid buffer address.
 *            The setup is the setup packet address.
 *
 */
typedef struct
{
    usb_setup_t *setup;
    uint8_t     *buf;
    uint32_t    len;
    uint8_t     is_setup;
} usb_dev_ctrl_req_t;

typedef struct
{
    uint8_t  *buf;
    uint32_t len;
} usb_desc_common_t;

typedef struct
{
    uint8_t  *buf;
    uint32_t len;
} usb_desc_device_t;

typedef struct
{
    uint8_t  *buf;
    uint32_t len;
} usb_desc_device_qualifier_t;

typedef struct
{
    uint8_t  *buf;
    uint32_t len;
    uint8_t  config;
} usb_desc_configuration_t;

typedef struct
{
    uint8_t  *buf;
    uint32_t len;
    uint16_t lang_id;
    uint8_t  str_idx;
} usb_desc_string_t;

typedef struct
{
    uint8_t  *buf;
    uint32_t len;
    uint8_t  itf_num;
} usb_desc_hid_t;

typedef struct
{
    uint8_t  *buf;
    uint32_t len;
    uint8_t  itf_num;
} usb_desc_hid_report_t;

typedef struct
{
    uint8_t  *buf;
    uint32_t len;
    uint8_t  idx;
    uint8_t  itf_num;
} usb_desc_hid_physical_t;

typedef union
{
    usb_desc_common_t                 common_desc;
    usb_desc_device_t                 device_desc;
    usb_desc_device_qualifier_t       device_qualifier_desc;
    usb_desc_configuration_t          configuration_desc;
    usb_desc_string_t                 string_desc;
    usb_desc_hid_t                    hid_desc;
    usb_desc_hid_report_t             hid_report_desc;
    usb_desc_hid_physical_t           hid_physical_desc;
} usb_desc_t;

typedef int (*usb_dev_class_init_fn)(uint8_t ctrl_id,
        usb_dev_class_config_t *config, class_handle_t *handle);

typedef int (*usb_dev_class_deinit_fn)(class_handle_t handle);

typedef int (*usb_dev_class_event_fn)(void *classHandle,
        uint32_t event, void *param);

typedef struct
{
    usb_dev_class_init_fn           init;
    usb_dev_class_deinit_fn         deinit;
    usb_dev_class_event_fn          cb;
    usb_dev_class_type_t            type;
} usb_device_class_map_t;

typedef struct
{
    usb_device_handle               handle;
    usb_dev_class_configs_t         *configs;
    uint8_t                         setupBuffer[USB_SETUP_PACKET_SIZE];
    uint16_t                        std_transact_buf;
    uint8_t                         ctrl_id;
} usb_device_common_class_t;

#if defined(__cplusplus)
extern "C" {
#endif

int usb_device_class_init(uint8_t ctrl_id, usb_dev_class_configs_t *configs,
        usb_device_handle *handle);
int usb_device_class_deinit(uint8_t ctrl_id);
int usb_device_class_get_speed(uint8_t ctrl_id, uint8_t *speed);
int usb_device_class_event(usb_device_handle handle,
        usb_device_class_event_t event, void *param);
int usb_device_class_cb(usb_device_handle handle, uint32_t event, void *param);
int usb_device_class_get_handle(uint8_t ctrl_id, usb_device_handle *handle);

/*
 * Helpers to work with descriptors.
 */
int usb_desc_get_string(usb_device_handle handle, usb_desc_string_t *desc);
int usb_desc_strings_alloc(void);


#if defined(__cplusplus)
}
#endif

#endif /* __USB_DEVICE_CLASS_H__ */
