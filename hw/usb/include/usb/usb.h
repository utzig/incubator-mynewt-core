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

#ifndef __USB_H__
#define __USB_H__

#include <stdint.h>
#include <stdio.h>

#include "usb_misc.h"
#include "usb_spec.h"

#define USB_STACK_VERSION_MAJOR     1
#define USB_STACK_VERSION_MINOR     4
#define USB_STACK_VERSION_BUGFIX    0

/* FIXME: apart from error enums and type definitions below,
 * everything here should probably be removed. No need for
 * version stuff, etc
 */

#define USB_MAKE_VERSION(major, minor, bugfix) \
    (((major) << 16) | ((minor) << 8) | (bugfix))

typedef enum
{
    kStatus_USB_Success = 0x00,             /*!< Success */
    kStatus_USB_Error,                      /*!< Failed */
    kStatus_USB_Busy,                       /*!< Busy */
    kStatus_USB_InvalidHandle,              /*!< Invalid handle */
    kStatus_USB_InvalidParameter,           /*!< Invalid parameter */
    kStatus_USB_InvalidRequest,             /*!< Invalid request */
    kStatus_USB_ControllerNotFound,         /*!< Controller cannot be found */
    kStatus_USB_InvalidControllerInterface, /*!< Invalid controller interface */
    kStatus_USB_NotSupported,        /*!< Configuration is not supported */
    kStatus_USB_Retry,               /*!< Enumeration get configuration retry */
    kStatus_USB_TransferStall,       /*!< Transfer stalled */
    kStatus_USB_TransferFailed,      /*!< Transfer failed */
    kStatus_USB_AllocFail,           /*!< Allocation failed */
    kStatus_USB_LackSwapBuffer,      /*!< Insufficient swap buffer for KHCI */
    kStatus_USB_TransferCancel,      /*!< The transfer cancelled */
    kStatus_USB_BandwidthFail,       /*!< Allocate bandwidth failed */
    kStatus_USB_MSDStatusFail,       /*!< For MSD, the CSW status means fail */
} usb_status_t;

typedef void *usb_host_handle;
typedef void *usb_device_handle;
typedef void *usb_otg_handle;

/* USB controller ID */
/* FIXME: this should be removed, hal dep takes care of it */
typedef enum _usb_controller_index
{
    kUSB_ControllerKhci0 = 0,
    kUSB_ControllerKhci1,
    kUSB_ControllerEhci0,
    kUSB_ControllerEhci1,
    kUSB_ControllerLpcIp3511Fs0,
    kUSB_ControllerLpcIp3511Fs1,
    kUSB_ControllerLpcIp3511Hs0,
    kUSB_ControllerLpcIp3511Hs1,
} usb_controller_index_t;

typedef struct _usb_version
{
    uint8_t major;
    uint8_t minor;
    uint8_t bugfix;
} usb_version_t;

/*! @} */

#endif /* __USB_H__ */
