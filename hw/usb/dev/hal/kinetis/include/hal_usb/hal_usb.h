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
 * Copyright 2016 NXP
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
 * o Neither the name of the copyright holder nor the names of its
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

#ifndef __HAL_USB_H__
#define __HAL_USB_H__

#include "MK64F12.h"

#define USB_DATA_ALIGNMENT

#define USB_KHCI_BDT_DEVICE_OUT_TOKEN     0x01
#define USB_KHCI_BDT_DEVICE_IN_TOKEN      0x09
#define USB_KHCI_BDT_DEVICE_SETUP_TOKEN   0x0D

#define USB_KHCI_BDT_OWN                  0x80
#define USB_KHCI_BDT_DATA01(x)            ((((uint32_t)(x)) & 0x01) << 0x06)
#define USB_KHCI_BDT_BC(x)                ((((uint32_t)(x)) & 0x3FF) << 0x10)
#define UBS_KHCI_BDT_KEEP                 0x20
#define UBS_KHCI_BDT_NINC                 0x10
#define USB_KHCI_BDT_DTS                  0x08
#define USB_KHCI_BDT_STALL                0x04

#define USB_DEVICE_MAX_FS_ISO_MAX_PACKET_SIZE       1023
#define USB_DEVICE_MAX_FS_NONE_ISO_MAX_PACKET_SIZE  64

typedef struct
{
    uint8_t  *transferBuffer;
    uint32_t transferLength;
    uint32_t transferDone;
    union
    {
        uint32_t state;
        struct
        {
            uint32_t maxPacketSize : 10;
            uint32_t stalled : 1;
            uint32_t data0 : 1;
            uint32_t bdtOdd : 1;
            uint32_t dmaAlign : 1;
            uint32_t transferring : 1;
            uint32_t zlt : 1;
        } stateBitField;
    } stateUnion;
} kinetis_usb_dev_ep_state_t;

typedef struct
{
    usb_dev_t           *dev;
    uint8_t             isResetting;
    uint8_t             controllerId;

    /* device specific */
    uint8_t             *bdt;
    volatile USB_Type   *registers;
    uint8_t             setupPacketBuffer[USB_SETUP_PACKET_SIZE * 2]; /*!< The setup request buffer */
    uint8_t             *dmaAlignBuffer; /*!< This buffer is used to fix the transferBuffer or transferLength does
                                            not align to 4-bytes when the function USB_DeviceKhciRecv is called.
                                            The macro USB_DEVICE_CONFIG_KHCI_DMA_ALIGN is used to enable or disable this feature.
                                            If the feature is enabled, when the transferBuffer or transferLength does not align to
                                            4-bytes,
                                            the transferLength is not more than USB_DEVICE_CONFIG_KHCI_DMA_ALIGN_BUFFER_LENGTH, and
                                            the flag isDmaAlignBufferInusing is zero, the dmaAlignBuffer is used to receive data
                                            and the flag isDmaAlignBufferInusing is set to 1.
                                            When the transfer is done, the received data, kept in dmaAlignBuffer, is copied
                                            to the transferBuffer, and the flag isDmaAlignBufferInusing is cleared.
                                          */
    kinetis_usb_dev_ep_state_t endpointState[MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS) * 2];
    uint8_t isDmaAlignBufferInusing;
    uint8_t setupBufferIndex;
#if defined(USB_DEVICE_CONFIG_OTG)
    uint8_t otgStatus;
#endif
} kinetis_usb_dev_state_t;

void usb_hal_init_clocks(void);
const usb_dev_ctrl_itf_t * usb_hal_controller_interface(void);
void usb_hal_set_dev_handle(void *);
void usb_hal_enable_irq(void);
void usb_hal_clear_memory(void);

#endif /* __HAL_USB_H__ */
