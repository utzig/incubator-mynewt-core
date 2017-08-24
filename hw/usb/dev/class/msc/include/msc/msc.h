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

#ifndef MSC_H
#define MSC_H

#include <stdint.h>
#include <usb/usb.h>

#include "msc_ufi.h"

#define USB_DEVICE_CONFIG_MSC_SUPPORT_DISK_LOCKING_MECHANISM   0
#define USB_DEVICE_CONFIG_MSC_IMPLEMENTING_DISK_DRIVE          0

#define USB_DEVICE_CONFIG_MSC_CLASS_CODE                       0x08

#define USB_DEVICE_MSC_BULK_ONLY_MASS_STORAGE_RESET            0xFF
#define USB_DEVICE_MSC_GET_MAX_LUN                             0xFE

#define USB_DEVICE_MSC_DCBWSIGNATURE            USB_LONG_TO_BIG_ENDIAN(0x55534243)
#define USB_DEVICE_MSC_DCSWSIGNATURE            USB_LONG_TO_BIG_ENDIAN(0x55534253U)

#define USB_DEVICE_MSC_CBW_DIRECTION_BIT                       0x80
#define USB_DEVICE_MSC_CBW_DIRECTION_SHIFT                     7

#define USB_DEVICE_MSC_CBW_LENGTH                              31

#define USB_DEVICE_MSC_CSW_LENGTH                              13

#define USB_DEVICE_MSC_COMMAND_PASSED                          0x00
#define USB_DEVICE_MSC_COMMAND_FAILED                          0x01
#define USB_DEVICE_MSC_PHASE_ERROR                             0x02

#define USB_DEVICE_MSC_INQUIRY_COMMAND                         0x12
#define USB_DEVICE_MSC_READ_10_COMMAND                         0x28
#define USB_DEVICE_MSC_READ_12_COMMAND                         0xA8
#define USB_DEVICE_MSC_REQUEST_SENSE_COMMAND                   0x03
#define USB_DEVICE_MSC_TEST_UNIT_READY_COMMAND                 0x00
#define USB_DEVICE_MSC_WRITE_10_COMMAND                        0x2A
#define USB_DEVICE_MSC_WRITE_12_COMMAND                        0xAA
#define USB_DEVICE_MSC_PREVENT_ALLOW_MEDIUM_REM_COMMAND        0x1E
#define USB_DEVICE_MSC_FORMAT_UNIT_COMMAND                     0x04
#define USB_DEVICE_MSC_READ_CAPACITY_10_COMMAND                0x25
#define USB_DEVICE_MSC_READ_CAPACITY_16_COMMAND                0x9E
#define USB_DEVICE_MSC_READ_FORMAT_CAPACITIES_COMMAND          0x23
#define USB_DEVICE_MSC_MODE_SENSE_10_COMMAND                   0x5A
#define USB_DEVICE_MSC_MODE_SENSE_6_COMMAND                    0x1A
#define USB_DEVICE_MSC_MODE_SELECT_10_COMMAND                  0x55
#define USB_DEVICE_MSC_MODE_SELECT_6_COMMAND                   0x15
#define USB_DEVICE_MSC_SEND_DIAGNOSTIC_COMMAND                 0x1D
#define USB_DEVICE_MSC_VERIFY_COMMAND                          0x2F
#define USB_DEVICE_MSC_START_STOP_UNIT_COMMAND                 0x1B

#define USB_DEVICE_MSC_MAX_RECV_TRANSFER_LENGTH                65536
#define USB_DEVICE_MSC_MAX_SEND_TRANSFER_LENGTH                65536

typedef struct
{
    uint32_t signature;          /*!< Byte 0-3 dCBWSignature*/
    uint32_t tag;                /*!< Byte 4-7 dCBWTag*/
    uint32_t dataTransferLength; /*!< Byte 8-11 dCBWDataTransferLength*/
    uint8_t flags;               /*!< Byte 12 bmCBWFlags*/
    uint8_t logicalUnitNumber;   /*!< Byte 13 bCBWLUN*/
    uint8_t cbLength;            /*!< Byte 14 bCBWCBLength*/
    uint8_t cbwcb[16];           /*!< Byte 15-30 CBWCB, CBWCB is used to store UFI command*/
} usb_device_msc_cbw_t;

typedef struct
{
    uint32_t signature;          /*!< Byte 0-3 dCSWSignature*/
    uint32_t tag;                /*!< Byte 4-7 dCSWTag*/
    uint32_t dataResidue;        /*!< Byte 8-11 dCSWDataResidue*/
    uint8_t cswStatus;           /*!< Byte 12 bCSWStatus*/
} usb_device_msc_csw_t;

typedef struct
{
    uint32_t startingLogicalBlockAddress; /*!< The logical block at which the read/write operation shall begin*/
    uint32_t transferNumber;              /*!< The number of contiguous logical blocks of data that shall be transferred*/
} usb_lba_transfer_information_struct_t;

typedef struct
{
    uint32_t totalLbaNumberSupports;    /*!< Total blocks number supported*/
    uint32_t lengthOfEachLba;           /*!< Length of each block*/
    uint32_t bulkInBufferSize;          /*!< Bulk in buffer size*/
    uint32_t bulkOutBufferSize;         /*!< Bulk out buffer size*/
    uint8_t logicalUnitNumberSupported; /*!< Number of LUN*/
} usb_device_lba_information_struct_t;

typedef struct
{
    uint32_t offset;                    /*!< Offset of the block need to access*/
    uint32_t size;                      /*!< Size of the transferred data*/
    uint8_t *buffer;                    /*!< Buffer address of the transferred data*/
} usb_device_lba_app_struct_t;

typedef struct
{
    uint8_t *cbwcb;                     /*!< current ufi command block strored in the CBW*/
    uint32_t size;                      /*!< Size of the transferred data if commmand has data flow*/
    uint8_t *buffer;                    /*!< Buffer address of the transferred data if commmand has data flow*/
    usb_device_request_sense_data_struct_t *requestSense; /*!< sense data for the current command*/
} usb_device_ufi_app_struct_t;

typedef struct
{
    uint32_t hostExpectedDataLength;   /*!< The number of bytes of data that the host expects to transfer */
    uint32_t deviceExpectedDataLength; /*!< The number of bytes of data that the device expects to transfer */
    uint8_t *buffer;                   /*!< Data buffer*/
    usb_lba_transfer_information_struct_t lbaInformation; /*!< Read/write information*/
    uint8_t lbaSendRecvSelect;                            /*!< Whether the command is read or write command*/
    uint8_t hostExpectedDirection;                        /*!< Host expected data direction*/
    uint8_t deviceExpectedDirection;                      /*!< Device expected data direction*/
} usb_device_msc_thirteen_case_struct_t;

typedef enum
{
    USB_DEVICE_MSC_STALL_IN_CBW = 1,  /*!< Stall in CBW*/
    USB_DEVICE_MSC_STALL_IN_DATA,     /*!< Stall in data transfer*/
    USB_DEVICE_MSC_STALL_IN_CSW,      /*!< Stall in CSW*/
} usb_device_msc_stall_type;

typedef enum
{
    kUSB_DeviceMscEventReadResponse = 0x01,  /*!< host has already read the whole data from device */
    kUSB_DeviceMscEventWriteResponse,        /*!< devcie has already received the data from host. */
    kUSB_DeviceMscEventWriteRequest,         /*!< Host want to write data to device through write command, devcie need prepare one buffer to store the data from host*/
    kUSB_DeviceMscEventReadRequest,          /*!< Host want to read data from device through read command, device need prepare one buffer containing data pending for transfer*/
    kUSB_DeviceMscEventGetLbaInformation,    /*!< Get device information */
    kUSB_DeviceMscEventFormatComplete,       /*!< Format complete */
    kUSB_DeviceMscEventTestUnitReady,        /*!<  Test Unit Ready command*/
    kUSB_DeviceMscEventInquiry,              /*!<  Inquiry Command command*/
    kUSB_DeviceMscEventModeSense,            /*!<  mode sense command*/
    kUSB_DeviceMscEventModeSelect,           /*!<  mode select command, prepare data buffer and buffer length to store data for mode select*/
    kUSB_DeviceMscEventModeSelectResponse, /*!<  got data of mode select command*/
    kUSB_DeviceMscEventRemovalRequest,     /*!< Prevent_allow_medium_command */
    kUSB_DeviceMscEventSendDiagnostic,     /*!< Send Diagnostic command */
    kUSB_DeviceMscEventStopEjectMedia,     /*!< Start_stop_unit_command */

} USB_DeviceMscEvent_t;

typedef struct
{
    usb_device_request_sense_data_struct_t requestSense;             /*!< Request Sense Standard Data*/
    usb_device_msc_thirteen_case_struct_t thirteenCase;              /*!< Thirteen possible cases*/
    usb_device_read_capacity_struct_t readCapacity;                  /*!< READ CAPACITY Data*/
    usb_device_read_capacity16_data_struct_t readCapacity16;         /*!< READ CAPACITY Data*/
    usb_device_inquiry_data_fromat_struct_t InquiryInfo;             /*!< Standard INQUIRY Data*/
    usb_device_mode_parameters_header_struct_t ModeParametersHeader; /*!< Mode Parameter Header*/
    uint8_t formattedDisk;                                           /*!< *Formatted or unformatted media*/
    uint8_t formatCapacityData[sizeof(usb_device_capacity_list_header_struct_t) +
                               sizeof(usb_device_current_max_capacity_descriptor_struct_t) +
                               sizeof(usb_device_formattable_capacity_descriptor_struct_t) * 3];
    /*!< Capacity List*/
} usb_device_msc_ufi_struct_t;

typedef struct _usb_device_msc_struct
{
    usb_device_handle handle;
    usb_dev_class_config_t *configurationStruct;
    usb_dev_itf_t *interfaceHandle;
    uint32_t transferRemaining;
    uint32_t currentOffset;
    uint32_t totalLogicalBlockNumber;
    uint32_t lengthOfEachLba;
    uint32_t implementingDiskDrive;
    uint32_t bulkInBufferSize;
    uint32_t bulkOutBufferSize;

    usb_device_msc_cbw_t *mscCbw; /*!< CBW structure */
    usb_device_msc_csw_t *mscCsw; /*!< CSW structure */

    usb_device_msc_ufi_struct_t mscUfi; /*!< UFI command information structure*/

    uint8_t dataOutFlag;          /*!< CBW indicating bulk out transfer, clear this flag when data transfer done*/
    uint8_t dataInFlag;           /*!< CBW indicating bulk in transfer, clear this flag when data transfer done*/
    uint8_t inEndpointStallFlag;  /*!< In endpoint stall flag*/
    uint8_t outEndpointStallFlag; /*!< Out endpoint stall flag*/
    uint8_t cbwValidFlag; /*!< The CBW was received after the device had sent a CSW or after a reset ,or else it is
                             invalid*/
    uint8_t performResetRecover;  /*!< Device need reset command from host*/
    uint8_t performResetDoneFlag; /*!< Device has perform reset command */
    uint8_t needInStallFlag;      /*!< In endpoint should be stalled*/
    uint8_t needOutStallFlag;     /*!< Out endpoint should be stalled*/
    uint8_t cbwPrimeFlag; /*!< CBW prime flag, prime means device MSC has been ready to receive CBW, the bulk out
                             endpoint has got the prepared buffer*/
    uint8_t cswPrimeFlag; /*!< CSW prime flag, prime means device MSC has been ready to receive CSW, the bulk in
                             endpoint has got the prepared buffer*/
    uint8_t stallStatus;  /*!< Stall status*/

    uint8_t logicalUnitNumber; /*!< Supported logical units number of device. See bulk only specification 3.2 Get
                                  Maximum LUN
                                  (class-specific request)*/
    uint8_t bulkInEndpoint;    /*!< Bulk in endpoint number*/
    uint8_t bulkOutEndpoint;   /*!< Bulk out endpoint number*/
    uint8_t alternate;         /*!< Current alternate setting of the interface */
    uint8_t configuration;     /*!< Current configuration */
    uint8_t interfaceNumber;   /*!< The interface number of the class */
} usb_dev_msc_t;

usb_status_t usb_dev_msc_init(uint8_t controllerId,
                              usb_dev_class_config_t *config,
                              class_handle_t *handle);
usb_status_t usb_dev_msc_deinit(class_handle_t handle);
usb_status_t usb_dev_msc_event(void *handle, uint32_t event, void *param);
//usb_status_t USB_DeviceMscLbaTransfer(usb_dev_msc_t *mscHandle,
//                                             uint8_t direction,
//                                             usb_lba_transfer_information_struct_t *lba_info_ptr);

#ifdef __cplusplus
extern "C" {
#endif

extern usb_status_t USB_DeviceMscUfiThirteenCasesCheck(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiRequestSenseCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiInquiryCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiReadCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiWriteCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiTestUnitReadyCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiVerifyCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiModeSenseCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiModeSelectCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiReadCapacityCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiReadFormatCapacityCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiFormatUnitCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiPreventAllowMediumCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiSendDiagnosticCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiStartStopUnitCommand(usb_dev_msc_t *mscHandle);
extern usb_status_t USB_DeviceMscUfiUnsupportCommand(usb_dev_msc_t *mscHandle);

#ifdef __cplusplus
}
#endif

#endif
