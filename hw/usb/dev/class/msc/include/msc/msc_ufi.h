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

#ifndef MSC_UFI_H
#define MSC_UFI_H

#include <stdint.h>

#define USB_DEVICE_MSC_UFI_NO_SENSE                                0x00
#define USB_DEVICE_MSC_UFI_RECOVERED_ERROR                         0x01
#define USB_DEVICE_MSC_UFI_NOT_READY                               0x02
#define USB_DEVICE_MSC_UFI_MEDIUM_ERROR                            0x03
#define USB_DEVICE_MSC_UFI_HARDWARE_ERROR                          0x04
#define USB_DEVICE_MSC_UFI_ILLEGAL_REQUEST                         0x05
#define USB_DEVICE_MSC_UFI_UNIT_ATTENTION                          0x06
#define USB_DEVICE_MSC_UFI_DATA_PROTECT                            0x07
#define USB_DEVICE_MSC_UFI_BLANK_CHECK                             0x08
#define USB_DEVICE_MSC_UFI_VENDOR_SPECIFIC_ERROR                   0x09
#define USB_DEVICE_MSC_UFI_ABORTED_COMMAND                         0x0B
#define USB_DEVICE_MSC_UFI_VOLUME_OVERFLOW                         0x0D
#define USB_DEVICE_MSC_UFI_MISCOMPARE                              0x0E

#define USB_DEVICE_MSC_UFI_INVALID_COMMAND_OPCODE                  0x20
#define USB_DEVICE_MSC_UFI_WRITE_FAULT                             0x03
#define USB_DEVICE_MSC_UFI_UNRECOVERED_READ_ERROR                  0x11
#define USB_DEVICE_MSC_UFI_UNKNOWN_ERROR                           0xFF
#define USB_DEVICE_MSC_UFI_INVALID_FIELD_IN_COMMAND_PKT            0x24
#define USB_DEVICE_MSC_UFI_LBA_OUT_OF_RANGE                        0x21

#define USB_DEVICE_MSC_UFI_REQ_SENSE_VALID_ERROR_CODE              0x70
#define USB_DEVICE_MSC_UFI_REQ_SENSE_ADDITIONAL_SENSE_LEN          0x0A

#define USB_DEVICE_MSC_UFI_PREVENT_ALLOW_REMOVAL_MASK              0x01
#define USB_DEVICE_MSC_UFI_LOAD_EJECT_START_MASK                   0x03

#define USB_DEVICE_MSC_UFI_FORMATTED_MEDIA                         0x02
#define USB_DEVICE_MSC_UFI_UNFORMATTED_MEDIA                       0x01
#define USB_DEVICE_MSC_UFI_NO_CARTRIDGE_IN_DRIVE                   0x03

#define USB_DEVICE_MSC_UFI_INQUIRY_ALLOCATION_LENGTH               0x24
#define USB_DEVICE_MSC_UFI_REQ_SENSE_DATA_LENGTH                   18
#define USB_DEVICE_MSC_UFI_READ_CAPACITY_DATA_LENGTH               0x08
#define USB_DEVICE_MSC_UFI_READ_CAPACITY16_DATA_LENGTH             0x0C

#define USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER                    0
#define USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER_SHIFT              5
#define USB_DEVICE_MSC_UFI_VERSIONS                                4
#define USB_DEVICE_MSC_UFI_PERIPHERAL_DEVICE_TYPE                  0x00
#define USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT                    1
#define USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT_SHIFT              7
#define USB_DEVICE_MSC_UFI_ADDITIONAL_LENGTH                       0x20

typedef struct
{
    uint8_t operationCode;     /*!< Operation Code*/
    uint8_t logicalUnitNumber; /*!< Specifies the logical unit (0~7) for which Inquiry data should be returned*/
    uint8_t pageCode;          /*!< Page Code*/
    uint8_t reserved;          /*!< Reserved*/
    uint8_t allocationLength;  /*!< Specifies the maximum number of bytes of inquiry data to be returned*/
    uint8_t reserved1[7];      /*!< Reserved*/
} usb_device_inquiry_command_struct_t;

typedef struct
{
    uint8_t operationCode;     /*!< Operation Code*/
    uint8_t logicalUnitNumber; /*!< Logical Unit Number*/
    uint8_t reserved[2];       /*!< reserved*/
    uint8_t allocationLength;  /*!< Allocation Length*/
    uint8_t reserved1[7];      /*!< reserved*/
} usb_device_request_sense_command_struct_t;

typedef struct
{
    uint8_t operationCode;     /*!< Operation Code*/
    uint8_t logicalUnitNumber; /*!< Logical Unit Number*/
    uint8_t reserved[5];       /*!< reserved*/
    uint16_t allocationLength; /*!< Allocation Length*/
    uint8_t reserved1[3];      /*!< reserved*/
} usb_device_read_format_capacities_command_struct_t;

typedef struct
{
    uint8_t operationCode;     /*!< Operation Code*/
    uint8_t logicalUnitNumber; /*!< Logical Unit Number*/
    uint32_t lba;              /*!< Logical Block Address*/
    uint8_t reserved[2];       /*!< Reserved*/
    uint8_t pmi;               /*!< This bit should be set to zero for UFI*/
    uint8_t reserved1[3];      /*!< Reserved*/
} usb_device_read_capacities_command_struct_t;

typedef struct
{
    uint8_t operationCode;     /*!< Operation Code*/
    uint8_t lunDpoFuaReladr;   /*!< Logical Unit Number DPO FUA RelAdr*/
    uint32_t lba;              /*!< Logical Block Address*/
    uint8_t reserved;          /*!< Reserved*/
    uint8_t transferLengthMsb; /*!< Transfer Length (MSB)*/
    uint8_t transferLengthLsb; /*!< Transfer Length (LSB)*/
    uint8_t reserved1[3];      /*!< Reserved*/
} usb_device_read_write_10_command_struct_t;

typedef struct
{
    uint8_t peripheralDeviceType; /*!< Peripheral Device Type*/
    uint8_t rmb;                  /*!< Removable Media Bit*/
    uint8_t versions;             /*!< ISO Version, ECMA Version, ANSI Version*/
    uint8_t responseDataFormat;   /*!< Response Data Format*/
    uint8_t additionalLength;     /*!< The Additional Length field shall specify the length in bytes of the parameters*/
    uint8_t reserved[3];          /*!< reserved*/
    uint8_t vendorInformatin[8];  /*!< Vendor Identification*/
    uint8_t productId[16];        /*!< Product Identification*/
    uint8_t productVersionLevel[4]; /*!< Product Revision Level*/
} usb_device_inquiry_data_fromat_struct_t;

typedef struct
{
    uint8_t validErrorCode;          /*!< Error Code*/
    uint8_t reserved;                /*!< reserved*/
    uint8_t senseKey;                /*!< Sense Key*/
    uint8_t information[4];          /*!< Information*/
    uint8_t additionalSenseLength;   /*!< Additional Sense Length*/
    uint8_t reserved1[4];            /*!< reserved*/
    uint8_t additionalSenseCode;     /*!< Additional Sense Code*/
    uint8_t additionalSenseQualifer; /*!< Additional Sense Code Qualifier*/
    uint8_t reserved2[4];            /*!< reserved*/
} usb_device_request_sense_data_struct_t;

typedef struct
{
    uint32_t lastLogicalBlockAddress; /*!< Last Logical Block Address*/
    uint32_t blockSize;               /*!< Block Length In Bytes*/
} usb_device_read_capacity_struct_t;

typedef struct
{
    uint32_t lastLogicalBlockAddress0; /*!<  Last Logical Block Address*/
    uint32_t lastLogicalBlockAddress1; /*!<  Last Logical Block Address*/
    uint32_t blockSize;                /*!< Block Length In Bytes*/
} usb_device_read_capacity16_data_struct_t;

typedef struct
{
    uint8_t reserverd[3];       /*!< reserved*/
    uint8_t capacityListLength; /*!< Capacity List Length*/
} usb_device_capacity_list_header_struct_t;

typedef struct
{
    uint32_t blockNumber;               /*!< Number of Blocks*/
    uint32_t descriptorCodeBlockLength; /*!< Byte 4 Descriptor Code , byte 5-7 Block Length*/
} usb_device_current_max_capacity_descriptor_struct_t;

typedef struct
{
    uint32_t blockNumber; /*!< Number of Blocks*/
    uint32_t blockLength; /*!< Block Length*/
} usb_device_formattable_capacity_descriptor_struct_t;

typedef struct
{
    uint16_t modeDataLength; /*!< Mode Data Length*/
    uint8_t mediumTypeCode;  /*!< The Medium Type Code field specifies the inserted medium type*/
    uint8_t wpDpfua;         /*!< WP and DPOFUA bit*/
    uint8_t reserved[4];     /*!< Reserved*/
} usb_device_mode_parameters_header_struct_t;

typedef struct
{
    uint8_t capacityListHead[sizeof(usb_device_capacity_list_header_struct_t)]; /*!<Capacity List Header*/
    uint8_t currentMaxCapacityDesccriptor[sizeof(
        usb_device_current_max_capacity_descriptor_struct_t)]; /*!<Current/Maximum Capacity Header*/
    uint8_t formattableCapacityDesccriptor[sizeof(usb_device_formattable_capacity_descriptor_struct_t) *
                                           3]; /*!<Formatting Capacity Descriptor*/
} usb_device_format_capacity_response_data_struct_t;

#endif
