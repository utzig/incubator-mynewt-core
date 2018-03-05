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

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <usb/usb.h>
#include <dev/dev.h>
#include <dev/class.h>

#if MYNEWT_VAL(USB_DEVICE_CONFIG_MSC)
#include <msc/msc.h>

#include <hal_usb/hal_usb.h>

int usb_dev_msc_recv(usb_dev_msc_t *msc);
int usb_dev_msc_send(usb_dev_msc_t *msc);
int usb_dev_msc_bulk_out(usb_device_handle handle, usb_dev_ep_cb_msg_t *msg,
        void *param);
int usb_dev_msc_bulk_in(usb_device_handle handle, usb_dev_ep_cb_msg_t *msg,
        void *param);

#if MYNEWT_VAL(USB_DEVICE_CONFIG_MSC) == 1
usb_device_msc_cbw_t s_MscCbw1;
#define MSCCBW_ARRAY ((usb_device_msc_cbw_t *[]){&s_MscCbw1})
usb_device_msc_csw_t s_MscCsw1;
#define MSCCSW_ARRAY ((usb_device_msc_csw_t *[]){&s_MscCsw1})
#elif MYNEWT_VAL(USB_DEVICE_CONFIG_MSC) == 2
usb_device_msc_cbw_t s_MscCbw1;
usb_device_msc_cbw_t s_MscCbw2;
#define MSCCBW_ARRAY ((usb_device_msc_cbw_t *[]){&s_MscCbw1, &s_MscCbw2})
usb_device_msc_csw_t s_MscCsw1;
usb_device_msc_csw_t s_MscCsw2;
#define MSCCSW_ARRAY ((usb_device_msc_csw_t *[]){&s_MscCsw1, &s_MscCsw2})
#elif MYNEWT_VAL(USB_DEVICE_CONFIG_MSC) == 3
usb_device_msc_cbw_t s_MscCbw1;
usb_device_msc_cbw_t s_MscCbw2;
usb_device_msc_cbw_t s_MscCbw3;
#define MSCCBW_ARRAY ((usb_device_msc_cbw_t *[]){&s_MscCbw1, &s_MscCbw2, &s_MscCbw3})
usb_device_msc_csw_t s_MscCsw1;
usb_device_msc_csw_t s_MscCsw2;
usb_device_msc_csw_t s_MscCsw3;
#define MSCCSW_ARRAY ((usb_device_msc_csw_t *[]){&s_MscCsw1, &s_MscCsw2, &s_MscCsw3})
#elif USB_DEVICE_CONFIG_MSC == 4
usb_device_msc_cbw_t s_MscCbw1;
usb_device_msc_cbw_t s_MscCbw2;
usb_device_msc_cbw_t s_MscCbw3;
usb_device_msc_cbw_t s_MscCbw4;
#define MSCCBW_ARRAY ((usb_device_msc_cbw_t *[]){&s_MscCbw1, &s_MscCbw2, &s_MscCbw3, &s_MscCbw4})
usb_device_msc_csw_t s_MscCsw1;
usb_device_msc_csw_t s_MscCsw2;
usb_device_msc_csw_t s_MscCsw3;
usb_device_msc_csw_t s_MscCsw4;
#define MSCCSW_ARRAY ((usb_device_msc_csw_t *[]){&s_MscCsw1, &s_MscCsw2, &s_MscCsw3, &s_MscCsw4})
#else
#error "the max support USB_DEVICE_CONFIG_MSC is 4"
#endif

usb_dev_msc_t g_msc_handle[MYNEWT_VAL(USB_DEVICE_CONFIG_MSC)];

static int
usb_dev_msc_alloc_handle(usb_dev_msc_t **handle)
{
    int i;

    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_MSC); i++) {
        if (!g_msc_handle[i].handle) {
            g_msc_handle[i].mscCbw = MSCCBW_ARRAY[i];
            g_msc_handle[i].mscCsw = MSCCSW_ARRAY[i];
            *handle = &g_msc_handle[i];
            return 0;
        }
    }

    return USB_BUSY;
}

static int
usb_dev_msc_free_handle(usb_dev_msc_t *handle)
{
    assert(handle);
    handle->handle = NULL;
    handle->configurationStruct = NULL;
    handle->configuration = 0;
    handle->alternate = 0;
    return 0;
}

static int
USB_DeviceMscProcessUfiCommand(usb_dev_msc_t *msc)
{
    usb_device_msc_ufi_struct_t *ufi = NULL;
    int err = USB_ERR;

    ufi = &msc->mscUfi;
    if (USB_DEVICE_MSC_REQUEST_SENSE_COMMAND != msc->mscCbw->cbwcb[0]) {
        ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_NO_SENSE;
        ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_NO_SENSE;
        ufi->requestSense.additionalSenseQualifer = USB_DEVICE_MSC_UFI_NO_SENSE;
    }
    ufi->thirteenCase.hostExpectedDataLength = msc->mscCbw->dataTransferLength;
    ufi->thirteenCase.hostExpectedDirection = (uint8_t)(msc->mscCbw->flags >> USB_DEVICE_MSC_CBW_DIRECTION_SHIFT);

    switch (msc->mscCbw->cbwcb[0]) {
    case USB_DEVICE_MSC_INQUIRY_COMMAND:
        err = USB_DeviceMscUfiInquiryCommand(msc);
        break;
    case USB_DEVICE_MSC_READ_10_COMMAND:
    case USB_DEVICE_MSC_READ_12_COMMAND:
        err = USB_DeviceMscUfiReadCommand(msc);
        break;
    case USB_DEVICE_MSC_REQUEST_SENSE_COMMAND:
        err = USB_DeviceMscUfiRequestSenseCommand(msc);
        break;
    case USB_DEVICE_MSC_TEST_UNIT_READY_COMMAND:
        err = USB_DeviceMscUfiTestUnitReadyCommand(msc);
        break;
    case USB_DEVICE_MSC_WRITE_10_COMMAND:
    case USB_DEVICE_MSC_WRITE_12_COMMAND:
        err = USB_DeviceMscUfiWriteCommand(msc);
        break;
    case USB_DEVICE_MSC_PREVENT_ALLOW_MEDIUM_REM_COMMAND:
        err = USB_DeviceMscUfiPreventAllowMediumCommand(msc);
        break;
    case USB_DEVICE_MSC_FORMAT_UNIT_COMMAND:
        err = USB_DeviceMscUfiFormatUnitCommand(msc);
        break;
    case USB_DEVICE_MSC_READ_CAPACITY_10_COMMAND:
    case USB_DEVICE_MSC_READ_CAPACITY_16_COMMAND:
        err = USB_DeviceMscUfiReadCapacityCommand(msc);
        break;
    case USB_DEVICE_MSC_MODE_SENSE_10_COMMAND:
    case USB_DEVICE_MSC_MODE_SENSE_6_COMMAND:
        err = USB_DeviceMscUfiModeSenseCommand(msc);
        break;
    case USB_DEVICE_MSC_MODE_SELECT_10_COMMAND:
    case USB_DEVICE_MSC_MODE_SELECT_6_COMMAND:
        err = USB_DeviceMscUfiModeSelectCommand(msc);
        break;
    case USB_DEVICE_MSC_READ_FORMAT_CAPACITIES_COMMAND:
        err = USB_DeviceMscUfiReadFormatCapacityCommand(msc);
        break;
    case USB_DEVICE_MSC_SEND_DIAGNOSTIC_COMMAND:
        err = USB_DeviceMscUfiSendDiagnosticCommand(msc);
        break;
    case USB_DEVICE_MSC_VERIFY_COMMAND:
        err = USB_DeviceMscUfiVerifyCommand(msc);
        break;
    case USB_DEVICE_MSC_START_STOP_UNIT_COMMAND:
        err = USB_DeviceMscUfiStartStopUnitCommand(msc);
        break;
    default:
        err = USB_DeviceMscUfiUnsupportCommand(msc);
        msc->dataOutFlag = 0;
        msc->dataInFlag = 0;
        msc->outEndpointStallFlag = 0;
        msc->inEndpointStallFlag = 0;
        msc->needOutStallFlag = 0;
        msc->needInStallFlag = 0;
        break;
    }

    if (USB_DEVICE_MSC_UFI_NO_SENSE != ufi->requestSense.senseKey &&
        USB_DEVICE_MSC_COMMAND_PASSED == msc->mscCsw->cswStatus &&
        USB_DEVICE_MSC_REQUEST_SENSE_COMMAND != msc->mscCbw->cbwcb[0]) {
        msc->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;
    }
    return err;
}

int
usb_dev_msc_bulk_in(usb_device_handle handle, usb_dev_ep_cb_msg_t *msg, void *param)
{
    usb_dev_msc_t *msc = (usb_dev_msc_t *)param;
    usb_device_msc_csw_t *csw;
    usb_device_lba_app_struct_t lba;
    int err = USB_ERR;

    if (msg->len == USB_UNINITIALIZED_VAL_32) {
        if (msc->dataInFlag && msc->configurationStruct->cb &&
            (USB_DEVICE_MSC_READ_10_COMMAND == msc->mscCbw->cbwcb[0] ||
             USB_DEVICE_MSC_READ_12_COMMAND == msc->mscCbw->cbwcb[0])) {
            lba.size = 0;
            lba.buffer = msg->buf;
            lba.offset = 0;
            msc->configurationStruct->cb((class_handle_t)msc,
                    kUSB_DeviceMscEventReadResponse, (void *)&lba);
        }
        return err;
    }
    if (msc->transferRemaining >= msg->len) {
        msc->transferRemaining -= msg->len;
    }

    if (msc->needInStallFlag) {
        msc->needInStallFlag = 0;
        msc->inEndpointStallFlag = 1;
        msc->dataInFlag = 0;
        usb_dev_ep_stall(msc->handle, msc->bulkInEndpoint);
        return err;
    }
    if (!msc->dataInFlag && msg->len == USB_DEVICE_MSC_CSW_LENGTH) {
        csw = (usb_device_msc_csw_t *)msg->buf;
    }

    if (msc->dataInFlag) {
        if (msc->configurationStruct->cb) {
            lba.size = msg->len;
            lba.buffer = msg->buf;
            lba.offset = msc->currentOffset;

            if (USB_DEVICE_MSC_READ_10_COMMAND == msc->mscCbw->cbwcb[0] ||
                USB_DEVICE_MSC_READ_12_COMMAND == msc->mscCbw->cbwcb[0]) {
                msc->configurationStruct->cb((class_handle_t)msc,
                        kUSB_DeviceMscEventReadResponse, (void *)&lba);
            }

            if (msc->transferRemaining) {
                msc->currentOffset += msg->len / msc->lengthOfEachLba;
                err = usb_dev_msc_send(msc);
            }
            if (!msc->transferRemaining) {
                msc->dataInFlag = 0;
                msc->cswPrimeFlag = 1;
                usb_device_send_req(msc->handle,
                                    msc->bulkInEndpoint,
                                    (uint8_t *)msc->mscCsw,
                                    USB_DEVICE_MSC_CSW_LENGTH);
            }
        }
    } else if (msg->len == USB_DEVICE_MSC_CSW_LENGTH && csw->signature == USB_DEVICE_MSC_DCSWSIGNATURE) {
        msc->cbwValidFlag = 1;
        msc->cswPrimeFlag = 0;
        (void)usb_device_recv_req(msc->handle, msc->bulkOutEndpoint,
                (uint8_t *)msc->mscCbw, USB_DEVICE_MSC_CBW_LENGTH);
        msc->cbwPrimeFlag = 1;
    }

    return err;
}

int
usb_dev_msc_bulk_out(usb_device_handle handle, usb_dev_ep_cb_msg_t *msg, void *param)
{
    usb_dev_msc_t *msc = (usb_dev_msc_t *)param;
    usb_device_lba_app_struct_t lba;
    int err = 0;

    if (msg->len == USB_UNINITIALIZED_VAL_32) {
        if (msc->dataOutFlag && msc->configurationStruct->cb &&
            (USB_DEVICE_MSC_WRITE_10_COMMAND == msc->mscCbw->cbwcb[0] ||
             USB_DEVICE_MSC_WRITE_12_COMMAND == msc->mscCbw->cbwcb[0])) {
            lba.size = 0;
            lba.buffer = msg->buf;
            lba.offset = 0;
            msc->configurationStruct->cb((class_handle_t)msc,
                    kUSB_DeviceMscEventWriteResponse, (void *)&lba);
        }
        return err;
    }

    if (msc->transferRemaining >= msg->len) {
        msc->transferRemaining -= msg->len;
    }

    if (USB_DEVICE_MSC_MODE_SELECT_10_COMMAND == msc->mscCbw->cbwcb[0] ||
        USB_DEVICE_MSC_MODE_SELECT_6_COMMAND == msc->mscCbw->cbwcb[0]) {
        if (msc->configurationStruct->cb) {
            msc->configurationStruct->cb((class_handle_t)msc,
                    kUSB_DeviceMscEventModeSelectResponse, (void *)NULL);
        }
    }
    if (msc->needOutStallFlag) {
        msc->needOutStallFlag = 0;
        msc->outEndpointStallFlag = 1;
        msc->dataOutFlag = 0;
        msc->cbwPrimeFlag = 0;
        usb_dev_ep_stall(msc->handle, msc->bulkOutEndpoint);
        return err;
    }

    if (msc->dataOutFlag) {
        lba.size = msg->len;
        lba.buffer = msg->buf;
        lba.offset = msc->currentOffset;

        if (msc->configurationStruct->cb) {
            if (USB_DEVICE_MSC_WRITE_10_COMMAND == msc->mscCbw->cbwcb[0] ||
                USB_DEVICE_MSC_WRITE_12_COMMAND == msc->mscCbw->cbwcb[0]) {
                msc->configurationStruct->cb((class_handle_t)msc,
                        kUSB_DeviceMscEventWriteResponse, (void *)&lba);
            }

            if (msc->transferRemaining) {
                msc->currentOffset += (msg->len / msc->lengthOfEachLba);
                err = usb_dev_msc_recv(msc);
            }
        }

        if (!msc->transferRemaining) {
            msc->dataOutFlag = 0;
            usb_device_send_req(msc->handle, msc->bulkInEndpoint,
                    (uint8_t *)msc->mscCsw, USB_DEVICE_MSC_CSW_LENGTH);
            msc->cswPrimeFlag = 1;
        }
    }
    else if (msc->cbwValidFlag && msg->len == USB_DEVICE_MSC_CBW_LENGTH &&
             msc->mscCbw->signature == USB_DEVICE_MSC_DCBWSIGNATURE &&
             !((msc->mscCbw->logicalUnitNumber & 0xF0) || (msc->mscCbw->cbLength & 0xE0)) &&
             (msc->mscCbw->logicalUnitNumber < (msc->logicalUnitNumber + 1)) &&
             ((msc->mscCbw->cbLength >= 0x01) && (msc->mscCbw->cbLength <= 0x10))) {
        msc->cbwPrimeFlag = 0;
        msc->transferRemaining = 0;

        msc->mscCsw->signature = USB_DEVICE_MSC_DCSWSIGNATURE;
        msc->mscCsw->dataResidue = 0;
        msc->mscCsw->tag = msc->mscCbw->tag;

        msc->cbwValidFlag = 0;

        msc->mscCbw->dataTransferLength = USB_U32_TO_LE(msc->mscCbw->dataTransferLength);

        msc->dataOutFlag = (!(msc->mscCbw->flags & USB_DEVICE_MSC_CBW_DIRECTION_BIT) &&
                msc->mscCbw->dataTransferLength) ? 1 : 0;
        msc->dataInFlag = ((msc->mscCbw->flags & USB_DEVICE_MSC_CBW_DIRECTION_BIT) &&
                msc->mscCbw->dataTransferLength) ? 1 : 0;

        if (msc->dataInFlag && msc->inEndpointStallFlag) {
            return USB_ERR;
        }

        err = USB_DeviceMscProcessUfiCommand(msc);
        if (err == USB_INVALID_REQ) {
            if (msc->dataOutFlag) {
                if (!msc->outEndpointStallFlag) {
                    msc->needOutStallFlag = 1;
                }
                msc->dataOutFlag = 0;
            } else if (msc->dataInFlag) {
                if (!msc->inEndpointStallFlag) {
                    msc->needInStallFlag = 1;
                }
                msc->dataInFlag = 0;
            }
            msc->stallStatus = (uint8_t)USB_DEVICE_MSC_STALL_IN_DATA;
        }

        if (!(msc->dataOutFlag || msc->dataInFlag || msc->needInStallFlag)) {
            usb_device_send_req(msc->handle, msc->bulkInEndpoint,
                    (uint8_t *)msc->mscCsw, USB_DEVICE_MSC_CSW_LENGTH);
            msc->cswPrimeFlag = 1;
        }
    } else {
        usb_dev_ep_stall(msc->handle, msc->bulkOutEndpoint);
        usb_dev_ep_stall(msc->handle, msc->bulkInEndpoint);
        msc->cbwValidFlag = 0;
        msc->outEndpointStallFlag = 1;
        msc->inEndpointStallFlag = 1;
        msc->stallStatus = (uint8_t)USB_DEVICE_MSC_STALL_IN_CBW;
        msc->performResetRecover = 1;
    }
    return err;
}

int
usb_dev_msc_eps_init(usb_dev_msc_t *msc)
{
    usb_device_interface_list_t *interfaceList;
    usb_dev_itf_t *interface = NULL;
    int i, j;
    int err = USB_ERR;

    if (!msc->configuration ||
        msc->configuration > msc->configurationStruct->info->configurations) {
        return err;
    }

    if (!msc->configurationStruct->info->interfaceList) {
        return err;
    }
    interfaceList = &msc->configurationStruct->info->interfaceList[msc->configuration - 1];

    for (i = 0; i < interfaceList->count; i++) {
        if (USB_DEVICE_CONFIG_MSC_CLASS_CODE == interfaceList->itfs[i].classCode) {
            for (j = 0; j < interfaceList->itfs[i].count; j++) {
                if (interfaceList->itfs[i].itf[j].alternateSetting == msc->alternate) {
                    interface = &interfaceList->itfs[i].itf[j];
                    break;
                }
            }
            msc->interfaceNumber = interfaceList->itfs[i].itf_num;
            break;
        }
    }
    if (!interface) {
        return err;
    }

    msc->interfaceHandle = interface;
    for (i = 0; i < interface->eps.count; i++) {
        usb_dev_ep_init_t epInitStruct;
        usb_dev_ep_cb_t ep_cb;
        epInitStruct.zlt = 0;
        epInitStruct.endpointAddress = interface->eps.ep[i].ep_addr;
        epInitStruct.maxPacketSize = interface->eps.ep[i].maxPacketSize;
        epInitStruct.transferType = interface->eps.ep[i].transferType;

        if (USB_IN == ((epInitStruct.endpointAddress & USB_DESC_EP_ADDR_DIR_MASK) >>
                       USB_DESC_EP_ADDR_DIR_SHIFT)) {
            msc->bulkInEndpoint = epInitStruct.endpointAddress;
            ep_cb.fn = usb_dev_msc_bulk_in;
        } else {
            msc->bulkOutEndpoint = epInitStruct.endpointAddress;
            ep_cb.fn = usb_dev_msc_bulk_out;
        }
        ep_cb.param = msc;

        err = usb_dev_ep_init(msc->handle, &epInitStruct, &ep_cb);
    }

    msc->dataOutFlag = 0;
    msc->dataInFlag = 0;
    msc->outEndpointStallFlag = 0;
    msc->inEndpointStallFlag = 0;
    msc->needOutStallFlag = 0;
    msc->needInStallFlag = 0;
    msc->cbwValidFlag = 1;
    msc->transferRemaining = 0;
    msc->performResetRecover = 0;
    msc->performResetDoneFlag = 0;
    msc->stallStatus = 0;

    if (msc->cbwPrimeFlag) {
        usb_device_cancel(msc->handle, msc->bulkOutEndpoint);
    }
    usb_device_recv_req(msc->handle, msc->bulkOutEndpoint,
            (uint8_t *)msc->mscCbw, USB_DEVICE_MSC_CBW_LENGTH);
    msc->cbwPrimeFlag = 1;

    return err;
}

int
usb_dev_msc_eps_deinit(usb_dev_msc_t *msc)
{
    int i;
    int err = USB_ERR;

    if (!msc->interfaceHandle) {
        return err;
    }

    for (i = 0; i < msc->interfaceHandle->eps.count; i++) {
        err = usb_dev_ep_deinit(msc->handle, msc->interfaceHandle->eps.ep[i].ep_addr);
    }
    msc->interfaceHandle = NULL;
    return err;
}

int
usb_dev_msc_init(uint8_t controllerId, usb_dev_class_config_t *config, class_handle_t *handle)
{
    usb_dev_msc_t *msc;
    uint32_t implementingDiskDrive = USB_DEVICE_CONFIG_MSC_IMPLEMENTING_DISK_DRIVE;
    usb_device_lba_information_struct_t diskInformation;
    usb_device_msc_ufi_struct_t *ufi = NULL;
    int err = USB_ERR;

    err = usb_dev_msc_alloc_handle(&msc);
    if (err) {
        return err;
    }

    err = usb_device_class_get_handle(controllerId, &msc->handle);

    if (err) {
        usb_dev_msc_free_handle(msc);
        return err;
    }
    if (!msc->handle) {
        usb_dev_msc_free_handle(msc);
        return USB_INVALID_HANDLE;
    }

    msc->configurationStruct = config;
    msc->configuration = 0;
    msc->alternate = 0xff;

    err = msc->configurationStruct->cb(
        (class_handle_t)msc, kUSB_DeviceMscEventGetLbaInformation, (void *)&diskInformation);

    if (((diskInformation.lengthOfEachLba) && (diskInformation.totalLbaNumberSupports)) == 0) {
        err = USB_ERR;
        usb_dev_msc_free_handle(msc);
        return err;
    }

    msc->logicalUnitNumber = diskInformation.logicalUnitNumberSupported;

    ufi = &msc->mscUfi;
    msc->totalLogicalBlockNumber = diskInformation.totalLbaNumberSupports;
    msc->lengthOfEachLba = diskInformation.lengthOfEachLba;
    msc->logicalUnitNumber = diskInformation.logicalUnitNumberSupported - 1;
    msc->bulkInBufferSize = diskInformation.bulkInBufferSize;
    msc->bulkOutBufferSize = diskInformation.bulkOutBufferSize;
    msc->implementingDiskDrive = implementingDiskDrive;

    ufi->requestSense.validErrorCode = USB_DEVICE_MSC_UFI_REQ_SENSE_VALID_ERROR_CODE;
    ufi->requestSense.additionalSenseLength = USB_DEVICE_MSC_UFI_REQ_SENSE_ADDITIONAL_SENSE_LEN;
    ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_NO_SENSE;
    ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_NO_SENSE;
    ufi->requestSense.additionalSenseQualifer = USB_DEVICE_MSC_UFI_NO_SENSE;

    ufi->readCapacity.lastLogicalBlockAddress = USB_U32_TO_BE(msc->totalLogicalBlockNumber - 1);
    ufi->readCapacity.blockSize = USB_U32_TO_BE((uint32_t)msc->lengthOfEachLba);
    ufi->readCapacity16.lastLogicalBlockAddress1 = USB_U32_TO_BE(msc->totalLogicalBlockNumber - 1);
    ufi->readCapacity16.blockSize = USB_U32_TO_BE((uint32_t)msc->lengthOfEachLba);

    msc->cbwPrimeFlag = 0;
    msc->cswPrimeFlag = 0;

    *handle = (class_handle_t)msc;
    return err;
}

int
usb_dev_msc_deinit(class_handle_t handle)
{
    usb_dev_msc_t *msc;
    int err = USB_ERR;

    msc = (usb_dev_msc_t *)handle;

    if (!msc) {
        return USB_INVALID_HANDLE;
    }

    err = usb_dev_msc_eps_deinit(msc);
    usb_dev_msc_free_handle(msc);
    return err;
}

int
usb_dev_msc_event(void *handle, uint32_t event, void *param)
{
    int err = USB_ERR;
    usb_dev_msc_t *msc;
    uint16_t interfaceAlternate;
    uint8_t *temp8;
    uint8_t alternate;

    if (!param || !handle) {
        return USB_INVALID_HANDLE;
    }

    msc = (usb_dev_msc_t *)handle;
    switch (event) {
    case kUSB_DeviceClassEventDeviceReset:
        msc->configuration = 0;
        break;
    case kUSB_DeviceClassEventSetConfiguration:
        temp8 = (uint8_t *)param;
        if (!msc->configurationStruct) {
            break;
        }
        if (*temp8 == msc->configuration) {
            break;
        }

        if (msc->configuration) {
            err = usb_dev_msc_eps_deinit(msc);
        }
        msc->configuration = *temp8;
        msc->alternate = 0;
        err = usb_dev_msc_eps_init(msc);
        break;
    case kUSB_DeviceClassEventSetInterface:
        if (!msc->configurationStruct) {
            break;
        }
        interfaceAlternate = *((uint16_t *)param);
        alternate = (uint8_t)(interfaceAlternate & 0xFF);

        if (msc->interfaceNumber != ((uint8_t)(interfaceAlternate >> 8))) {
            break;
        }
        if (alternate == msc->alternate) {
            break;
        }

        err = usb_dev_msc_eps_deinit(msc);
        msc->alternate = alternate;

        err = usb_dev_msc_eps_init(msc);
        break;

    case kUSB_DeviceClassEventSetEndpointHalt:
        if (!msc->configurationStruct || !msc->interfaceHandle) {
            break;
        }
        temp8 = (uint8_t *)param;

        if (msc->inEndpointStallFlag == 0 && *temp8 == msc->bulkInEndpoint) {
            err = usb_dev_ep_stall(msc->handle, *temp8);
            msc->inEndpointStallFlag = 1;
        }
        if (msc->outEndpointStallFlag == 0 && *temp8 == msc->bulkOutEndpoint) {
            err = usb_dev_ep_stall(msc->handle, *temp8);
            msc->outEndpointStallFlag = 1;
        }
        break;
    case kUSB_DeviceClassEventClearEndpointHalt:
        if (!msc->configurationStruct || !msc->interfaceHandle ||
            msc->performResetRecover == 1) {
            break;
        }
        temp8 = ((uint8_t *)param);
        if (msc->inEndpointStallFlag == 1 && *temp8 == msc->bulkInEndpoint) {
            err = usb_dev_ep_unstall(msc->handle, *temp8);
            msc->inEndpointStallFlag = 0;
        }
        if (msc->outEndpointStallFlag == 1 && *temp8 == msc->bulkOutEndpoint) {
            err = usb_dev_ep_unstall(msc->handle, *temp8);
            msc->outEndpointStallFlag = 0;
        }
        if (((msc->stallStatus == USB_DEVICE_MSC_STALL_IN_CSW) ||
             (msc->stallStatus == USB_DEVICE_MSC_STALL_IN_DATA)) &&
            (msc->performResetDoneFlag != 1))  {
            if (msc->cswPrimeFlag == 1) {
                usb_device_cancel(msc->handle, msc->bulkInEndpoint);
            }
            usb_device_send_req(msc->handle, msc->bulkInEndpoint,
                    (uint8_t *)msc->mscCsw, USB_DEVICE_MSC_CSW_LENGTH);
            msc->cswPrimeFlag = 1;
            msc->stallStatus = 0;
        }
        if ((msc->performResetDoneFlag == 1) && (msc->inEndpointStallFlag == 0) &&
            (msc->outEndpointStallFlag == 0)) {
            msc->performResetDoneFlag = 0;
            if (msc->cbwPrimeFlag) {
                usb_device_cancel(msc->handle, msc->bulkOutEndpoint);
            }
            usb_device_recv_req(msc->handle, msc->bulkOutEndpoint,
                    (uint8_t *)msc->mscCbw, USB_DEVICE_MSC_CBW_LENGTH);
            msc->cbwPrimeFlag = 1;
            msc->stallStatus = 0;
        }
        break;
    case kUSB_DeviceClassEventClassRequest:
        if (param) {
            usb_dev_ctrl_req_t *req = (usb_dev_ctrl_req_t *)param;

            if ((req->setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) !=
                USB_REQ_TYPE_RECIPIENT_INTERFACE) {
                break;
            }

            switch (req->setup->bRequest) {
            case USB_DEVICE_MSC_GET_MAX_LUN:
                if (req->setup->wIndex == msc->interfaceNumber &&
                    !req->setup->wValue && req->setup->wLength == 1 &&
                    (req->setup->bmRequestType & USB_REQ_TYPE_DIR_MASK) == USB_REQ_TYPE_DIR_IN) {
                    req->buf = &msc->logicalUnitNumber;
                    req->len = req->setup->wLength;
                } else {
                    err = USB_INVALID_REQ;
                }
                break;

            case USB_DEVICE_MSC_BULK_ONLY_MASS_STORAGE_RESET:
                if (req->setup->wIndex == msc->interfaceNumber &&
                    !req->setup->wValue && !req->setup->wLength &&
                    (req->setup->bmRequestType & USB_REQ_TYPE_DIR_MASK) == USB_REQ_TYPE_DIR_OUT) {
                    err = usb_dev_msc_eps_deinit(msc);
                    err = usb_dev_msc_eps_init(msc);
                    msc->outEndpointStallFlag = 1;
                    msc->inEndpointStallFlag = 1;
                    msc->performResetRecover = 0;
                    msc->performResetDoneFlag = 1;
                } else {
                    err = USB_INVALID_REQ;
                }
                break;

            default:
                break;
            }
        }
        break;

    default:
        break;
    }

    return err;
}

int
usb_dev_msc_send(usb_dev_msc_t *msc)
{
    usb_device_lba_app_struct_t lba;
    int err = 0;

    lba.offset = msc->currentOffset;
    lba.size = (msc->bulkInBufferSize > USB_DEVICE_MSC_MAX_SEND_TRANSFER_LENGTH) ?
                   USB_DEVICE_MSC_MAX_SEND_TRANSFER_LENGTH :
                   msc->bulkInBufferSize;
    lba.size = (msc->transferRemaining > lba.size) ?
        lba.size : msc->transferRemaining;

    lba.buffer = NULL;
    msc->configurationStruct->cb((class_handle_t)msc, kUSB_DeviceMscEventReadRequest, &lba);

    if (msc->currentOffset < msc->totalLogicalBlockNumber) {
        err = usb_device_send_req(msc->handle, msc->bulkInEndpoint, lba.buffer, lba.size);
    } else {
        msc->needInStallFlag = 0;
        msc->inEndpointStallFlag = 1;
        msc->dataInFlag = 0;
        msc->stallStatus = (uint8_t)USB_DEVICE_MSC_STALL_IN_DATA;
        usb_dev_ep_stall(msc->handle, msc->bulkInEndpoint);
    }
    return err;
}

int
usb_dev_msc_recv(usb_dev_msc_t *msc)
{
    usb_device_lba_app_struct_t lba;
    int err = 0;

    lba.offset = msc->currentOffset;
    lba.size = (msc->bulkOutBufferSize > USB_DEVICE_MSC_MAX_RECV_TRANSFER_LENGTH) ?
        USB_DEVICE_MSC_MAX_RECV_TRANSFER_LENGTH : msc->bulkOutBufferSize;
    lba.size = (msc->transferRemaining > lba.size) ?
        lba.size : msc->transferRemaining;

    lba.buffer = NULL;
    msc->configurationStruct->cb((class_handle_t)msc,
            kUSB_DeviceMscEventWriteRequest, &lba);

    if (msc->currentOffset < msc->totalLogicalBlockNumber) {
        err = usb_device_recv_req(msc->handle, msc->bulkOutEndpoint, lba.buffer, lba.size);
    } else {
        msc->needOutStallFlag = 0;
        msc->outEndpointStallFlag = 1;
        msc->dataOutFlag = 0;
        msc->stallStatus = (uint8_t)USB_DEVICE_MSC_STALL_IN_DATA;
        usb_dev_ep_stall(msc->handle, msc->bulkOutEndpoint);
    }
    return err;
}

int
USB_DeviceMscLbaTransfer(usb_dev_msc_t *msc, uint8_t dir,
                         usb_lba_transfer_information_struct_t *lba_info_ptr)
{
    int err = 0;

    msc->transferRemaining = lba_info_ptr->transferNumber * msc->lengthOfEachLba;
    msc->currentOffset = lba_info_ptr->startingLogicalBlockAddress;

    if (dir == USB_IN) {
        err = usb_dev_msc_send(msc);
    } else {
        err = usb_dev_msc_recv(msc);
    }

    return err;
}
#endif
