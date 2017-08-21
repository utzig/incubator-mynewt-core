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

#if MYNEWT_VAL(USB_DEVICE_CONFIG_MSC)
#include <msc/msc.h>
#include <msc/msc_ufi.h>

usb_status_t USB_DeviceMscLbaTransfer(usb_dev_msc_t *mscHandle,
                         uint8_t dir,
                         usb_lba_transfer_information_struct_t *lba_info_ptr);
usb_status_t
USB_DeviceMscUfiThirteenCasesCheck(usb_dev_msc_t *mscHandle)
{
    usb_status_t error = kStatus_USB_Success;
    usb_device_msc_ufi_struct_t *ufi;
    usb_device_msc_thirteen_case_struct_t *mscCheckEvent;

    mscCheckEvent = (usb_device_msc_thirteen_case_struct_t *)&mscHandle->mscUfi.thirteenCase;
    ufi = &mscHandle->mscUfi;
    if (mscCheckEvent->hostExpectedDataLength == 0) {
        mscHandle->mscCsw->dataResidue = 0;
        if (mscCheckEvent->deviceExpectedDataLength == 0) {
            mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_PASSED;
        } else {
            mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_PHASE_ERROR;
        }
    } else if (mscCheckEvent->hostExpectedDirection)  {
        if (mscCheckEvent->deviceExpectedDataLength == 0) {
            mscHandle->mscCsw->dataResidue =
                mscCheckEvent->hostExpectedDataLength - mscCheckEvent->deviceExpectedDataLength;
            error = usb_device_send_req(mscHandle->handle, mscHandle->bulkInEndpoint, mscCheckEvent->buffer,
                                        mscCheckEvent->deviceExpectedDataLength);

            if (kStatus_USB_Success == error) {
                mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_PASSED;
            } else {
                mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;
                ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_MEDIUM_ERROR;
                ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_UNRECOVERED_READ_ERROR;
            }
            error = kStatus_USB_InvalidRequest;
        } else if (mscCheckEvent->deviceExpectedDirection) {
            if (mscCheckEvent->hostExpectedDataLength > mscCheckEvent->deviceExpectedDataLength) {
                mscHandle->mscCsw->dataResidue =
                    mscCheckEvent->hostExpectedDataLength - mscCheckEvent->deviceExpectedDataLength;

                if (ufi->thirteenCase.lbaSendRecvSelect == 1) {
                    error = USB_DeviceMscLbaTransfer(mscHandle, USB_IN, &mscCheckEvent->lbaInformation);
                } else {
                    error = usb_device_send_req(mscHandle->handle, mscHandle->bulkInEndpoint, mscCheckEvent->buffer,
                                                mscCheckEvent->deviceExpectedDataLength);
                }

                if (kStatus_USB_Success == error) {
                    mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_PASSED;
                } else {
                    mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;
                    ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_MEDIUM_ERROR;
                    ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_UNRECOVERED_READ_ERROR;
                }
                error = kStatus_USB_InvalidRequest;
            } else if (mscCheckEvent->hostExpectedDataLength == mscCheckEvent->deviceExpectedDataLength) {
                mscHandle->mscCsw->dataResidue = 0;
                if (ufi->thirteenCase.lbaSendRecvSelect == 1) {
                    error = USB_DeviceMscLbaTransfer(mscHandle, USB_IN, &mscCheckEvent->lbaInformation);
                } else {
                    error = usb_device_send_req(mscHandle->handle, mscHandle->bulkInEndpoint, mscCheckEvent->buffer,
                                                mscCheckEvent->deviceExpectedDataLength);
                }

                if (kStatus_USB_Success == error) {
                    mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_PASSED;
                } else {
                    mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;
                    ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_MEDIUM_ERROR;
                    ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_UNRECOVERED_READ_ERROR;
                }
            } else {
                mscHandle->mscCsw->dataResidue = 0;

                if (ufi->thirteenCase.lbaSendRecvSelect == 1) {
                    mscCheckEvent->lbaInformation.transferNumber =
                        mscCheckEvent->hostExpectedDataLength / mscHandle->lengthOfEachLba;
                    mscHandle->mscCsw->dataResidue =
                        mscCheckEvent->hostExpectedDataLength -
                        mscCheckEvent->lbaInformation.transferNumber * mscHandle->lengthOfEachLba;
                    error = USB_DeviceMscLbaTransfer(mscHandle, USB_IN, &mscCheckEvent->lbaInformation);
                } else {
                    error = usb_device_send_req(mscHandle->handle,
                                                mscHandle->bulkInEndpoint,
                                                mscCheckEvent->buffer,
                                                mscCheckEvent->hostExpectedDataLength);
                }

                if (kStatus_USB_Success == error) {
                    if (ufi->thirteenCase.lbaSendRecvSelect == 1) {
                        mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_PHASE_ERROR;
                    } else {
                        mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_PASSED;
                    }
                } else {
                    mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;
                    ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_MEDIUM_ERROR;
                    ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_UNRECOVERED_READ_ERROR;
                }
            }
        } else {
            mscHandle->mscCsw->dataResidue = mscCheckEvent->hostExpectedDataLength;
            error = usb_device_send_req(mscHandle->handle,
                                        mscHandle->bulkInEndpoint,
                                        mscCheckEvent->buffer,
                                        0);
            mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_PHASE_ERROR;
            error = kStatus_USB_InvalidRequest;
        }
    } else {
        if (0 == mscCheckEvent->deviceExpectedDataLength) {
            usb_dev_ep_stall(mscHandle->handle, mscHandle->bulkOutEndpoint);
            mscHandle->mscCsw->dataResidue = mscCheckEvent->hostExpectedDataLength;
            mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;
            mscHandle->outEndpointStallFlag = 1;
            error = kStatus_USB_InvalidRequest;
        } else if (mscCheckEvent->deviceExpectedDirection) {
            usb_dev_ep_stall(mscHandle->handle, mscHandle->bulkOutEndpoint);
            mscHandle->mscCsw->dataResidue = mscCheckEvent->hostExpectedDataLength;
            mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_PHASE_ERROR;
            mscHandle->outEndpointStallFlag = 1;
            error = kStatus_USB_InvalidRequest;
        } else {
            if (mscCheckEvent->hostExpectedDataLength > mscCheckEvent->deviceExpectedDataLength) {
                mscHandle->mscCsw->dataResidue =
                    mscCheckEvent->hostExpectedDataLength - mscCheckEvent->deviceExpectedDataLength;

                if (ufi->thirteenCase.lbaSendRecvSelect == 1) {
                    error = USB_DeviceMscLbaTransfer(mscHandle, USB_OUT, &mscCheckEvent->lbaInformation);
                } else {
                    error = usb_device_recv_req(mscHandle->handle, mscHandle->bulkOutEndpoint, mscCheckEvent->buffer,
                                                mscCheckEvent->deviceExpectedDataLength);
                }

                if (kStatus_USB_Success == error) {
                    mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_PASSED;
                } else {
                    mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;
                    ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_MEDIUM_ERROR;
                    ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_WRITE_FAULT;
                }
                error = kStatus_USB_InvalidRequest;
            } else if (mscCheckEvent->hostExpectedDataLength == mscCheckEvent->deviceExpectedDataLength) {
                mscHandle->mscCsw->dataResidue = 0;
                if (ufi->thirteenCase.lbaSendRecvSelect == 1) {
                    error = USB_DeviceMscLbaTransfer(mscHandle, USB_OUT, &mscCheckEvent->lbaInformation);
                } else {
                    error = usb_device_recv_req(mscHandle->handle, mscHandle->bulkOutEndpoint, mscCheckEvent->buffer,
                                                mscCheckEvent->deviceExpectedDataLength);
                }
                if (kStatus_USB_Success == error) {
                    mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_PASSED;
                } else {
                    mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;
                    ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_MEDIUM_ERROR;
                    ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_WRITE_FAULT;
                }
            } else {
                mscHandle->mscCsw->dataResidue = 0;
                if (ufi->thirteenCase.lbaSendRecvSelect == 1) {
                    mscCheckEvent->lbaInformation.transferNumber =
                        mscCheckEvent->hostExpectedDataLength / mscHandle->lengthOfEachLba;
                    mscHandle->mscCsw->dataResidue =
                        mscCheckEvent->hostExpectedDataLength -
                        mscCheckEvent->lbaInformation.transferNumber * mscHandle->lengthOfEachLba;
                    error = USB_DeviceMscLbaTransfer(mscHandle, USB_OUT, &mscCheckEvent->lbaInformation);
                } else {
                    error = usb_device_recv_req(mscHandle->handle,
                                                mscHandle->bulkOutEndpoint,
                                                mscCheckEvent->buffer,
                                                mscCheckEvent->hostExpectedDataLength);
                }

                if (kStatus_USB_Success == error) {
                    if (ufi->thirteenCase.lbaSendRecvSelect == 1) {
                        mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_PHASE_ERROR;
                    } else {
                        mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_PASSED;
                    }
                } else {
                    ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_MEDIUM_ERROR;
                    ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_WRITE_FAULT;
                }
            }
        }
    }
    return error;
}

usb_status_t
USB_DeviceMscUfiRequestSenseCommand(usb_dev_msc_t *mscHandle)
{
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    ufi->thirteenCase.deviceExpectedDataLength = USB_DEVICE_MSC_UFI_REQ_SENSE_DATA_LENGTH;
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.buffer = (uint8_t *)&ufi->requestSense;
    ufi->thirteenCase.lbaSendRecvSelect = 0;
    return USB_DeviceMscUfiThirteenCasesCheck(mscHandle);
}

usb_status_t
USB_DeviceMscUfiInquiryCommand(usb_dev_msc_t *mscHandle)
{
    usb_device_ufi_app_struct_t temp;
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    temp.requestSense = &ufi->requestSense;
    temp.cbwcb = &mscHandle->mscCbw->cbwcb[0];
    temp.size = 0U;
    temp.buffer = NULL;

    if (mscHandle->configurationStruct->classCallback) {
        mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle, kUSB_DeviceMscEventInquiry,
                                                      (void *)&temp);
    }
    ufi->thirteenCase.deviceExpectedDataLength = temp.size;
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.buffer = temp.buffer;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    return USB_DeviceMscUfiThirteenCasesCheck(mscHandle);
}

usb_status_t
USB_DeviceMscUfiReadCommand(usb_dev_msc_t *mscHandle)
{
    usb_device_msc_ufi_struct_t *ufi = NULL;
    uint32_t logicalBlockAddress = 0;
    uint32_t lbaTransferLength = 0;

    ufi = &mscHandle->mscUfi;

    logicalBlockAddress = ((uint32_t)mscHandle->mscCbw->cbwcb[2] << 24);
    logicalBlockAddress |= ((uint32_t)mscHandle->mscCbw->cbwcb[3] << 16);
    logicalBlockAddress |= ((uint32_t)mscHandle->mscCbw->cbwcb[4] << 8);
    logicalBlockAddress |= ((uint32_t)mscHandle->mscCbw->cbwcb[5]);

    if (mscHandle->mscCbw->cbwcb[0] == USB_DEVICE_MSC_READ_10_COMMAND) {
        lbaTransferLength = (uint16_t)((uint16_t)mscHandle->mscCbw->cbwcb[7] << 8);
        lbaTransferLength |= (uint16_t)mscHandle->mscCbw->cbwcb[8];
    } else if (mscHandle->mscCbw->cbwcb[0] == USB_DEVICE_MSC_READ_12_COMMAND) {
        lbaTransferLength = ((uint32_t)mscHandle->mscCbw->cbwcb[6] << 24);
        lbaTransferLength |= ((uint32_t)mscHandle->mscCbw->cbwcb[7] << 16);
        lbaTransferLength |= ((uint32_t)mscHandle->mscCbw->cbwcb[8] << 8);
        lbaTransferLength |= ((uint32_t)mscHandle->mscCbw->cbwcb[9]);
    }

    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.deviceExpectedDataLength = mscHandle->lengthOfEachLba * lbaTransferLength;
    ufi->thirteenCase.buffer = NULL;

    ufi->thirteenCase.lbaSendRecvSelect = 1;
    ufi->thirteenCase.lbaInformation.startingLogicalBlockAddress = logicalBlockAddress;
    ufi->thirteenCase.lbaInformation.transferNumber = lbaTransferLength;

    return USB_DeviceMscUfiThirteenCasesCheck(mscHandle);
}

usb_status_t
USB_DeviceMscUfiWriteCommand(usb_dev_msc_t *mscHandle)
{
    uint32_t logicalBlockAddress = 0;
    uint32_t lbaTransferLength = 0;
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    logicalBlockAddress = ((uint32_t)mscHandle->mscCbw->cbwcb[2] << 24);
    logicalBlockAddress |= ((uint32_t)mscHandle->mscCbw->cbwcb[3] << 16);
    logicalBlockAddress |= ((uint32_t)mscHandle->mscCbw->cbwcb[4] << 8);
    logicalBlockAddress |= ((uint32_t)mscHandle->mscCbw->cbwcb[5]);

    if (mscHandle->mscCbw->cbwcb[0] == USB_DEVICE_MSC_WRITE_10_COMMAND) {
        lbaTransferLength = (uint16_t)((uint16_t)mscHandle->mscCbw->cbwcb[7] << 8);
        lbaTransferLength |= (uint16_t)mscHandle->mscCbw->cbwcb[8];
    } else if (mscHandle->mscCbw->cbwcb[0] == USB_DEVICE_MSC_WRITE_12_COMMAND) {
        lbaTransferLength = ((uint32_t)mscHandle->mscCbw->cbwcb[6] << 24);
        lbaTransferLength |= ((uint32_t)mscHandle->mscCbw->cbwcb[7] << 16);
        lbaTransferLength |= ((uint32_t)mscHandle->mscCbw->cbwcb[8] << 8);
        lbaTransferLength |= ((uint32_t)mscHandle->mscCbw->cbwcb[9]);
    }

    ufi->thirteenCase.deviceExpectedDirection = USB_OUT;
    ufi->thirteenCase.deviceExpectedDataLength = mscHandle->lengthOfEachLba * lbaTransferLength;
    ufi->thirteenCase.buffer = NULL;

    ufi->thirteenCase.lbaSendRecvSelect = 1;
    ufi->thirteenCase.lbaInformation.startingLogicalBlockAddress = logicalBlockAddress;
    ufi->thirteenCase.lbaInformation.transferNumber = lbaTransferLength;

    return USB_DeviceMscUfiThirteenCasesCheck(mscHandle);
}

usb_status_t
USB_DeviceMscUfiTestUnitReadyCommand(usb_dev_msc_t *mscHandle)
{
    usb_device_msc_ufi_struct_t *ufi = NULL;
    usb_status_t error;
    usb_device_ufi_app_struct_t temp;

    ufi = &mscHandle->mscUfi;
    temp.requestSense = &ufi->requestSense;
    temp.cbwcb = &mscHandle->mscCbw->cbwcb[0];

    ufi->thirteenCase.deviceExpectedDataLength = 0;
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.buffer = NULL;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    error = USB_DeviceMscUfiThirteenCasesCheck(mscHandle);

    if (mscHandle->configurationStruct->classCallback) {
        mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle, kUSB_DeviceMscEventTestUnitReady,
                                                      (void *)&temp);
    }
    return error;
}

usb_status_t
USB_DeviceMscUfiVerifyCommand(usb_dev_msc_t *mscHandle)
{
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    ufi->thirteenCase.deviceExpectedDataLength = 0;
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.buffer = NULL;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    return USB_DeviceMscUfiThirteenCasesCheck(mscHandle);
}

usb_status_t
USB_DeviceMscUfiModeSenseCommand(usb_dev_msc_t *mscHandle)
{
    usb_device_ufi_app_struct_t temp;
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    temp.requestSense = &ufi->requestSense;
    temp.cbwcb = &mscHandle->mscCbw->cbwcb[0];
    temp.size = 0U;
    temp.buffer = NULL;

    if (mscHandle->configurationStruct->classCallback) {
        mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle, kUSB_DeviceMscEventModeSense,
                                                      (void *)&temp);
    }
    ufi->thirteenCase.deviceExpectedDataLength = temp.size;
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.buffer = temp.buffer;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    return USB_DeviceMscUfiThirteenCasesCheck(mscHandle);
}

usb_status_t
USB_DeviceMscUfiModeSelectCommand(usb_dev_msc_t *mscHandle)
{
    usb_status_t error = kStatus_USB_TransferFailed;
    usb_device_ufi_app_struct_t temp;
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    temp.requestSense = &ufi->requestSense;
    temp.cbwcb = &mscHandle->mscCbw->cbwcb[0];
    temp.buffer = NULL;
    temp.size = 0;
    if (mscHandle->configurationStruct->classCallback) {
        mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle, kUSB_DeviceMscEventInquiry,
                                                      (void *)&temp);
    }

    ufi->thirteenCase.deviceExpectedDataLength = temp.size;
    ufi->thirteenCase.deviceExpectedDirection = USB_OUT;
    ufi->thirteenCase.buffer = temp.buffer;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    error = USB_DeviceMscUfiThirteenCasesCheck(mscHandle);

    if (mscHandle->mscCbw->cbwcb[1] & 0x01) {
        ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_ILLEGAL_REQUEST;
        ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_INVALID_FIELD_IN_COMMAND_PKT;
    }
    return error;
}

usb_status_t
USB_DeviceMscUfiReadCapacityCommand(usb_dev_msc_t *mscHandle)
{
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    if (mscHandle->mscCbw->cbwcb[0] == USB_DEVICE_MSC_READ_CAPACITY_10_COMMAND) {
        ufi->thirteenCase.deviceExpectedDataLength = USB_DEVICE_MSC_UFI_READ_CAPACITY_DATA_LENGTH;
        ufi->thirteenCase.buffer = (uint8_t *)&(ufi->readCapacity);
    } else {
        ufi->thirteenCase.deviceExpectedDataLength = USB_DEVICE_MSC_UFI_READ_CAPACITY16_DATA_LENGTH;
        ufi->thirteenCase.buffer = (uint8_t *)&(ufi->readCapacity16);
    }
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    return USB_DeviceMscUfiThirteenCasesCheck(mscHandle);
}

usb_status_t
USB_DeviceMscUfiReadFormatCapacityCommand(usb_dev_msc_t *mscHandle)
{
    usb_device_msc_ufi_struct_t *ufi = NULL;
    usb_device_current_max_capacity_descriptor_struct_t current_max_head;
    usb_device_formattable_capacity_descriptor_struct_t formattable_capacity_head;
    usb_device_capacity_list_header_struct_t capacityListHead = {{0x00, 0x00, 0x00}, 0x00};
    uint32_t response_size;
    uint16_t allocation_length;
    uint8_t num_formattable_cap_desc;
    uint8_t descriptor_code;
    uint8_t count = 0;
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t *ptr;

    ufi = &mscHandle->mscUfi;
    allocation_length = (uint16_t)((uint8_t)(mscHandle->mscCbw->cbwcb[7] << 8) | mscHandle->mscCbw->cbwcb[8]);
    /*reference ufi command spec table-33 Descriptor Code definition*/
    num_formattable_cap_desc = (uint8_t)(ufi->formattedDisk ? (mscHandle->implementingDiskDrive ? 0x02 : 0x03) : 0x00);

    formattable_capacity_head.blockNumber = mscHandle->totalLogicalBlockNumber;
    formattable_capacity_head.blockLength = mscHandle->lengthOfEachLba;

    descriptor_code =
        (uint8_t)(ufi->formattedDisk ? USB_DEVICE_MSC_UFI_FORMATTED_MEDIA : USB_DEVICE_MSC_UFI_UNFORMATTED_MEDIA);
    capacityListHead.capacityListLength = num_formattable_cap_desc * 8;
    current_max_head.blockNumber = mscHandle->totalLogicalBlockNumber;
    current_max_head.descriptorCodeBlockLength = (uint8_t)(descriptor_code << 24) | mscHandle->lengthOfEachLba;

    response_size = sizeof(usb_device_capacity_list_header_struct_t) +
                    sizeof(usb_device_current_max_capacity_descriptor_struct_t) +
                    sizeof(usb_device_formattable_capacity_descriptor_struct_t) * num_formattable_cap_desc;

    if (response_size > allocation_length) {
        response_size = allocation_length;
    }
    if (sizeof(ufi->formatCapacityData) < response_size) {
#if (defined(_DEBUG) && _DEBUG)
        usb_echo("format_capacity_response_data buff size less than need\n");
#endif
    }

    ptr = (uint8_t *)&capacityListHead;
    for (count = 0; count < sizeof(capacityListHead); count++) {
        ufi->formatCapacityData[count] = ptr[i++];
    }
    ptr = (uint8_t *)&current_max_head;
    i = 0;
    for (; i < sizeof(current_max_head); count++) {
        ufi->formatCapacityData[count] = ptr[i++];
    }

    if (ufi->formattedDisk) {
        for (i = 0; i < num_formattable_cap_desc; i++) {
            ptr = (uint8_t *)&formattable_capacity_head;

            for (; count < sizeof(formattable_capacity_head); count++) {
                ufi->formatCapacityData[count] = ptr[j++];
            }
        }
    }

    ufi->thirteenCase.deviceExpectedDataLength = response_size;
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.buffer = ufi->formatCapacityData;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    return USB_DeviceMscUfiThirteenCasesCheck(mscHandle);
}

usb_status_t
USB_DeviceMscUfiFormatUnitCommand(usb_dev_msc_t *mscHandle)
{
    usb_status_t error;
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    ufi->thirteenCase.deviceExpectedDataLength = 0;
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.buffer = NULL;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    error = USB_DeviceMscUfiThirteenCasesCheck(mscHandle);

    if (mscHandle->mscCsw->cswStatus != USB_DEVICE_MSC_PHASE_ERROR) {
        if ((mscHandle->mscCbw->cbwcb[1] & 0x1f) == 0x17) {
            mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_PASSED;
        } else {
            mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;
            ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_ILLEGAL_REQUEST;
            ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_INVALID_FIELD_IN_COMMAND_PKT;
        }
    }
    return error;
}

usb_status_t
USB_DeviceMscUfiPreventAllowMediumCommand(usb_dev_msc_t *mscHandle)
{
    usb_status_t error;
    usb_device_ufi_app_struct_t temp;
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    temp.requestSense = &ufi->requestSense;
    temp.cbwcb = &mscHandle->mscCbw->cbwcb[0];

    ufi->thirteenCase.deviceExpectedDataLength = 0;
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.buffer = NULL;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    error = USB_DeviceMscUfiThirteenCasesCheck(mscHandle);

    if (mscHandle->configurationStruct->classCallback) {
        mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle, kUSB_DeviceMscEventRemovalRequest,
                                                      (void *)&temp);
    }

    return error;
}

usb_status_t
USB_DeviceMscUfiSendDiagnosticCommand(usb_dev_msc_t *mscHandle)
{
    usb_status_t error;
    usb_device_ufi_app_struct_t temp;
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    temp.requestSense = &ufi->requestSense;
    temp.cbwcb = &mscHandle->mscCbw->cbwcb[0];

    ufi->thirteenCase.deviceExpectedDataLength = 0;
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.buffer = NULL;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    error = USB_DeviceMscUfiThirteenCasesCheck(mscHandle);
    if (mscHandle->configurationStruct->classCallback != NULL) {
        mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle, kUSB_DeviceMscEventRemovalRequest,
                                                      (void *)&temp);
    }
    return error;
}

usb_status_t USB_DeviceMscUfiStartStopUnitCommand(usb_dev_msc_t *mscHandle)
{
    usb_status_t error;
    usb_device_ufi_app_struct_t temp;
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    temp.requestSense = &ufi->requestSense;
    temp.cbwcb = &mscHandle->mscCbw->cbwcb[0];

    ufi->thirteenCase.deviceExpectedDataLength = 0;
    ufi->thirteenCase.deviceExpectedDirection = USB_IN;
    ufi->thirteenCase.buffer = NULL;
    ufi->thirteenCase.lbaSendRecvSelect = 0;

    error = USB_DeviceMscUfiThirteenCasesCheck(mscHandle);

    if (mscHandle->mscCsw->cswStatus != USB_DEVICE_MSC_PHASE_ERROR) {
        if (mscHandle->configurationStruct->classCallback) {
            mscHandle->configurationStruct->classCallback((class_handle_t)mscHandle, kUSB_DeviceMscEventStopEjectMedia,
                                                          (void *)&temp);
        }
    }

    return error;
}

usb_status_t
USB_DeviceMscUfiUnsupportCommand(usb_dev_msc_t *mscHandle)
{
    usb_device_msc_ufi_struct_t *ufi = &mscHandle->mscUfi;

    mscHandle->mscCsw->dataResidue = 0;
    mscHandle->mscCsw->cswStatus = USB_DEVICE_MSC_COMMAND_FAILED;

    ufi->requestSense.senseKey = USB_DEVICE_MSC_UFI_ILLEGAL_REQUEST;
    ufi->requestSense.additionalSenseCode = USB_DEVICE_MSC_UFI_INVALID_COMMAND_OPCODE;
    ufi->requestSense.additionalSenseQualifer = USB_DEVICE_MSC_UFI_NO_SENSE;

    return kStatus_USB_Success;
}

#endif
