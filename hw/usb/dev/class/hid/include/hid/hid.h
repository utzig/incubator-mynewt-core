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

#ifndef __USB_DEVICE_HID_H__
#define __USB_DEVICE_HID_H__

#include <stdint.h>
#include <usb/usb.h>

#define USB_DEVICE_CONFIG_HID_CLASS_CODE                   0x03

#define USB_DEVICE_HID_REQUEST_GET_REPORT                  0x01
#define USB_DEVICE_HID_REQUEST_GET_REPORT_TYPE_INPUT       0x01
#define USB_DEVICE_HID_REQUEST_GET_REPORT_TYPE_OUPUT       0x02
#define USB_DEVICE_HID_REQUEST_GET_REPORT_TYPE_FEATURE     0x03

#define USB_DEVICE_HID_REQUEST_GET_IDLE                    0x02

#define USB_DEVICE_HID_REQUEST_GET_PROTOCOL                0x03

#define USB_DEVICE_HID_REQUEST_SET_REPORT                  0x09

#define USB_DEVICE_HID_REQUEST_SET_IDLE                    0x0A

#define USB_DEVICE_HID_REQUEST_SET_PROTOCOL                0x0B

/*! @brief Available common EVENT types in HID class callback */
typedef enum _usb_device_hid_event
{
    kUSB_DeviceHidEventSendResponse = 0x01U, /*!< Send data completed */
    kUSB_DeviceHidEventRecvResponse,         /*!< Data received */
    kUSB_DeviceHidEventGetReport,            /*!< Get report request */
    kUSB_DeviceHidEventGetIdle,              /*!< Get idle request */
    kUSB_DeviceHidEventGetProtocol,          /*!< Get protocol request */
    kUSB_DeviceHidEventSetReport,            /*!< Set report request */
    kUSB_DeviceHidEventSetIdle,              /*!< Set idle request */
    kUSB_DeviceHidEventSetProtocol,          /*!< Set protocol request */
    kUSB_DeviceHidEventRequestReportBuffer,  /*!< Get buffer to save the data of the set report request. */
} usb_device_hid_event_t;

/*!
 * @brief The device HID GET/SET report structure.
 *
 * This structure is used to pass data when the event type is kUSB_DeviceHidEventGetReport,
 * kUSB_DeviceHidEventSetReport, and kUSB_DeviceHidEventRequestReportBuffer.
 * 1. kUSB_DeviceHidEventGetReport
 *    The structure is used to save the report buffer and report length got from the application.
 *    The reportBuffer is the report data buffer address filled by the application.
 *    The reportLength is the report length.
 *    The reportType is the requested report type.
 *    The reportId is the requested report ID.
 *
 * 2. kUSB_DeviceHidEventSetReport
 *    The structure is used to pass the report data received from the host to the application.
 *    The reportBuffer is buffer address of the report data received from the host.
 *    The reportLength is the report data length.
 *    The reportType is the requested report type.
 *    The reportId is the requested report ID.
 *
 * 3. kUSB_DeviceHidEventRequestReportBuffer
 *    The structure is used to get the buffer to save the report data sent by the host.
 *    The reportBuffer is buffer address to receive to report data. It is filled by the application.
 *    The reportLength is the requested report data buffer length.
 *    The reportType is the requested report type.
 *    The reportId is the requested report ID.
 */
typedef struct
{
    uint8_t *reportBuffer;
    uint32_t reportLength;
    uint8_t reportType;
    uint8_t reportId;
} usb_device_hid_report_struct_t;

typedef struct
{
    usb_device_handle handle;
    usb_dev_class_config_t *config;
    usb_dev_itf_t *interfaceHandle;
    uint8_t configuration;
    uint8_t interfaceNumber;
    uint8_t alternate;
    uint8_t idleRate;
    uint8_t protocol;
    uint8_t interruptInPipeBusy;
    uint8_t interruptOutPipeBusy;
} usb_dev_hid_t;

#if defined(__cplusplus)
extern "C" {
#endif

int usb_dev_hid_init(uint8_t controllerId, usb_dev_class_config_t *config,
        class_handle_t *handle);
int usb_dev_hid_deinit(class_handle_t handle);
int usb_dev_hid_event(void *handle, uint32_t event, void *param);
int usb_dev_hid_send(class_handle_t handle, uint8_t ep, uint8_t *buf, uint32_t len);
int usb_dev_hid_recv(class_handle_t handle, uint8_t ep, uint8_t *buf, uint32_t len);

#if defined(__cplusplus)
}
#endif

#endif /* __USB_DEVICE_HID_H__ */
