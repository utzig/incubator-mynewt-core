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
#ifndef _USB_CLASS_CDC_H_
#define _USB_CLASS_CDC_H_

#include <os/os.h>

#define CDC_COMM_CLASS                                    0x02
#define CDC_DATA_CLASS                                    0x0A

#define USB_CDC_DIRECT_LINE_CONTROL_MODEL                 0x01
#define USB_CDC_ABSTRACT_CONTROL_MODEL                    0x02
#define USB_CDC_TELEPHONE_CONTROL_MODEL                   0x03
#define USB_CDC_MULTI_CHANNEL_CONTROL_MODEL               0x04
#define USB_CDC_CAPI_CONTROL_MOPDEL                       0x05
#define USB_CDC_ETHERNET_NETWORKING_CONTROL_MODEL         0x06
#define USB_CDC_ATM_NETWORKING_CONTROL_MODEL              0x07
#define USB_CDC_WIRELESS_HANDSET_CONTROL_MODEL            0x08
#define USB_CDC_DEVICE_MANAGEMENT                         0x09
#define USB_CDC_MOBILE_DIRECT_LINE_MODEL                  0x0A
#define USB_CDC_OBEX                                      0x0B
#define USB_CDC_ETHERNET_EMULATION_MODEL                  0x0C

#define USB_CDC_NO_CLASS_SPECIFIC_PROTOCOL                0x00
#define USB_CDC_AT_250_PROTOCOL                           0x01
#define USB_CDC_AT_PCCA_101_PROTOCOL                      0x02
#define USB_CDC_AT_PCCA_101_ANNEX_O                       0x03
#define USB_CDC_AT_GSM_7_07                               0x04
#define USB_CDC_AT_3GPP_27_007                            0x05
#define USB_CDC_AT_TIA_CDMA                               0x06
#define USB_CDC_ETHERNET_EMULATION_PROTOCOL               0x07
#define USB_CDC_EXTERNAL_PROTOCOL                         0xFE
#define USB_CDC_VENDOR_SPECIFIC                           0xFF

#define USB_CDC_PYHSICAL_INTERFACE_PROTOCOL               0x30
#define USB_CDC_HDLC_PROTOCOL                             0x31
#define USB_CDC_TRANSPARENT_PROTOCOL                      0x32
#define USB_CDC_MANAGEMENT_PROTOCOL                       0x50
#define USB_CDC_DATA_LINK_Q931_PROTOCOL                   0x51
#define USB_CDC_DATA_LINK_Q921_PROTOCOL                   0x52
#define USB_CDC_DATA_COMPRESSION_V42BIS                   0x90
#define USB_CDC_EURO_ISDN_PROTOCOL                        0x91
#define USB_CDC_RATE_ADAPTION_ISDN_V24                    0x92
#define USB_CDC_CAPI_COMMANDS                             0x93
#define USB_CDC_HOST_BASED_DRIVER                         0xFD
#define USB_CDC_UNIT_FUNCTIONAL                           0xFE

#define USB_CDC_HEADER_FUNC_DESC                          0x00
#define USB_CDC_CALL_MANAGEMENT_FUNC_DESC                 0x01
#define USB_CDC_ABSTRACT_CONTROL_FUNC_DESC                0x02
#define USB_CDC_DIRECT_LINE_FUNC_DESC                     0x03
#define USB_CDC_TELEPHONE_RINGER_FUNC_DESC                0x04
#define USB_CDC_TELEPHONE_REPORT_FUNC_DESC                0x05
#define USB_CDC_UNION_FUNC_DESC                           0x06
#define USB_CDC_COUNTRY_SELECT_FUNC_DESC                  0x07
#define USB_CDC_TELEPHONE_MODES_FUNC_DESC                 0x08
#define USB_CDC_TERMINAL_FUNC_DESC                        0x09
#define USB_CDC_NETWORK_CHANNEL_FUNC_DESC                 0x0A
#define USB_CDC_PROTOCOL_UNIT_FUNC_DESC                   0x0B
#define USB_CDC_EXTENSION_UNIT_FUNC_DESC                  0x0C
#define USB_CDC_MULTI_CHANNEL_FUNC_DESC                   0x0D
#define USB_CDC_CAPI_CONTROL_FUNC_DESC                    0x0E
#define USB_CDC_ETHERNET_NETWORKING_FUNC_DESC             0x0F
#define USB_CDC_ATM_NETWORKING_FUNC_DESC                  0x10
#define USB_CDC_WIRELESS_CONTROL_FUNC_DESC                0x11
#define USB_CDC_MOBILE_DIRECT_LINE_FUNC_DESC              0x12
#define USB_CDC_MDLM_DETAIL_FUNC_DESC                     0x13
#define USB_CDC_DEVICE_MANAGEMENT_FUNC_DESC               0x14
#define USB_CDC_OBEX_FUNC_DESC                            0x15
#define USB_CDC_COMMAND_SET_FUNC_DESC                     0x16
#define USB_CDC_COMMAND_SET_DETAIL_FUNC_DESC              0x17
#define USB_CDC_TELEPHONE_CONTROL_FUNC_DESC               0x18
#define USB_CDC_OBEX_SERVICE_ID_FUNC_DESC                 0x19

#define USB_DEVICE_CONFIG_CDC_ACM_MAX_INSTANCE                  1
#define USB_DEVICE_CONFIG_CDC_COMM_CLASS_CODE                   0x02
#define USB_DEVICE_CONFIG_CDC_DATA_CLASS_CODE                   0x0A

#define USB_DEVICE_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND        0x00
#define USB_DEVICE_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE        0x01
#define USB_DEVICE_CDC_REQUEST_SET_COMM_FEATURE                 0x02
#define USB_DEVICE_CDC_REQUEST_GET_COMM_FEATURE                 0x03
#define USB_DEVICE_CDC_REQUEST_CLEAR_COMM_FEATURE               0x04
#define USB_DEVICE_CDC_REQUEST_SET_AUX_LINE_STATE               0x10
#define USB_DEVICE_CDC_REQUEST_SET_HOOK_STATE                   0x11
#define USB_DEVICE_CDC_REQUEST_PULSE_SETUP                      0x12
#define USB_DEVICE_CDC_REQUEST_SEND_PULSE                       0x13
#define USB_DEVICE_CDC_REQUEST_SET_PULSE_TIME                   0x14
#define USB_DEVICE_CDC_REQUEST_RING_AUX_JACK                    0x15
#define USB_DEVICE_CDC_REQUEST_SET_LINE_CODING                  0x20
#define USB_DEVICE_CDC_REQUEST_GET_LINE_CODING                  0x21
#define USB_DEVICE_CDC_REQUEST_SET_CONTROL_LINE_STATE           0x22
#define USB_DEVICE_CDC_REQUEST_SEND_BREAK                       0x23
#define USB_DEVICE_CDC_REQUEST_SET_RINGER_PARAMS                0x30
#define USB_DEVICE_CDC_REQUEST_GET_RINGER_PARAMS                0x31
#define USB_DEVICE_CDC_REQUEST_SET_OPERATION_PARAM              0x32
#define USB_DEVICE_CDC_REQUEST_GET_OPERATION_PARAM              0x33
#define USB_DEVICE_CDC_REQUEST_SET_LINE_PARAMS                  0x34
#define USB_DEVICE_CDC_REQUEST_GET_LINE_PARAMS                  0x35
#define USB_DEVICE_CDC_REQUEST_DIAL_DIGITS                      0x36
#define USB_DEVICE_CDC_REQUEST_SET_UNIT_PARAMETER               0x37
#define USB_DEVICE_CDC_REQUEST_GET_UNIT_PARAMETER               0x38
#define USB_DEVICE_CDC_REQUEST_CLEAR_UNIT_PARAMETER             0x39
#define USB_DEVICE_CDC_REQUEST_SET_ETHERNET_MULTICAST_FILTERS   0x40
#define USB_DEVICE_CDC_REQUEST_SET_ETHERNET_POW_PATTER_FILTER   0x41
#define USB_DEVICE_CDC_REQUEST_GET_ETHERNET_POW_PATTER_FILTER   0x42
#define USB_DEVICE_CDC_REQUEST_SET_ETHERNET_PACKET_FILTER       0x43
#define USB_DEVICE_CDC_REQUEST_GET_ETHERNET_STATISTIC           0x44
#define USB_DEVICE_CDC_REQUEST_SET_ATM_DATA_FORMAT              0x50
#define USB_DEVICE_CDC_REQUEST_GET_ATM_DEVICE_STATISTICS        0x51
#define USB_DEVICE_CDC_REQUEST_SET_ATM_DEFAULT_VC               0x52
#define USB_DEVICE_CDC_REQUEST_GET_ATM_VC_STATISTICS            0x53
#define USB_DEVICE_CDC_REQUEST_MDLM_SPECIFIC_REQUESTS_MASK      0x7F

#define USB_DEVICE_CDC_NOTIF_NETWORK_CONNECTION                 0x00
#define USB_DEVICE_CDC_NOTIF_RESPONSE_AVAIL                     0x01
#define USB_DEVICE_CDC_NOTIF_AUX_JACK_HOOK_STATE                0x08
#define USB_DEVICE_CDC_NOTIF_RING_DETECT                        0x09
#define USB_DEVICE_CDC_NOTIF_SERIAL_STATE                       0x20
#define USB_DEVICE_CDC_NOTIF_CALL_STATE_CHANGE                  0x28
#define USB_DEVICE_CDC_NOTIF_LINE_STATE_CHANGE                  0x29
#define USB_DEVICE_CDC_NOTIF_CONNECTION_SPEED_CHANGE            0x2A

#define USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE                   0x01
#define USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING                  0x02

#define USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION    0x02
#define USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE          0x01
#define USB_DEVICE_CDC_UART_STATE_RX_CARRIER                    0x01
#define USB_DEVICE_CDC_UART_STATE_TX_CARRIER                    0x02
#define USB_DEVICE_CDC_UART_STATE_BREAK                         0x04
#define USB_DEVICE_CDC_UART_STATE_RING_SIGNAL                   0x08
#define USB_DEVICE_CDC_UART_STATE_FRAMING                       0x10
#define USB_DEVICE_CDC_UART_STATE_PARITY                        0x20
#define USB_DEVICE_CDC_UART_STATE_OVERRUN                       0x40

typedef enum
{
    CDC_EVT_SEND_RESPONSE = 0x01,
    CDC_EVT_RECV_RESPONSE,
    CDC_EVT_SERIAL_STATE_NOTIF,
    CDC_EVT_SEND_ENCAPSULATED_COMMAND,
    CDC_EVT_GET_ENCAPSULATED_RESPONSE,
    CDC_EVT_SET_COMM_FEATURE,
    CDC_EVT_GET_COMM_FEATURE,
    CDC_EVT_CLEAR_COMM_FEATURE,
    CDC_EVT_GET_LINE_CODING,
    CDC_EVT_SET_LINE_CODING,
    CDC_EVT_SET_CONTROL_LINE_STATE,
    CDC_EVT_SEND_BREAK,
} usb_dev_cdc_event_t;

typedef struct
{
    uint8_t  **buffer;
    uint32_t *length;
    uint16_t interfaceIndex;
    uint16_t setupValue;
    uint8_t  isSetup;
} usb_dev_cdc_req_param_t;

typedef struct
{
    struct os_mutex mutex;
    uint8_t ep;
    bool is_busy;
} usb_dev_cdc_pipe_t;

typedef struct
{
    usb_device_handle handle;
    usb_dev_class_config_t *config;
    usb_dev_itf_t *comm_itf;
    usb_dev_itf_t *data_itf;
    usb_dev_cdc_pipe_t bulk_in;
    usb_dev_cdc_pipe_t bulk_out;
    usb_dev_cdc_pipe_t intr_in;
    uint8_t config_num;
    uint8_t itf_num;
    uint8_t alternate;
    uint8_t has_sent;
} usb_dev_cdc_t;

extern uint8_t g_cdc_device_descriptor[DESC_LEN_DEVICE];

#if defined(__cplusplus)
extern "C" {
#endif

int usb_dev_cdc_init(uint8_t ctrl_id, usb_dev_class_config_t *config, class_handle_t *handle);
int usb_dev_cdc_deinit(class_handle_t handle);
int usb_dev_cdc_event(void *handle, uint32_t event, void *param);
int usb_dev_cdc_send(class_handle_t handle, uint8_t ep, uint8_t *buf, uint32_t len);
int usb_dev_cdc_recv(class_handle_t handle, uint8_t ep, uint8_t *buf, uint32_t len);

#if defined(__cplusplus)
}
#endif

#endif /* _USB_CLASS_CDC_H_ */
