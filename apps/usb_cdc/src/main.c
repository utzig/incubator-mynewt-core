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

#include <syscfg/syscfg.h>
#include "sysinit/sysinit.h"
#include "sysflash/sysflash.h"
#include <os/os.h>
#include <bsp/bsp.h>
#include <hal/hal_gpio.h>
#include <hal/hal_flash.h>
#include <console/console.h>
#include <log/log.h>
#include <stats/stats.h>
#include "flash_map/flash_map.h"
#include <hal/hal_system.h>
#if MYNEWT_VAL(SPLIT_LOADER)
#include "split/split.h"
#endif
#include <assert.h>
#include <string.h>
#include <os/os_time.h>

#include <stdio.h>

#include <usb/usb.h>
#include <dev/class.h>
#include <dev/dev.h>
#include <cdc/cdc.h>

//FIXME: required for USB_DATA_ALIGNMENT
#include <hal_usb/hal_usb.h>

#ifdef ARCH_sim
#include <mcu/mcu_sim.h>
#endif

#define MAX_CBMEM_BUF 600

#define BLINKER_PRIO                       8
#define BLINKER_STACK_SIZE                 OS_STACK_ALIGN(192)
static struct os_task blinker_task;

#define USB_APP_TASK_PRIO                  7
#define USB_APP_STACK_SIZE                 OS_STACK_ALIGN(2048)
static struct os_task usb_app_task;

#define USB_DEV_TASK_PRIO                  6
#define USB_DEV_STACK_SIZE                 OS_STACK_ALIGN(2048)
static struct os_task usb_dev_task;

//static struct log my_log;

#if 0
STATS_SECT_START(gpio_stats)
STATS_SECT_ENTRY(toggles)
STATS_SECT_END

//static STATS_SECT_DECL(gpio_stats) g_stats_gpio_toggle;

static STATS_NAME_START(gpio_stats)
STATS_NAME(gpio_stats, toggles)
STATS_NAME_END(gpio_stats)
#endif

#define CONTROLLER_ID kUSB_ControllerKhci0

#define USB_DEVICE_SPECIFIC_BCD_VERSION                   0x0200
#define USB_DEVICE_DEMO_BCD_VERSION                       0x0101

#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL           67
#define USB_DESCRIPTOR_LENGTH_CDC_HEADER_FUNC             5
#define USB_DESCRIPTOR_LENGTH_CDC_CALL_MANAG              5
#define USB_DESCRIPTOR_LENGTH_CDC_ABSTRACT                4
#define USB_DESCRIPTOR_LENGTH_CDC_UNION_FUNC              5

#define USB_DEVICE_CONFIGURATION_COUNT                    1
#define USB_DEVICE_STRING_COUNT                           3
#define USB_DEVICE_LANGUAGE_COUNT                         1

#define USB_CDC_VCOM_CONFIGURE_INDEX                      1

#define USB_CDC_VCOM_ENDPOINT_CIC_COUNT                   1
#define USB_CDC_VCOM_ENDPOINT_DIC_COUNT                   2
#define USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT                1
#define USB_CDC_VCOM_BULK_IN_ENDPOINT                     2
#define USB_CDC_VCOM_BULK_OUT_ENDPOINT                    3
#define USB_CDC_VCOM_INTERFACE_COUNT                      2
#define USB_CDC_VCOM_COMM_INTERFACE_INDEX                 0
#define USB_CDC_VCOM_DATA_INTERFACE_INDEX                 1

#define HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE              16
#define FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE              16
#define HS_CDC_VCOM_INTERRUPT_IN_INTERVAL                 0x07
#define FS_CDC_VCOM_INTERRUPT_IN_INTERVAL                 0x08
#define HS_CDC_VCOM_BULK_IN_PACKET_SIZE                   512
#define FS_CDC_VCOM_BULK_IN_PACKET_SIZE                   64
#define HS_CDC_VCOM_BULK_OUT_PACKET_SIZE                  512
#define FS_CDC_VCOM_BULK_OUT_PACKET_SIZE                  64
#define DATA_BUFF_SIZE                                    FS_CDC_VCOM_BULK_OUT_PACKET_SIZE

#define USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE              0x24
#define USB_DESCRIPTOR_TYPE_CDC_CS_ENDPOINT               0x25

#define USB_DEVICE_CLASS                                  0x02
#define USB_DEVICE_SUBCLASS                               0x00
#define USB_DEVICE_PROTOCOL                               0x00

#define USB_DEVICE_MAX_POWER                              0x32

#define USB_CDC_VCOM_CIC_CLASS                            CDC_COMM_CLASS
#define USB_CDC_VCOM_CIC_SUBCLASS                         USB_CDC_ABSTRACT_CONTROL_MODEL
#define USB_CDC_VCOM_CIC_PROTOCOL                         USB_CDC_NO_CLASS_SPECIFIC_PROTOCOL

#define USB_CDC_VCOM_DIC_CLASS                            CDC_DATA_CLASS
#define USB_CDC_VCOM_DIC_SUBCLASS                         0x00
#define USB_CDC_VCOM_DIC_PROTOCOL                         USB_CDC_NO_CLASS_SPECIFIC_PROTOCOL


#define LINE_CODING_SIZE                                  0x07
#define LINE_CODING_DTERATE                               115200
#define LINE_CODING_CHARFORMAT                            0x00
#define LINE_CODING_PARITYTYPE                            0x00
#define LINE_CODING_DATABITS                              0x08

#define COMM_FEATURE_DATA_SIZE                            0x02
#define STATUS_ABSTRACT_STATE                             0x0000
#define COUNTRY_SETTING                                   0x0000

#define NOTIF_PACKET_SIZE                                 8
#define UART_BITMAP_SIZE                                  2
#define NOTIF_REQUEST_TYPE                                0xA1

#define USB_CDC_VCOM_INTERFACE_COUNT 2

typedef uint32_t mytype_t;

typedef struct _usb_cdc_vcom_t
{
    usb_device_handle deviceHandle;
    class_handle_t cdcAcmHandle;
    volatile uint8_t attach;
    //xTaskHandle deviceTaskHandle;
    //xTaskHandle applicationTaskHandle;
    uint8_t speed;
    volatile uint8_t startTransactions;
    uint8_t currentConfiguration;
    uint8_t currentInterfaceAlternateSetting[USB_CDC_VCOM_INTERFACE_COUNT];
} usb_cdc_vcom_t;

/* Define the infomation relates to abstract control model */
typedef struct _usb_cdc_acm_info
{
    uint8_t serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE]; /* Serial state buffer of the CDC device to notify the
                                                                     serial state to host. */
    bool dtePresent;
    uint16_t breakDuration;   /* Length of time in milliseconds of the break signal */
    uint8_t dteStatus;        /* Status of data terminal equipment                  */
    uint8_t currentInterface;
    uint16_t uartState;
} usb_cdc_acm_info_t;

static usb_status_t usb_device_cdc_cb(class_handle_t handle, uint32_t event,
        void *param);
static usb_status_t usb_device_cb(usb_device_handle handle, uint32_t event,
        void *param);

extern usb_dev_ep_t g_UsbDeviceCdcVcomDicEndpoints[];
usb_cdc_vcom_t s_cdc_vcom;

static uint8_t s_lineCoding[LINE_CODING_SIZE] = {
    (LINE_CODING_DTERATE >> 0)  & 0xFF,
    (LINE_CODING_DTERATE >> 8)  & 0xFF,
    (LINE_CODING_DTERATE >> 16) & 0xFF,
    (LINE_CODING_DTERATE >> 24) & 0xFF,
    LINE_CODING_CHARFORMAT,
    LINE_CODING_PARITYTYPE,
    LINE_CODING_DATABITS
};

static uint8_t s_abstractState[COMM_FEATURE_DATA_SIZE] = {
    (STATUS_ABSTRACT_STATE >> 0) & 0xFF,
    (STATUS_ABSTRACT_STATE >> 8) & 0xFF,
};

static uint8_t s_countryCode[COMM_FEATURE_DATA_SIZE] = {
    (COUNTRY_SETTING >> 0) & 0xFF,
    (COUNTRY_SETTING >> 8) & 0xFF,
};

USB_DATA_ALIGNMENT static usb_cdc_acm_info_t s_usbCdcAcmInfo = {
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 0, 0, 0, 0, 0
};

/* Data buffer for receiving and sending*/
static USB_DATA_ALIGNMENT uint8_t s_currRecvBuf[DATA_BUFF_SIZE];
static USB_DATA_ALIGNMENT uint8_t s_currSendBuf[DATA_BUFF_SIZE];
static uint32_t s_recvSize = 0;
static uint32_t s_sendSize = 0;

usb_dev_ep_t g_UsbDeviceCdcVcomCicEndpoints[USB_CDC_VCOM_ENDPOINT_CIC_COUNT] = {
    {
        USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT | (USB_IN << 7),
        USB_ENDPOINT_INTERRUPT,
        FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE,
    },
};

usb_dev_ep_t g_UsbDeviceCdcVcomDicEndpoints[USB_CDC_VCOM_ENDPOINT_DIC_COUNT] = {
    {
        USB_CDC_VCOM_BULK_IN_ENDPOINT | (USB_IN << 7),
        USB_ENDPOINT_BULK,
        FS_CDC_VCOM_BULK_IN_PACKET_SIZE,
    },
    {
        USB_CDC_VCOM_BULK_OUT_ENDPOINT | (USB_OUT << 7),
        USB_ENDPOINT_BULK,
        FS_CDC_VCOM_BULK_OUT_PACKET_SIZE,
    }
};

usb_dev_itf_t g_UsbDeviceCdcVcomCommunicationInterface[] = {{
    0,
    {
        USB_CDC_VCOM_ENDPOINT_CIC_COUNT,
        g_UsbDeviceCdcVcomCicEndpoints,
    },
}};

usb_dev_itf_t g_UsbDeviceCdcVcomDataInterface[] = {{
    0,
    {
        USB_CDC_VCOM_ENDPOINT_DIC_COUNT,
        g_UsbDeviceCdcVcomDicEndpoints,
    },
}};

usb_dev_itfs_t g_UsbDeviceCdcVcomInterfaces[USB_CDC_VCOM_INTERFACE_COUNT] = {
    {
        USB_CDC_VCOM_CIC_CLASS,
        USB_CDC_VCOM_CIC_SUBCLASS,
        USB_CDC_VCOM_CIC_PROTOCOL,
        USB_CDC_VCOM_COMM_INTERFACE_INDEX,
        g_UsbDeviceCdcVcomCommunicationInterface,
        sizeof(g_UsbDeviceCdcVcomCommunicationInterface) / sizeof(usb_dev_itfs_t)
    },
    {
        USB_CDC_VCOM_DIC_CLASS,
        USB_CDC_VCOM_DIC_SUBCLASS,
        USB_CDC_VCOM_DIC_PROTOCOL,
        USB_CDC_VCOM_DATA_INTERFACE_INDEX,
        g_UsbDeviceCdcVcomDataInterface,
        sizeof(g_UsbDeviceCdcVcomDataInterface) / sizeof(usb_dev_itfs_t)
    },
};

#if 0
usb_device_interface_list_t g_UsbDeviceCdcVcomInterfaceList[USB_DEVICE_CONFIGURATION_COUNT] = {
    {
        USB_CDC_VCOM_INTERFACE_COUNT,
        g_UsbDeviceCdcVcomInterfaces,
    },
};
#endif

static usb_device_interface_list_t g_UsbDeviceCdcVcomInterfaceList[USB_DEVICE_CONFIGURATION_COUNT] = {
    {
        USB_CDC_VCOM_INTERFACE_COUNT,
        g_UsbDeviceCdcVcomInterfaces,
    },
};

static usb_dev_class_t g_UsbDeviceCdcVcomConfig = {
    g_UsbDeviceCdcVcomInterfaceList,
    kUSB_DeviceClassTypeCdc,
    USB_DEVICE_CONFIGURATION_COUNT,
};

static usb_dev_class_config_t cdc_config[] = {
    {
        usb_device_cdc_cb,
        0,
        &g_UsbDeviceCdcVcomConfig,
    },
};

/* USB device class configuraion information */
static usb_dev_class_configs_t s_cdcAcmConfigList = {
    cdc_config,
    usb_device_cb,
    1,
};

uint8_t g_UsbDeviceDescriptor[USB_DESCRIPTOR_LENGTH_DEVICE] = {
    /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_LENGTH_DEVICE,
    /* DEVICE Descriptor Type */
    USB_DESCRIPTOR_TYPE_DEVICE,
    /* USB Specification Release Number in Binary-Coded Decimal (i.e., 2.10 is 210H). */
    USB_SHORT_GET_LOW(USB_DEVICE_SPECIFIC_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_SPECIFIC_BCD_VERSION),
    /* Class code (assigned by the USB-IF). */
    USB_DEVICE_CLASS,
    /* Subclass code (assigned by the USB-IF). */
    USB_DEVICE_SUBCLASS,
    /* Protocol code (assigned by the USB-IF). */
    USB_DEVICE_PROTOCOL,
    /* Maximum packet size for endpoint zero (only 8, 16, 32, or 64 are valid) */
    USB_CONTROL_MAX_PACKET_SIZE,
    /* Vendor ID (assigned by the USB-IF) */
    0xC9, 0x1F,
    /* Product ID (assigned by the manufacturer) */
    0x94, 0x00,
    /* Device release number in binary-coded decimal */
    USB_SHORT_GET_LOW(USB_DEVICE_DEMO_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_DEMO_BCD_VERSION),
    /* Index of string descriptor describing manufacturer */
    0x01,
    /* Index of string descriptor describing product */
    0x02,
    /* Index of string descriptor describing the device's serial number */
    0x00,
    /* Number of possible configurations */
    USB_DEVICE_CONFIGURATION_COUNT,
};

uint8_t g_UsbDeviceConfigurationDescriptor[USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL] = {
    /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_LENGTH_CONFIGURE,
    /* CONFIGURATION Descriptor Type */
    USB_DESCRIPTOR_TYPE_CONFIGURE,
    /* Total length of data returned for this configuration. */
    USB_SHORT_GET_LOW(USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL),
    USB_SHORT_GET_HIGH(USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL),
    /* Number of interfaces supported by this configuration */
    USB_CDC_VCOM_INTERFACE_COUNT,
    /* Value to use as an argument to the SetConfiguration() request to select this configuration */
    USB_CDC_VCOM_CONFIGURE_INDEX,
    /* Index of string descriptor describing this configuration */
    0,
    /* Configuration characteristics D7: Reserved (set to one) D6: Self-powered D5: Remote Wakeup D4...0: Reserved
       (reset to zero) */
    (USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK) |
        (MYNEWT_VAL(USB_DEVICE_CONFIG_SELF_POWER) << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_SHIFT) |
        (MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP) << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_SHIFT),
    /* Maximum power consumption of the USB * device from the bus in this specific * configuration when the device is
       fully * operational. Expressed in 2 mA units *  (i.e., 50 = 100 mA).  */
    USB_DEVICE_MAX_POWER,

    /* Communication Interface Descriptor */
    USB_DESCRIPTOR_LENGTH_INTERFACE,
    USB_DESCRIPTOR_TYPE_INTERFACE,
    USB_CDC_VCOM_COMM_INTERFACE_INDEX,
    0x00,
    USB_CDC_VCOM_ENDPOINT_CIC_COUNT,
    USB_CDC_VCOM_CIC_CLASS,
    USB_CDC_VCOM_CIC_SUBCLASS,
    USB_CDC_VCOM_CIC_PROTOCOL,
    0x00, /* Interface Description String Index*/

    /* CDC Class-Specific descriptor */
    USB_DESCRIPTOR_LENGTH_CDC_HEADER_FUNC, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE,  /* CS_INTERFACE Descriptor Type */
    USB_CDC_HEADER_FUNC_DESC, 0x10,
    0x01, /* USB Class Definitions for Communications the Communication specification version 1.10 */

    USB_DESCRIPTOR_LENGTH_CDC_CALL_MANAG, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE, /* CS_INTERFACE Descriptor Type */
    USB_CDC_CALL_MANAGEMENT_FUNC_DESC,
    0x01, /*Bit 0: Whether device handle call management itself 1, Bit 1: Whether device can send/receive call
             management information over a Data Class Interface 0 */
    0x01, /* Indicates multiplexed commands are handled via data interface */

    USB_DESCRIPTOR_LENGTH_CDC_ABSTRACT,   /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE, /* CS_INTERFACE Descriptor Type */
    USB_CDC_ABSTRACT_CONTROL_FUNC_DESC,
    0x06, /* Bit 0: Whether device supports the request combination of Set_Comm_Feature, Clear_Comm_Feature, and
             Get_Comm_Feature 0, Bit 1: Whether device supports the request combination of Set_Line_Coding,
             Set_Control_Line_State, Get_Line_Coding, and the notification Serial_State 1, Bit ...  */

    USB_DESCRIPTOR_LENGTH_CDC_UNION_FUNC, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE, /* CS_INTERFACE Descriptor Type */
    USB_CDC_UNION_FUNC_DESC,
    0x00,                                 /* The interface number of the Communications or Data Class interface  */
    0x01,                                 /* Interface number of subordinate interface in the Union  */

    /*Notification Endpoint descriptor */
    USB_DESCRIPTOR_LENGTH_ENDPOINT,
    USB_DESCRIPTOR_TYPE_ENDPOINT,
    USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT | (USB_IN << 7),
    USB_ENDPOINT_INTERRUPT,
    USB_SHORT_GET_LOW(FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE),
    FS_CDC_VCOM_INTERRUPT_IN_INTERVAL,

    /* Data Interface Descriptor */
    USB_DESCRIPTOR_LENGTH_INTERFACE,
    USB_DESCRIPTOR_TYPE_INTERFACE,
    USB_CDC_VCOM_DATA_INTERFACE_INDEX,
    0x00,
    USB_CDC_VCOM_ENDPOINT_DIC_COUNT,
    USB_CDC_VCOM_DIC_CLASS,
    USB_CDC_VCOM_DIC_SUBCLASS,
    USB_CDC_VCOM_DIC_PROTOCOL,
    0x00, /* Interface Description String Index*/

    /*Bulk IN Endpoint descriptor */
    USB_DESCRIPTOR_LENGTH_ENDPOINT,
    USB_DESCRIPTOR_TYPE_ENDPOINT,
    USB_CDC_VCOM_BULK_IN_ENDPOINT | (USB_IN << 7),
    USB_ENDPOINT_BULK,
    USB_SHORT_GET_LOW(FS_CDC_VCOM_BULK_IN_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_CDC_VCOM_BULK_IN_PACKET_SIZE),
    0x00, /* The polling interval value is every 0 Frames */

    /*Bulk OUT Endpoint descriptor */
    USB_DESCRIPTOR_LENGTH_ENDPOINT,
    USB_DESCRIPTOR_TYPE_ENDPOINT,
    USB_CDC_VCOM_BULK_OUT_ENDPOINT | (USB_OUT << 7),
    USB_ENDPOINT_BULK,
    USB_SHORT_GET_LOW(FS_CDC_VCOM_BULK_OUT_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_CDC_VCOM_BULK_OUT_PACKET_SIZE),
    0x00, /* The polling interval value is every 0 Frames */
};

/* FIXME: move string descriptors to cdc (or device?) and allow
 * overwriting strings with syscfg
 */
uint8_t g_UsbDeviceString0[4] = {
    sizeof(g_UsbDeviceString0),
    USB_DESCRIPTOR_TYPE_STRING,
    0x09,
    0x04
};

uint8_t g_UsbDeviceString1[38] = {
    sizeof(g_UsbDeviceString1),
    USB_DESCRIPTOR_TYPE_STRING,
    'N', 0, 'X', 0, 'P', 0, ' ', 0, 'S', 0, 'E', 0, 'M', 0, 'I', 0,
    'C', 0, 'O', 0, 'N', 0, 'D', 0, 'U', 0, 'C', 0, 'T', 0, 'O', 0,
    'R', 0, 'S', 0,
};

uint8_t g_UsbDeviceString2[42] = {
    sizeof(g_UsbDeviceString2),
    USB_DESCRIPTOR_TYPE_STRING,
    'M', 0, 'C', 0, 'U', 0, ' ', 0, 'V', 0, 'I', 0, 'R', 0, 'T', 0,
    'U', 0, 'A', 0, 'L', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0,
    'D', 0, 'E', 0, 'M', 0, 'O', 0,
};

uint8_t *g_UsbDeviceStringDescriptorArray[USB_DEVICE_STRING_COUNT] = {
    g_UsbDeviceString0,
    g_UsbDeviceString1,
    g_UsbDeviceString2
};

uint32_t g_UsbDeviceStringDescriptorLength[USB_DEVICE_STRING_COUNT] = {
    sizeof(g_UsbDeviceString0),
    sizeof(g_UsbDeviceString1),
    sizeof(g_UsbDeviceString2)
};

usb_language_t g_UsbDeviceLanguage[USB_DEVICE_LANGUAGE_COUNT] = {{
    g_UsbDeviceStringDescriptorArray,
    g_UsbDeviceStringDescriptorLength,
    (uint16_t) 0x0409,
}};

usb_language_list_t g_UsbDeviceLanguageList = {
    g_UsbDeviceString0,
    sizeof(g_UsbDeviceString0),
    g_UsbDeviceLanguage,
    USB_DEVICE_LANGUAGE_COUNT,
};

static usb_status_t
USB_DeviceGetDeviceDescriptor(usb_device_handle handle,
        usb_device_get_device_descriptor_struct_t *deviceDescriptor)
{
    deviceDescriptor->buffer = g_UsbDeviceDescriptor;
    deviceDescriptor->length = USB_DESCRIPTOR_LENGTH_DEVICE;
    return kStatus_USB_Success;
}

static usb_status_t
USB_DeviceGetConfigurationDescriptor(usb_device_handle handle,
        usb_device_get_configuration_descriptor_struct_t *configurationDescriptor)
{
    if (USB_CDC_VCOM_CONFIGURE_INDEX > configurationDescriptor->configuration) {
        configurationDescriptor->buffer = g_UsbDeviceConfigurationDescriptor;
        configurationDescriptor->length = USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL;
        return kStatus_USB_Success;
    }

    return kStatus_USB_InvalidRequest;
}

static usb_status_t
USB_DeviceGetStringDescriptor(usb_device_handle handle,
        usb_device_get_string_descriptor_struct_t *stringDescriptor)
{
    if (!stringDescriptor->stringIndex) {
        stringDescriptor->buffer = (uint8_t *)g_UsbDeviceLanguageList.languageString;
        stringDescriptor->length = g_UsbDeviceLanguageList.stringLength;
    } else {
        uint8_t languageId = 0;
        uint8_t languageIndex = USB_DEVICE_STRING_COUNT;

        for (; languageId < USB_DEVICE_LANGUAGE_COUNT; languageId++) {
            if (stringDescriptor->languageId == g_UsbDeviceLanguageList.languageList[languageId].languageId) {
                if (stringDescriptor->stringIndex < USB_DEVICE_STRING_COUNT) {
                    languageIndex = stringDescriptor->stringIndex;
                }
                break;
            }
        }

        if (languageIndex == USB_DEVICE_STRING_COUNT) {
            return kStatus_USB_InvalidRequest;
        }
        stringDescriptor->buffer = (uint8_t *)g_UsbDeviceLanguageList.languageList[languageId].string[languageIndex];
        stringDescriptor->length = g_UsbDeviceLanguageList.languageList[languageId].length[languageIndex];
    }

    return kStatus_USB_Success;
}

//FIXME: this is only need for high speed, check later
#if 0
static usb_status_t
USB_DeviceSetSpeed(usb_device_handle handle, uint8_t speed)
{
    int i;
    usb_descriptor_union_t *ptr1;
    usb_descriptor_union_t *ptr2;
    int is_high_speed = (speed == USB_HIGH_SPEED);
    uint8_t ep_type;

    ptr1 = (usb_descriptor_union_t *) &g_UsbDeviceConfigurationDescriptor[0];
    ptr2 = (usb_descriptor_union_t *) &g_UsbDeviceConfigurationDescriptor[USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL - 1];
    ep_type = ptr1->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;

    while (ptr1 < ptr2) {
        if (ptr1->common.bDescriptorType == USB_DESCRIPTOR_TYPE_ENDPOINT) {
            if (is_high_speed) {
                if (ep_type == USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT) {
                    ptr1->endpoint.bInterval = HS_CDC_VCOM_INTERRUPT_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE,
                            ptr1->endpoint.wMaxPacketSize);
                } else if (ep_type == USB_CDC_VCOM_BULK_IN_ENDPOINT) {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_CDC_VCOM_BULK_IN_PACKET_SIZE,
                            ptr1->endpoint.wMaxPacketSize);
                } else if (ep_type == USB_CDC_VCOM_BULK_OUT_ENDPOINT) {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_CDC_VCOM_BULK_OUT_PACKET_SIZE,
                            ptr1->endpoint.wMaxPacketSize);
                }
            } else {
                if (ep_type == USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT) {
                    ptr1->endpoint.bInterval = FS_CDC_VCOM_INTERRUPT_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE,
                            ptr1->endpoint.wMaxPacketSize);
                } else if (ep_type == USB_CDC_VCOM_BULK_IN_ENDPOINT) {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_CDC_VCOM_BULK_IN_PACKET_SIZE,
                            ptr1->endpoint.wMaxPacketSize);
                } else if (ep_type == USB_CDC_VCOM_BULK_OUT_ENDPOINT) {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_CDC_VCOM_BULK_OUT_PACKET_SIZE,
                            ptr1->endpoint.wMaxPacketSize);
                }
            }
        }
        ptr1 = (usb_descriptor_union_t *)((uint8_t *)ptr1 + ptr1->common.bLength);
    }

    for (i = 0; i < USB_CDC_VCOM_ENDPOINT_CIC_COUNT; i++) {
        g_UsbDeviceCdcVcomCicEndpoints[i].maxPacketSize = is_high_speed ?
            HS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE :
            FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE;
    }

    for (i = 0; i < USB_CDC_VCOM_ENDPOINT_DIC_COUNT; i++) {
        g_UsbDeviceCdcVcomDicEndpoints[i].maxPacketSize = is_high_speed ?
            HS_CDC_VCOM_BULK_OUT_PACKET_SIZE :
            FS_CDC_VCOM_BULK_OUT_PACKET_SIZE;
    }

    return kStatus_USB_Success;
}
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE)
volatile static uint8_t s_waitForDataReceive = 0;
volatile static uint8_t s_comOpen = 0;
#endif

static usb_status_t
usb_device_cdc_cb(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint32_t len;
    uint16_t *uartBitmap;
    usb_cdc_acm_info_t *acmInfo = &s_usbCdcAcmInfo;
    usb_dev_cdc_req_param_t *acmReqParam;
    usb_dev_ep_cb_msg_t *epCbParam;
    acmReqParam = (usb_dev_cdc_req_param_t *)param;
    epCbParam = (usb_dev_ep_cb_msg_t *)param;

    //printf("cdc event=%d\n", event);
    switch (event) {
    case kUSB_DeviceCdcEventSendResponse:
        if (epCbParam->length && !(epCbParam->length % g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize)) {
            /* If the last packet is the size of endpoint, then send also zero-ended packet,
             ** meaning that we want to inform the host that we do not have any additional
             ** data, so it can flush the output.
             */
            error = usb_dev_cdc_send(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0);
        } else if (s_cdc_vcom.attach && s_cdc_vcom.startTransactions) {
            if (epCbParam->buffer || (!epCbParam->buffer && !epCbParam->length)) {
                /* User: add your own code for send complete event */
                /* Schedule buffer for next receive event */
                error = usb_dev_cdc_recv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                         g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
#if defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE)
                s_waitForDataReceive = 1;
                USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
#endif
            }
        }
        break;
    case kUSB_DeviceCdcEventRecvResponse:
        if (s_cdc_vcom.attach && s_cdc_vcom.startTransactions) {
            s_recvSize = epCbParam->length;

#if defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE)
            s_waitForDataReceive = 0;
            USB0->INTEN |= USB_INTEN_SOFTOKEN_MASK;
#endif
            if (!s_recvSize) {
                /* Schedule buffer for next receive event */
                error = usb_dev_cdc_recv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                         g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
#if  defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE)
                s_waitForDataReceive = 1;
                USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
#endif
            }
        }
        break;
    case kUSB_DeviceCdcEventSerialStateNotif:
        ((usb_dev_cdc_t *)handle)->hasSentState = 0;
        error = kStatus_USB_Success;
        break;
    case kUSB_DeviceCdcEventSendEncapsulatedCommand:
        break;
    case kUSB_DeviceCdcEventGetEncapsulatedResponse:
        break;
    case kUSB_DeviceCdcEventSetCommFeature:
        if (acmReqParam->setupValue == USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE) {
            if (acmReqParam->isSetup) {
                *acmReqParam->buffer = s_abstractState;
            } else {
                *acmReqParam->length = 0;
            }
        }
        else if (acmReqParam->setupValue == USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING) {
            if (acmReqParam->isSetup) {
                *(acmReqParam->buffer) = s_countryCode;
            } else {
                *(acmReqParam->length) = 0;
            }
        }
        error = kStatus_USB_Success;
        break;
    case kUSB_DeviceCdcEventGetCommFeature:
        if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
        {
            *(acmReqParam->buffer) = s_abstractState;
            *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
        }
        else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
        {
            *(acmReqParam->buffer) = s_countryCode;
            *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
        }
        error = kStatus_USB_Success;
        break;
    case kUSB_DeviceCdcEventClearCommFeature:
        break;
    case kUSB_DeviceCdcEventGetLineCoding:
        *acmReqParam->buffer = s_lineCoding;
        *acmReqParam->length = LINE_CODING_SIZE;
        error = kStatus_USB_Success;
        break;
    case kUSB_DeviceCdcEventSetLineCoding:
    {
        if (acmReqParam->isSetup) {
            *acmReqParam->buffer = s_lineCoding;
        } else {
            *acmReqParam->length = 0;
        }
    }
        error = kStatus_USB_Success;
        break;
    case kUSB_DeviceCdcEventSetControlLineState:
    {
        s_usbCdcAcmInfo.dteStatus = acmReqParam->setupValue;
        /* activate/deactivate Tx carrier */
        if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION) {
            acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
        } else {
            acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
        }

        /* activate carrier and DTE */
        if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) {
            acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
        } else {
            acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
        }

        /* Indicates to DCE if DTE is present or not */
        acmInfo->dtePresent = (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) ? 1 : 0;

        acmInfo->serialStateBuf[0] = NOTIF_REQUEST_TYPE;                /* bmRequestType */
        acmInfo->serialStateBuf[1] = USB_DEVICE_CDC_NOTIF_SERIAL_STATE; /* bNotification */
        acmInfo->serialStateBuf[2] = 0x00;                              /* wValue */
        acmInfo->serialStateBuf[3] = 0x00;
        acmInfo->serialStateBuf[4] = 0x00; /* wIndex */
        acmInfo->serialStateBuf[5] = 0x00;
        acmInfo->serialStateBuf[6] = UART_BITMAP_SIZE; /* wLength */
        acmInfo->serialStateBuf[7] = 0x00;
        acmInfo->serialStateBuf[4] = acmReqParam->interfaceIndex;

        uartBitmap = (uint16_t *)&acmInfo->serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE - 2];
        *uartBitmap = acmInfo->uartState;
        len = (uint32_t)(NOTIF_PACKET_SIZE + UART_BITMAP_SIZE);
        if (!((usb_dev_cdc_t *)handle)->hasSentState) {
            error = usb_dev_cdc_send(handle, USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT, acmInfo->serialStateBuf, len);
            if (error != kStatus_USB_Success) {
                //FIXME
                //usb_echo("kUSB_DeviceCdcEventSetControlLineState error!");
            }
            ((usb_dev_cdc_t *)handle)->hasSentState = 1;
        }

        if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION) {
            /*  To do: CARRIER_ACTIVATED */
        } else {
            /* To do: CARRIER_DEACTIVATED */
        }

        if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) {
            /* DTE_ACTIVATED */
            if (s_cdc_vcom.attach) {
                s_cdc_vcom.startTransactions = 1;

#if MYNEWT_VAL(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE)
                s_waitForDataReceive = 1;
                USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
                s_comOpen = 1;
                usb_echo("USB_APP_CDC_DTE_ACTIVATED\r\n");
#endif
            }
        } else {
            /* DTE_DEACTIVATED */
            if (s_cdc_vcom.attach) {
                s_cdc_vcom.startTransactions = 0;
            }
        }
    }
    break;
    }

    return error;
}

static usb_status_t
usb_device_cb(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;

    //printf("event=%lu\n", event);
    switch (event) {
    case kUSB_DeviceEventBusReset:
        s_cdc_vcom.attach = 0;

#if 0 //FIXME: need for high speed?
#if defined(USB_DEVICE_CONFIG_EHCI)
        if (USB_DeviceClassGetSpeed(CONTROLLER_ID, &s_cdc_vcom.speed) == kStatus_USB_Success) {
            USB_DeviceSetSpeed(handle, s_cdc_vcom.speed);
        }
#endif
#endif
        break;
    case kUSB_DeviceEventSetConfiguration:
        if (param) {
            s_cdc_vcom.attach = 1;
            s_cdc_vcom.currentConfiguration = *((uint8_t *)param);
            if (*((uint8_t *)param) == USB_CDC_VCOM_CONFIGURE_INDEX) {
                usb_dev_cdc_recv(s_cdc_vcom.cdcAcmHandle,
                        USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                        g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize);
            }
        }
        break;
    case kUSB_DeviceEventSetInterface:
        if (s_cdc_vcom.attach) {
            uint8_t iface = (uint8_t)((*((uint16_t *)param) & 0xFF00) >> 0x08);
            uint8_t alt_setting = (uint8_t)(*((uint16_t *)param) & 0x00FF);
            if (iface < USB_CDC_VCOM_INTERFACE_COUNT) {
                s_cdc_vcom.currentInterfaceAlternateSetting[iface] = alt_setting;
            }
        }
        break;
    case kUSB_DeviceEventGetConfiguration:
        break;
    case kUSB_DeviceEventGetInterface:
        break;
    case kUSB_DeviceEventGetDeviceDescriptor:
        if (param) {
            error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
        }
        break;
    case kUSB_DeviceEventGetConfigurationDescriptor:
        if (param) {
            error = USB_DeviceGetConfigurationDescriptor(handle,
                                                         (usb_device_get_configuration_descriptor_struct_t *)param);
        }
        break;
    case kUSB_DeviceEventGetStringDescriptor:
        if (param) {
            error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
        }
        break;
    }

    return error;
}

static void
usb_device_application_init(void)
{
    usb_status_t status;

    usb_hal_init_clocks();
    usb_hal_clear_memory();

    s_cdc_vcom.speed = USB_SPEED_FULL;
    s_cdc_vcom.attach = 0;
    s_cdc_vcom.cdcAcmHandle = 0;
    s_cdc_vcom.deviceHandle = NULL;

    status = usb_device_class_init(CONTROLLER_ID, &s_cdcAcmConfigList, &s_cdc_vcom.deviceHandle);
    if (status == kStatus_USB_Success) {
        //FIXME
        //usb_echo("USB device CDC virtual com demo\r\n");
        s_cdc_vcom.cdcAcmHandle = s_cdcAcmConfigList.config->handle;
    } else {
        //FIXME
        //usb_echo("USB device init failed\r\n");
        return;
    }

    usb_hal_set_dev_handle(s_cdc_vcom.deviceHandle);
    usb_hal_enable_irq();
    usb_device_run(s_cdc_vcom.deviceHandle);
}

static void
usb_app_task_handler(void *arg)
{
    usb_status_t error = kStatus_USB_Error;
    uint8_t last_attach = s_cdc_vcom.attach;
    int i;

    while (1) {
        if (s_cdc_vcom.attach && s_cdc_vcom.startTransactions) {
            if (s_recvSize && s_recvSize != 0xFFFFFFFF) {
                //printf("s_recvSize=%d\n", s_recvSize);
                for (i = 0; i < s_recvSize; i++) {
                    s_currSendBuf[s_sendSize++] = s_currRecvBuf[i];
                }
                s_recvSize = 0;
            }

            if (s_sendSize) {
                //printf("s_sendSize=%d\n", s_sendSize);
                uint32_t size = s_sendSize;
                s_sendSize = 0;

                error = usb_dev_cdc_send(s_cdc_vcom.cdcAcmHandle,
                        USB_CDC_VCOM_BULK_IN_ENDPOINT, s_currSendBuf, size);
                if (error != kStatus_USB_Success)
                {
                    /* Failure to send Data Handling code here */
                }
            }
#if 0 // MYNEWT_VAL(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE)
            if (s_waitForDataReceive) {
                if (s_comOpen) {
                    /* Wait for all the packets been sent during opening the com port. Otherwise these packets may
                     * wake up the system.
                    */
                    usb_echo("Waiting to enter lowpower ...\r\n");
                    for (uint32_t i = 0; i < 16000000; ++i) {
                        __ASM("NOP");
                    }

                    s_comOpen = 0;
                }

                usb_echo("Enter lowpower\r\n");
                USB0->INTEN &= ~USB_INTEN_TOKDNEEN_MASK;

                SMC_SetPowerModeVlps(SMC);

                s_waitForDataReceive = 0;
                USB0->INTEN |= USB_INTEN_TOKDNEEN_MASK;
                usb_echo("Exit  lowpower\r\n");
            }
#endif
        }
        if (last_attach != s_cdc_vcom.attach) {
            printf("attach=%d, startTransaction=%d\n", s_cdc_vcom.attach, s_cdc_vcom.startTransactions);
            last_attach = s_cdc_vcom.attach;
        }
        os_time_delay(1);
    }
}

static void
usb_dev_task_handler(void *handle)
{
    while (1) {
        usb_device_task_fn(handle);
        //os_time_delay(1);
    }
}

static void
blinker_handler(void *arg)
{
    hal_gpio_init_out(LED_BLINK_PIN, 1);

    while (1) {
        os_time_delay(OS_TICKS_PER_SEC * 3);
        hal_gpio_toggle(LED_BLINK_PIN);
    }
}

/**
 * init_tasks
 *
 * Called by main.c after sysinit(). This function performs initializations
 * that are required before tasks are running.
 *
 * @return int 0 success; error otherwise.
 */
static void
init_tasks(void)
{
    os_stack_t *pstack;

    pstack = malloc(sizeof(os_stack_t) * BLINKER_STACK_SIZE);
    assert(pstack);

    os_task_init(&blinker_task, "led_blinker", blinker_handler, NULL,
            BLINKER_PRIO, OS_WAIT_FOREVER, pstack, BLINKER_STACK_SIZE);

    usb_device_application_init();

    pstack = malloc(sizeof(os_stack_t) * USB_APP_STACK_SIZE);
    assert(pstack);

    os_task_init(&usb_app_task, "usb_app_task", usb_app_task_handler, NULL,
            USB_APP_TASK_PRIO, OS_WAIT_FOREVER, pstack, USB_APP_STACK_SIZE);

    pstack = malloc(sizeof(os_stack_t) * USB_DEV_STACK_SIZE);
    assert(pstack);

    //printf("s_cdc_vcom.deviceHandle=0x%lx\n", s_cdc_vcom.deviceHandle);
    os_task_init(&usb_dev_task, "usb_dev_task", usb_dev_task_handler, s_cdc_vcom.deviceHandle,
            USB_DEV_TASK_PRIO, OS_WAIT_FOREVER, pstack, USB_DEV_STACK_SIZE);
}

/**
 * main
 *
 * The main task for the project. This function initializes the packages, calls
 * init_tasks to initialize additional tasks (and possibly other objects),
 * then starts serving events from default event queue.
 *
 * @return int NOTE: this function should never return!
 */
int
main(int argc, char **argv)
{
    //int rc;

#ifdef ARCH_sim
    mcu_sim_parse_args(argc, argv);
#endif

    sysinit();

#if 0
    stats_init(STATS_HDR(g_stats_gpio_toggle),
               STATS_SIZE_INIT_PARMS(g_stats_gpio_toggle, STATS_SIZE_32),
               STATS_NAME_INIT_PARMS(gpio_stats));

    stats_register("gpio_toggle", STATS_HDR(g_stats_gpio_toggle));
#endif

    printf("booting\n");

    init_tasks();

    /*
     * As the last thing, process events from default event queue.
     */
    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }
}
