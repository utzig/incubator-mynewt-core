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
#include <msc/msc.h>

#include <hal_usb/hal_usb.h>

#ifdef ARCH_sim
#include <mcu/mcu_sim.h>
#endif

#define BLINKER_PRIO                       8
#define BLINKER_STACK_SIZE                 OS_STACK_ALIGN(192)
static struct os_task blinker_task;

#define USB_DEV_TASK_PRIO                  7
#define USB_DEV_STACK_SIZE                 OS_STACK_ALIGN(2048)
static struct os_task usb_dev_task;

#define CONTROLLER_ID kUSB_ControllerKhci0

#if MYNEWT_VAL(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE)
volatile static uint8_t s_waitForDataReceive = 0;
volatile static uint8_t s_comOpen = 0;
#endif

#define LENGTH_OF_EACH_LBA                    512
#define TOTAL_LOGICAL_ADDRESS_BLOCKS_NORMAL   48
/* Net Disk Size , default disk is 48*512, that is 24kByte, however , the disk reconnised by that PC only has 4k Byte,
 * This is caused by that the file system also need memory*/
#define DISK_SIZE_NORMAL (TOTAL_LOGICAL_ADDRESS_BLOCKS_NORMAL * LENGTH_OF_EACH_LBA)

#define LOGICAL_UNIT_SUPPORTED 1

#define USB_DEVICE_SPECIFIC_BCD_VERSION    0x0200
#define USB_DEVICE_DEMO_BCD_VERSION        0x0101
#define USB_DEVICE_MAX_POWER               0x32

#define USB_CONFIGURE_COUNT                1
#define USB_DEVICE_STRING_COUNT            4
#define USB_DEVICE_LANGUAGE_COUNT          1
#define USB_INTERFACE_COUNT                1

#define USB_MSC_CONFIGURE_INDEX            1

#define USB_MSC_ENDPOINT_COUNT             2
#define USB_MSC_BULK_IN_ENDPOINT           1
#define USB_MSC_BULK_OUT_ENDPOINT          2

/* usb descritpor length */
#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL                          \
    (USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_INTERFACE + \
     USB_DESCRIPTOR_LENGTH_ENDPOINT * USB_MSC_ENDPOINT_COUNT)

#define HS_MSC_BULK_IN_PACKET_SIZE         512
#define HS_MSC_BULK_OUT_PACKET_SIZE        512
#define FS_MSC_BULK_IN_PACKET_SIZE         64
#define FS_MSC_BULK_OUT_PACKET_SIZE        64

#define USB_DESCRIPTOR_LENGTH_STRING0      0x04
#define USB_DESCRIPTOR_LENGTH_STRING1      38
#define USB_DESCRIPTOR_LENGTH_STRING2      34
#define USB_DESCRIPTOR_LENGTH_STRING3      34
#define USB_STRING_DESCRIPTOR_ERROR_LENGTH 34

#define USB_MSC_INTERFACE_INDEX            0
#define USB_MSC_INTERFACE_COUNT            1

#define USB_DEVICE_CLASS                   0x00
#define USB_DEVICE_SUBCLASS                0x00
#define USB_DEVICE_PROTOCOL                0x00

#define USB_MSC_CLASS                      0x08
/* scsi command set */
#define USB_MSC_SUBCLASS                   0x06
/* bulk only transport protocol */
#define USB_MSC_PROTOCOL                   0x50

typedef struct
{
    usb_device_handle deviceHandle;
    class_handle_t mscHandle;
    //xTaskHandle device_task_handle;
    //xTaskHandle application_task_handle;
    uint8_t *storageDisk;
    uint8_t diskLock;
    uint8_t read_write_error;
    uint8_t currentConfiguration;
    uint8_t currentInterfaceAlternateSetting[USB_MSC_INTERFACE_COUNT];
    uint8_t speed;
    uint8_t attach;
} usb_msc_struct_t;

usb_device_inquiry_data_fromat_struct_t g_InquiryInfo = {
    (USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER << USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER_SHIFT) |
        USB_DEVICE_MSC_UFI_PERIPHERAL_DEVICE_TYPE,
    (uint8_t)(USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT << USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT_SHIFT),
    USB_DEVICE_MSC_UFI_VERSIONS,
    0x02,
    USB_DEVICE_MSC_UFI_ADDITIONAL_LENGTH,
    {
        0x00, 0x00, 0x00,
    },
    {
        'N', 'X', 'P', ' ', 'S', 'E', 'M', 'I',
    },
    {
        'N', 'X', 'P', ' ', 'M', 'A', 'S', 'S', ' ', 'S', 'T', 'O', 'R', 'A', 'G', 'E',
    },
    {
        '0', '0', '0', '1',
    },
};

usb_device_mode_parameters_header_struct_t g_ModeParametersHeader = {
    /*refer to ufi spec mode parameter header*/
    0x0000, /*!< Mode Data Length*/
    0x00,   /*!<Default medium type (current mounted medium type)*/
    0x00,   /*!MODE SENSE command, a Write Protected bit of zero indicates the medium is write enabled*/
    {
        0x00, 0x00, 0x00, 0x00,
    },
};

static uint8_t s_StorageDisk[DISK_SIZE_NORMAL];
usb_msc_struct_t g_msc;

usb_dev_ep_t g_UsbDeviceMscEndpoints[USB_MSC_ENDPOINT_COUNT] = {
    {
        USB_MSC_BULK_IN_ENDPOINT | 0x80,
        USB_ENDPOINT_BULK,
        FS_MSC_BULK_IN_PACKET_SIZE,
    },
    {
        USB_MSC_BULK_OUT_ENDPOINT,
        USB_ENDPOINT_BULK,
        FS_MSC_BULK_OUT_PACKET_SIZE,
    },
};

usb_dev_itf_t g_UsbDeviceMscInterface[] = {
    {
        0,
        {
            USB_MSC_ENDPOINT_COUNT,
            g_UsbDeviceMscEndpoints,
        },
        NULL,
    },
};

usb_dev_itfs_t g_UsbDeviceMscInterfaces[USB_MSC_INTERFACE_COUNT] = {
    {
        USB_MSC_CLASS,
        USB_MSC_SUBCLASS,
        USB_MSC_PROTOCOL,
        USB_MSC_INTERFACE_INDEX,
        g_UsbDeviceMscInterface,
        sizeof(g_UsbDeviceMscInterface) / sizeof(usb_dev_itfs_t),
    },
};

usb_device_interface_list_t g_UsbDeviceMscInterfaceList[USB_CONFIGURE_COUNT] = {
    {
        USB_MSC_INTERFACE_COUNT,
        g_UsbDeviceMscInterfaces,
    },
};

uint8_t g_UsbDeviceDescriptor[USB_DESCRIPTOR_LENGTH_DEVICE] = {

    USB_DESCRIPTOR_LENGTH_DEVICE,
    USB_DESCRIPTOR_TYPE_DEVICE,
    USB_SHORT_GET_LOW(USB_DEVICE_SPECIFIC_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_SPECIFIC_BCD_VERSION),

    USB_DEVICE_CLASS,
    USB_DEVICE_SUBCLASS,
    USB_DEVICE_PROTOCOL,
    USB_CONTROL_MAX_PACKET_SIZE,

    0xC9, 0x1F,                                          /* Vendor ID (assigned by the USB-IF) */
    0x92, 0x00,                                          /* Product ID (assigned by the manufacturer) */
    USB_SHORT_GET_LOW(USB_DEVICE_DEMO_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_DEMO_BCD_VERSION),
    0x01,                                           /* Index of string descriptor describing manufacturer */
    0x02,                                           /* Index of string descriptor describing product */
    0x03,               /* Index of string descriptor describing the device's serial number */
    USB_CONFIGURE_COUNT, /* Number of possible configurations */
};

uint8_t g_UsbDeviceConfigurationDescriptor[USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL] = {
    USB_DESCRIPTOR_LENGTH_CONFIGURE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_CONFIGURE,   /* CONFIGURATION Descriptor Type */
    USB_SHORT_GET_LOW(USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL),
    USB_SHORT_GET_HIGH(
        USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL), /* Total length of data returned for this configuration. */
    USB_MSC_INTERFACE_COUNT,                      /* Number of interfaces supported by this configuration */
    USB_MSC_CONFIGURE_INDEX,                      /* Value to use as an argument to the
                                                          SetConfiguration() request to select this configuration */
    0,                                            /* Index of string descriptor describing this configuration */
    (USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK) |
        (MYNEWT_VAL(USB_DEVICE_CONFIG_SELF_POWER) << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_SHIFT) |
        (MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP) << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_SHIFT),
    USB_DEVICE_MAX_POWER,
    USB_DESCRIPTOR_LENGTH_INTERFACE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_INTERFACE,   /* INTERFACE Descriptor Type */
    USB_MSC_INTERFACE_INDEX,         /* Number of this interface. */
    0x00,                            /* Value used to select this alternate setting
                                                             for the interface identified in the prior field */
    USB_MSC_ENDPOINT_COUNT,          /* Number of endpoints used by this
                                                              interface (excluding endpoint zero). */

    USB_MSC_CLASS,    /* Class code (assigned by the USB-IF). */
    USB_MSC_SUBCLASS, /* Subclass code (assigned by the USB-IF). */
    USB_MSC_PROTOCOL, /* Protocol code (assigned by the USB). */
    0x00,             /* Index of string descriptor describing this interface */

    USB_DESCRIPTOR_LENGTH_ENDPOINT, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_ENDPOINT,   /* ENDPOINT Descriptor Type */
    USB_MSC_BULK_IN_ENDPOINT | 0x80,
    /* The address of the endpoint on the USB device
                             described by this descriptor. */
    USB_ENDPOINT_BULK, /* This field describes the endpoint's attributes */
    USB_SHORT_GET_LOW(FS_MSC_BULK_IN_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_MSC_BULK_IN_PACKET_SIZE), /* Maximum packet size this endpoint is capable of sending or
                                                       receiving when this configuration is selected. */
    0x00,                                           /*Useless for bulk in endpoint*/

    USB_DESCRIPTOR_LENGTH_ENDPOINT, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_ENDPOINT,   /* ENDPOINT Descriptor Type */
    USB_MSC_BULK_OUT_ENDPOINT,
    /* The address of the endpoint on the USB device
                         described by this descriptor. */
    USB_ENDPOINT_BULK, /* This field describes the endpoint's attributes */
    FS_MSC_BULK_OUT_PACKET_SIZE, 0x00,
    0x00  /*For high-speed bulk/control OUT endpoints, the bInterval must specify the
         maximum NAK rate of the endpoint. refer to usb spec 9.6.6*/
};

uint8_t g_UsbDeviceString0[USB_DESCRIPTOR_LENGTH_STRING0] = {
    sizeof(g_UsbDeviceString0),
    USB_DESCRIPTOR_TYPE_STRING,
    0x09,
    0x04,
};

uint8_t g_UsbDeviceString1[USB_DESCRIPTOR_LENGTH_STRING1] = {
    sizeof(g_UsbDeviceString1),
    USB_DESCRIPTOR_TYPE_STRING,
    'N', 0, 'X', 0, 'P', 0, ' ', 0, 'S', 0, 'E', 0, 'M', 0, 'I', 0,
    'C', 0, 'O', 0, 'N', 0, 'D', 0, 'U', 0, 'C', 0, 'T', 0, 'O', 0,
    'R', 0, 'S', 0,
};

uint8_t g_UsbDeviceString2[USB_DESCRIPTOR_LENGTH_STRING2] = {
    sizeof(g_UsbDeviceString2),
    USB_DESCRIPTOR_TYPE_STRING,
    'M', 0, 'C', 0, 'U', 0, ' ', 0, 'M', 0, 'A', 0, 'S', 0, 'S', 0,
    ' ', 0, 'S', 0, 'T', 0, 'O', 0, 'R', 0, 'A', 0, 'G', 0, 'E', 0,
};

uint8_t g_UsbDeviceString3[USB_DESCRIPTOR_LENGTH_STRING3] = {
    sizeof(g_UsbDeviceString3),
    USB_DESCRIPTOR_TYPE_STRING,
    '0', 0, '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0,
    '8', 0, '9', 0, 'A', 0, 'B', 0, 'C', 0, 'D', 0, 'E', 0, 'F', 0,
};

uint8_t g_UsbDeviceStringN[USB_STRING_DESCRIPTOR_ERROR_LENGTH] = {
    sizeof(g_UsbDeviceStringN),
    USB_DESCRIPTOR_TYPE_STRING,
    'B', 0, 'A', 0, 'D', 0, ' ', 0, 'S', 0, 'T', 0, 'R', 0, 'I', 0,
    'N', 0, 'G', 0, ' ', 0, 'I', 0, 'N', 0, 'D', 0, 'E', 0, 'X', 0,
};

uint32_t g_UsbStringDescriptorSize[USB_DEVICE_STRING_COUNT + 1] = {
    sizeof(g_UsbDeviceString0),
    sizeof(g_UsbDeviceString1),
    sizeof(g_UsbDeviceString2),
    sizeof(g_UsbDeviceString3),
    sizeof(g_UsbDeviceStringN),
};

uint8_t *g_UsbStringDescriptors[USB_DEVICE_STRING_COUNT + 1] = {
    g_UsbDeviceString0,
    g_UsbDeviceString1,
    g_UsbDeviceString2,
    g_UsbDeviceString3,
    g_UsbDeviceStringN,
};

usb_language_t g_UsbLanguage[USB_DEVICE_LANGUAGE_COUNT] = {{
    g_UsbStringDescriptors,
    g_UsbStringDescriptorSize,
    (uint16_t)0x0409,
}};

usb_language_list_t g_UsbDeviceLanguageList = {
    g_UsbDeviceString0,
    sizeof(g_UsbDeviceString0),
    g_UsbLanguage,
    USB_DEVICE_LANGUAGE_COUNT,
};

usb_dev_class_t g_UsbDeviceMscConfig = {
    g_UsbDeviceMscInterfaceList, /* The interface list of the msc */
    kUSB_DeviceClassTypeMsc,     /* The msc class type */
    USB_CONFIGURE_COUNT,         /* The configuration count */
};

static usb_status_t usb_dev_cb(usb_device_handle handle, uint32_t event, void *param);
static usb_status_t usb_dev_msc_cb(class_handle_t handle, uint32_t event, void *param);

usb_dev_class_config_t msc_config[] = {
    {
        usb_dev_msc_cb,
        0,
        &g_UsbDeviceMscConfig,
    },
};

static usb_dev_class_configs_t msc_config_list = {
    msc_config,
    usb_dev_cb,
    1,
};

usb_status_t USB_DeviceGetDeviceDescriptor(usb_device_handle handle,
                                           usb_device_get_device_descriptor_struct_t *deviceDescriptor)
{
    deviceDescriptor->buffer = g_UsbDeviceDescriptor;
    deviceDescriptor->length = USB_DESCRIPTOR_LENGTH_DEVICE;
    return kStatus_USB_Success;
}

usb_status_t USB_DeviceGetConfigurationDescriptor(
    usb_device_handle handle, usb_device_get_configuration_descriptor_struct_t *configurationDescriptor)
{
    if (USB_MSC_CONFIGURE_INDEX > configurationDescriptor->configuration)
    {
        configurationDescriptor->buffer = g_UsbDeviceConfigurationDescriptor;
        configurationDescriptor->length = USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL;
        return kStatus_USB_Success;
    }
    return kStatus_USB_InvalidRequest;
}

usb_status_t USB_DeviceGetStringDescriptor(usb_device_handle handle,
                                           usb_device_get_string_descriptor_struct_t *stringDescriptor)
{
    if (stringDescriptor->stringIndex == 0) {
        stringDescriptor->buffer = (uint8_t *)g_UsbDeviceLanguageList.languageString;
        stringDescriptor->length = g_UsbDeviceLanguageList.stringLength;
    } else {
        uint8_t languageId = 0U;
        uint8_t languageIndex = USB_DEVICE_STRING_COUNT;

        for (; languageId < USB_DEVICE_LANGUAGE_COUNT; languageId++) {
            if (stringDescriptor->languageId == g_UsbDeviceLanguageList.languageList[languageId].languageId) {
                if (stringDescriptor->stringIndex < USB_DEVICE_STRING_COUNT) {
                    languageIndex = stringDescriptor->stringIndex;
                }
                break;
            }
        }

        if (USB_DEVICE_STRING_COUNT == languageIndex) {
            return kStatus_USB_InvalidRequest;
        }
        stringDescriptor->buffer = (uint8_t *)g_UsbDeviceLanguageList.languageList[languageId].string[languageIndex];
        stringDescriptor->length = g_UsbDeviceLanguageList.languageList[languageId].length[languageIndex];
    }
    return kStatus_USB_Success;
}


static usb_status_t
usb_dev_msc_cb(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    usb_device_lba_information_struct_t *lbaInformationStructure;
    usb_device_lba_app_struct_t *lbaData;
    usb_device_ufi_app_struct_t *ufi;

    switch (event)
    {
    case kUSB_DeviceMscEventReadResponse:
        lbaData = (usb_device_lba_app_struct_t *)param;
        break;
    case kUSB_DeviceMscEventWriteResponse:
        lbaData = (usb_device_lba_app_struct_t *)param;
        break;
    case kUSB_DeviceMscEventWriteRequest:
        lbaData = (usb_device_lba_app_struct_t *)param;
        /*offset is the write start address get from write command, refer to class driver*/
        lbaData->buffer = g_msc.storageDisk + lbaData->offset * LENGTH_OF_EACH_LBA;
        break;
    case kUSB_DeviceMscEventReadRequest:
        lbaData = (usb_device_lba_app_struct_t *)param;
        /*offset is the read start address get from read command, refer to class driver*/
        lbaData->buffer = g_msc.storageDisk + lbaData->offset * LENGTH_OF_EACH_LBA;
        break;
    case kUSB_DeviceMscEventGetLbaInformation:
        lbaInformationStructure = (usb_device_lba_information_struct_t *)param;
        lbaInformationStructure->lengthOfEachLba = LENGTH_OF_EACH_LBA;
        lbaInformationStructure->totalLbaNumberSupports = TOTAL_LOGICAL_ADDRESS_BLOCKS_NORMAL;
        lbaInformationStructure->logicalUnitNumberSupported = LOGICAL_UNIT_SUPPORTED;
        lbaInformationStructure->bulkInBufferSize = DISK_SIZE_NORMAL;
        lbaInformationStructure->bulkOutBufferSize = DISK_SIZE_NORMAL;
        break;
    case kUSB_DeviceMscEventTestUnitReady:
        /*change the test unit ready command's sense data if need, be careful to modify*/
        ufi = (usb_device_ufi_app_struct_t *)param;
        break;
    case kUSB_DeviceMscEventInquiry:
        ufi = (usb_device_ufi_app_struct_t *)param;
        ufi->size = sizeof(usb_device_inquiry_data_fromat_struct_t);
        ufi->buffer = (uint8_t *)&g_InquiryInfo;
        break;
    case kUSB_DeviceMscEventModeSense:
        ufi = (usb_device_ufi_app_struct_t *)param;
        ufi->size = sizeof(usb_device_mode_parameters_header_struct_t);
        ufi->buffer = (uint8_t *)&g_ModeParametersHeader;
        break;
    case kUSB_DeviceMscEventModeSelect:
        break;
    case kUSB_DeviceMscEventModeSelectResponse:
        ufi = (usb_device_ufi_app_struct_t *)param;
        break;
    case kUSB_DeviceMscEventFormatComplete:
        break;
    case kUSB_DeviceMscEventRemovalRequest:
        break;
    default:
        break;
    }
    return error;
}

static usb_status_t
usb_dev_cb(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint16_t *temp16 = (uint16_t *)param;
    uint8_t *temp8 = (uint8_t *)param;

    //printf("event=%lu\n", event);
    switch (event) {
    case kUSB_DeviceEventBusReset:
        g_msc.attach = 0;
        error = kStatus_USB_Success;
        break;
    case kUSB_DeviceEventSetConfiguration:
        if (param) {
            g_msc.attach = 1;
            g_msc.currentConfiguration = *temp8;
        }
        break;
    case kUSB_DeviceEventSetInterface:
        if (g_msc.attach) {
            uint8_t interface = (uint8_t)((*temp16 & 0xFF00) >> 0x08);
            uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FF);
            if (interface < USB_MSC_INTERFACE_COUNT) {
                g_msc.currentInterfaceAlternateSetting[interface] = alternateSetting;
            }
        }
        break;
    case kUSB_DeviceEventGetConfiguration:
        if (param) {
            *temp8 = g_msc.currentConfiguration;
            error = kStatus_USB_Success;
        }
        break;
    case kUSB_DeviceEventGetInterface:
        if (param) {
            uint8_t interface = (uint8_t)((*temp16 & 0xFF00) >> 0x08);
            if (interface < USB_INTERFACE_COUNT) {
                *temp16 = (*temp16 & 0xFF00) | g_msc.currentInterfaceAlternateSetting[interface];
                error = kStatus_USB_Success;
            } else {
                error = kStatus_USB_InvalidRequest;
            }
        }
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

    g_msc.speed = USB_SPEED_FULL;
    g_msc.attach = 0;
    g_msc.mscHandle = (class_handle_t)NULL;
    g_msc.deviceHandle = NULL;
    g_msc.storageDisk = &s_StorageDisk[0];

    status = usb_device_class_init(CONTROLLER_ID, &msc_config_list, &g_msc.deviceHandle);
    if (status == kStatus_USB_Success) {
        g_msc.mscHandle = msc_config_list.config->handle;
    } else {
        return;
    }

    usb_hal_set_dev_handle(g_msc.deviceHandle);
    usb_hal_enable_irq();
    usb_device_run(g_msc.deviceHandle);
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

    pstack = malloc(sizeof(os_stack_t) * USB_DEV_STACK_SIZE);
    assert(pstack);

    os_task_init(&usb_dev_task, "usb_dev_task", usb_dev_task_handler, g_msc.deviceHandle,
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

    console_printf("booting\n");

    init_tasks();

    /*
     * As the last thing, process events from default event queue.
     */
    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }
}
