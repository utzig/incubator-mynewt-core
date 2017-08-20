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

#ifndef __HAL_USB_H__
#define __HAL_USB_H__

#include <usb/usb.h>
#include <stm32f7xx_ll_usb.h>
#include <os/os.h>

#define USB_DATA_ALIGNMENT

typedef enum
{
    LPM_L0 = 0x00, /* on */
    LPM_L1 = 0x01, /* LPM L1 sleep */
} PCD_LPM_StateTypeDef;

enum {
    USB_STATE_OFF = 0,
    USB_STATE_RESET = 1,
    USB_STATE_READY = 2,
};

typedef struct
{
    uint8_t  *xfer_buf;
    uint32_t xfer_len;
    uint32_t xfer_done;
    /* TODO: mcu specific ep state */
} stm32_usb_dev_ep_state_t;

typedef struct
{
    usb_dev_t                *dev;
    uint8_t                  isResetting;
    uint8_t                  controllerId;

    /* device specific */
    USB_OTG_GlobalTypeDef    *Instance;    /*!< Register base address              */
    USB_OTG_CfgTypeDef       Init;         /*!< PCD required parameters            */
    USB_OTG_EPTypeDef        IN_ep[16];    /*!< IN endpoint parameters             */
    USB_OTG_EPTypeDef        OUT_ep[16];   /*!< OUT endpoint parameters            */
    HAL_LockTypeDef          Lock;         /*!< PCD peripheral status              */
    //__IO PCD_StateTypeDef    State;        /*!< PCD communication state            */
    uint32_t                 Setup[12];    /*!< Setup packet buffer                */
    PCD_LPM_StateTypeDef     LPM_State;    /*!< LPM State                          */
    uint32_t                 BESL;
    uint32_t                 lpm_active;   /*!< Enable or disable the Link Power Management.
                                          This parameter can be set to ENABLE or DISABLE */

    uint32_t battery_charging_active;     /*!< Enable or disable Battery charging.
                                          This parameter can be set to ENABLE or DISABLE */
    void                     *pData;       /*!< Pointer to upper stack Handler */

    /* TODO: do we need this? */
#if 0
    uint8_t                  setupPacketBuffer[USB_SETUP_PACKET_SIZE * 2];
    stm32_usb_dev_ep_state_t ep_state[MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS) * 2];
    uint8_t                  setupBufferIndex;
    uint8_t                  otgStatus;
#endif
} stm32_usb_dev_state_t;

void usb_hal_init_clocks(void);
const usb_device_controller_interface_struct_t * usb_hal_controller_interface(void);
void usb_hal_set_dev_handle(void *);
void usb_hal_enable_irq(void);
void usb_hal_clear_memory(void);

#endif /* __HAL_USB_H__ */
