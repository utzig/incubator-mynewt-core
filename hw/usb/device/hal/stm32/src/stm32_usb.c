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

#include <usb/usb.h>
#include <device/device.h>
#include <hal_usb/hal_usb.h>

#include <syscfg/syscfg.h>
#include <os/os.h>

#include <cmsis_nvic.h>

//FIXME:
//
static PCD_HandleTypeDef g_hpcd;

static usb_device_khci_state_struct_t s_UsbDeviceKhciState;

static void
_init_cb_message_with_code(usb_device_callback_message_struct_t *message,
                           usb_device_notification_t code)
{
    message->buffer = NULL;
    message->code = code;
    message->length = 0;
    message->isSetup = 0;
}

#if 0 // FIXME: notify events in interrupt
    message.isSetup = isSetup;
    message.code = endpoint | (uint8_t)(((uint32_t)direction << 0x07));
    usb_dev_notify(state->deviceHandle, &message);
#endif

/*
 * Handle the USB bus reset interrupt.
 */
static void
USB_DeviceKhciInterruptReset(usb_device_khci_state_struct_t *state)
{
    usb_device_callback_message_struct_t message;

    state->isResetting = 1;
    state->registers->ISTAT = USBx_ISTAT_USBRST;

#if MYNEWT_VAL(USB_KINETIS_LOW_POWER_MODE)
    state->registers->ISTAT = USBx_ISTAT_SLEEP;
    state->registers->USBCTRL &= ~USB_USBCTRL_SUSP_MASK;
#endif

    _init_cb_message_with_code(&message, kUSB_DeviceNotifyBusReset);
    usb_dev_notify(state->deviceHandle, &message);
}

/* The USB suspend and resume signals need to be detected and handled when the
 * low power or remote wakeup function enabled.
 */
#if MYNEWT_VAL(USB_KINETIS_LOW_POWER_MODE)
static void
USB_DeviceKhciInterruptSleep(usb_device_khci_state_struct_t *state)
{
    usb_device_callback_message_struct_t message;

    state->registers->INTEN |= USBx_ISTAT_RESUME;
    state->registers->USBTRC0 |= USB_USBTRC0_USBRESMEN_MASK;
    state->registers->USBCTRL |= USB_USBCTRL_SUSP_MASK;

    state->registers->INTEN &= ~((uint32_t)USBx_ISTAT_SLEEP);

    state->registers->ISTAT = USBx_ISTAT_SLEEP;
    state->registers->ISTAT = USBx_ISTAT_RESUME;

    _init_cb_message_with_code(&message, kUSB_DeviceNotifySuspend);
    usb_dev_notify(state->deviceHandle, &message);
}

static void
USB_DeviceKhciInterruptResume(usb_device_khci_state_struct_t *state)
{
    usb_device_callback_message_struct_t message;

    state->registers->USBCTRL &= ~USB_USBCTRL_SUSP_MASK;
    state->registers->INTEN |= USBx_ISTAT_SLEEP;
    state->registers->INTEN &= ~((uint32_t)USBx_ISTAT_RESUME);
    state->registers->USBTRC0 &= ~USB_USBTRC0_USBRESMEN_MASK;

    state->registers->ISTAT = USBx_ISTAT_RESUME;
    state->registers->ISTAT = USBx_ISTAT_SLEEP;

    _init_cb_message_with_code(&message, kUSB_DeviceNotifyResume);
    usb_dev_notify(state->deviceHandle, &message);
}
#endif /* USB_KINETIS_LOW_POWER_MODE */

#if defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && \
    defined(FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED)
static void
USB_DeviceKhciInterruptVbusRising(usb_device_khci_state_struct_t *state)
{
    usb_device_callback_message_struct_t message;

    state->registers->MISCCTRL &= ~USB_MISCCTRL_VREDG_EN_MASK;
    state->registers->MISCCTRL |= USB_MISCCTRL_VREDG_EN_MASK;

    _init_cb_message_with_code(&message, kUSB_DeviceNotifyAttach);
    usb_dev_notify(state->deviceHandle, &message);
}

static void
USB_DeviceKhciInterruptVbusFalling(usb_device_khci_state_struct_t *state)
{
    usb_device_callback_message_struct_t message;

    state->registers->MISCCTRL &= ~USB_MISCCTRL_VFEDG_EN_MASK;
    state->registers->MISCCTRL |= USB_MISCCTRL_VFEDG_EN_MASK;

    _init_cb_message_with_code(&message, kUSB_DeviceNotifyDetach);
    usb_dev_notify(state->deviceHandle, &message);
}
#endif /* USB_DEVICE_CONFIG_DETACH_ENABLE || FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED */

#if 0
void
USB_DeviceKhciInterruptSof(usb_device_khci_state_struct_t *state)
{
    state->registers->ISTAT = USBx_ISTAT_SOFTOK;
    state->registers->ISTAT = USBx_ISTAT_RESUME;
}
#endif

static void
USB_DeviceKhciInterruptStall(usb_device_khci_state_struct_t *state)
{
    while (state->registers->ISTAT & USBx_ISTAT_STALL) {
        state->registers->ISTAT = USBx_ISTAT_STALL;
    }

    /* Un-stall the control in and out pipe when the control in or out pipe stalled. */
    if ((state->endpointState[(USB_CONTROL_ENDPOINT << 1) |
                                  USB_IN].stateUnion.stateBitField.stalled) ||
        (state->endpointState[(USB_CONTROL_ENDPOINT << 1) | USB_OUT]
         .stateUnion.stateBitField.stalled)) {
        USB_DeviceKhciEndpointUnstall(state, USB_CONTROL_ENDPOINT |
             (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
        USB_DeviceKhciEndpointUnstall(state, USB_CONTROL_ENDPOINT |
             (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
    }
}

#if defined(USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING)
static void
USB_DeviceKhciInterruptError(usb_device_khci_state_struct_t *state)
{
    usb_device_callback_message_struct_t message;

    state->registers->ISTAT = (USBx_ISTAT_ERROR);

    _init_cb_message_with_code(&message, kUSB_DeviceNotifyError);
    usb_dev_notify(state->deviceHandle, &message);
}
#endif

static usb_status_t
stm32_usb_dev_init(uint8_t controllerId,
                   usb_device_handle handle,
                   usb_device_controller_handle *khandle)
{
    usb_device_khci_state_struct_t *state;
    uint32_t khci_base[] = USB_BASE_ADDRS;

    state = &s_UsbDeviceKhciState;
    state->controllerId = controllerId;

    //FIXME
    state->registers = (volatile USB_Type *) USB0_BASE;

    stm32_usb_dev_control(state, USB_DEV_CTRL_STOP, NULL);


    hpcd->State= HAL_PCD_STATE_READY;
    USB_DevDisconnect (hpcd->Instance);

    USB_DeviceKhciSetDefaultState(state);

    *khandle = state;
    state->deviceHandle = (usb_device_struct_t *)handle;

    return kStatus_USB_Success;
}

static usb_status_t
usb_stm32_deinit(usb_device_controller_handle handle)
{
    usb_device_khci_state_struct_t *state =
        (usb_device_khci_state_struct_t *)handle;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    state->registers->ISTAT = 0xFF;
    state->registers->INTEN &= ~0xFF;
    state->registers->ADDR = 0;

    state->registers->CTL = 0x00;
    state->registers->USBCTRL |= USB_USBCTRL_PDE_MASK |
                                 USB_USBCTRL_SUSP_MASK;

    return kStatus_USB_Success;
}

static usb_status_t
usb_stm32_send(usb_device_controller_handle handle,
               uint8_t  ep_addr,
               uint8_t  *buffer,
               uint32_t length)
{
    usb_device_khci_state_struct_t *state =
        (usb_device_khci_state_struct_t *)handle;
    uint32_t index = (USB_EP_NUMBER(endpointAddress) << 1) | USB_IN;
    usb_status_t error = kStatus_USB_Error;

    /* Save the tansfer information */
    if (!state->endpointState[index].stateUnion.stateBitField.transferring) {
        state->endpointState[index].transferDone = 0;
        state->endpointState[index].transferBuffer = buffer;
        state->endpointState[index].transferLength = length;
        state->endpointState[index].stateUnion.stateBitField.dmaAlign = 1;
    }

    /* Data length needs to less than max packet size in each call. */
    if (length >
        state->endpointState[index].stateUnion.stateBitField.maxPacketSize)
    {
        length =
            state->endpointState[index].stateUnion.stateBitField.
            maxPacketSize;
    }

    /* Send data when the device is not resetting. */
    if (!state->isResetting) {
        error = USB_DeviceKhciEndpointTransfer(state,
            USB_EP_NUMBER(endpointAddress), USB_IN,
            (uint8_t *)((uint32_t)state->endpointState[index].transferBuffer +
                        (uint32_t)state->endpointState[index].transferDone),
            length);
    }

    /* Prime a transfer to receive next setup packet if the dat length is zero in a control in endpoint. */
    if (!state->endpointState[index].transferDone && !length &&
            USB_EP_NUMBER(endpointAddress) == USB_CONTROL_ENDPOINT) {
        USB_DeviceKhciPrimeNextSetup(state);
    }
    return error;


    USB_OTG_EPTypeDef *ep;

    ep = &hpcd->IN_ep[ep_addr & 0x7F];

    ep->xfer_buff = pBuf;
    ep->xfer_len = len;
    ep->xfer_count = 0;
    ep->is_in = 1;
    ep->num = ep_addr & 0x7F;

    if (hpcd->Init.dma_enable) {
        ep->dma_addr = (uint32_t)pBuf;
    }

    if (!(ep_addr & 0x7F)) {
        USB_EP0StartXfer(hpcd->Instance, ep, hpcd->Init.dma_enable);
    } else {
        USB_EPStartXfer(hpcd->Instance, ep, hpcd->Init.dma_enable);
    }

    return HAL_OK;
}

/*!
 * @brief Receive data through a specified endpoint.
 *
 * This function Receives data through a specified endpoint.
 *
 * @param khciHandle      Pointer of the device KHCI handle.
 * @param endpointAddress Endpoint index.
 * @param buffer           The memory address to save the received data.
 * @param length           The data length want to be received.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value just means if the receiving request is successful or not; the transfer done is notified by the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
static usb_status_t
usb_stm32_recv(usb_device_controller_handle khciHandle,
               uint8_t endpointAddress,
               uint8_t *buffer,
               uint32_t length)
{
    usb_device_khci_state_struct_t *khciState =
        (usb_device_khci_state_struct_t *)khciHandle;
    uint32_t index = (USB_EP_NUMBER(endpointAddress) << 1) | USB_OUT;
    usb_status_t error = kStatus_USB_Error;

    if (!length && USB_EP_NUMBER(endpointAddress) == USB_CONTROL_ENDPOINT) {
        khciState->endpointState[index].stateUnion.stateBitField.transferring = 0;
        USB_DeviceKhciPrimeNextSetup(khciState);
    } else {
        /* Save the transfer information */
        if (!khciState->endpointState[index].stateUnion.stateBitField.transferring) {
            khciState->endpointState[index].transferDone = 0;
            khciState->endpointState[index].transferBuffer = buffer;
            khciState->endpointState[index].transferLength = length;
        }
        khciState->endpointState[index].stateUnion.stateBitField.dmaAlign = 1;

        /* Data length needs to less than max packet size in each call. */
        if (length >
            khciState->endpointState[index].stateUnion.stateBitField.maxPacketSize) {
            length = khciState->endpointState[index].stateUnion.stateBitField.
                    maxPacketSize;
        }

        buffer = (uint8_t *)(
                (uint32_t)buffer +
                (uint32_t)khciState->endpointState[index].transferDone);

        if (khciState->dmaAlignBuffer && !khciState->isDmaAlignBufferInusing &&
            (USB_DEVICE_CONFIG_KHCI_DMA_ALIGN_BUFFER_LENGTH >= length) &&
            ((length & 0x03) || (((uint32_t)buffer) & 0x03))) {
            khciState->endpointState[index].stateUnion.stateBitField.dmaAlign = 0;
            buffer = khciState->dmaAlignBuffer;
            khciState->isDmaAlignBufferInusing = 1;
        }

        /* Receive data when the device is not resetting. */
        if (!khciState->isResetting) {
            error = USB_DeviceKhciEndpointTransfer(khciState,
                                                   USB_EP_NUMBER(endpointAddress),
                                                   USB_OUT, buffer, length);
        }
    }
    return error;
}

/*!
 * The function is used to cancel the pending transfer in a specified endpoint.
 *
 * @param khciHandle      Pointer of the device KHCI handle.
 * @param ep               Endpoint address, bit7 is the direction of endpoint, 1U - IN, abd 0U - OUT.
 */
static usb_status_t
usb_stm32_cancel(usb_device_controller_handle khciHandle, uint8_t ep)
{
    usb_device_khci_state_struct_t *khciState =
        (usb_device_khci_state_struct_t *)khciHandle;
    usb_device_callback_message_struct_t message;
    //FIXME: should not be USB_EP_DIR???
    uint8_t index = (USB_EP_NUMBER(ep) << 1) | USB_EP_DIR(ep);

    /* Cancel the transfer and notify the up layer when the endpoint is busy. */
    if (khciState->endpointState[index].stateUnion.stateBitField.transferring)
    {
        message.length = USB_UNINITIALIZED_VAL_32;
        message.buffer = khciState->endpointState[index].transferBuffer;
        message.code = ep;
        message.isSetup = 0;
        khciState->endpointState[index].stateUnion.stateBitField.transferring = 0;
        usb_dev_notify(khciState->deviceHandle, &message);
    }
    return kStatus_USB_Success;
}

static usb_status_t
stm32_usb_dev_control(usb_device_controller_handle handle,
                     usb_device_control_type_t type, void *param)
{
    //TODO

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    switch (type) {
    case USB_DEV_CTRL_RUN:
        error = kStatus_USB_Success;
        break;
    case USB_DEV_CTRL_STOP:
        error = kStatus_USB_Success;
        break;
    case USB_DEV_CTRL_EP_INIT:
        if (param) {
            error = stm32_usb_dev_ep_init(state,
                (usb_device_endpoint_init_struct_t *) param);
            //printf("endpoint_init %d\n", error);
        }
        break;
    case USB_DEV_CTRL_EP_DEINIT:
        if (param) {
            temp8 = (uint8_t *)param;
            error = USB_DeviceKhciEndpointDeinit(state, *temp8);
        }
        break;
    case USB_DEV_CTRL_EP_STALL:
        if (param) {
            temp8 = (uint8_t *)param;
            error = stm32_usb_dev_ep_stall(state, *temp8);
        }
        break;
    case USB_DEV_CTRL_EP_UNSTALL:
        if (param) {
            temp8 = (uint8_t *)param;
            error = USB_DeviceKhciEndpointUnstall(state, *temp8);
        }
        break;
    case USB_DEV_CTRL_GET_STATUS:
        if (param) {
            temp16 = (uint16_t *)param;
            *temp16 =
                (MYNEWT_VAL(USB_DEVICE_CONFIG_SELF_POWER) <<
                 (USB_REQUEST_STANDARD_GET_STATUS_DEVICE_SELF_POWERED_SHIFT))
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
                | ((uint16_t)(((uint32_t)deviceHandle->remotewakeup)
                                    << (
                                  USB_REQUEST_STANDARD_GET_STATUS_DEVICE_REMOTE_WARKUP_SHIFT)))
#endif
            ;
            error = kStatus_USB_Success;
        }
        break;
    case USB_DEV_CTRL_GET_EP_STATUS:
        if (param) {
            usb_device_endpoint_status_struct_t *endpointStatus =
                (usb_device_endpoint_status_struct_t *)param;

            if (USB_EP_NUMBER(endpointStatus->endpointAddress) <
                    MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
                endpointStatus->endpointStatus = (uint16_t)(
                    state->endpointState[
                        (USB_EP_NUMBER(endpointStatus->endpointAddress) << 1)
                        | USB_EP_DIR(endpointStatus->endpointAddress)]
                        .stateUnion.stateBitField.stalled == 1) ?
                    kUSB_DeviceEndpointStateStalled :
                    kUSB_DeviceEndpointStateIdle;
                error = kStatus_USB_Success;
            }
        }
        break;
    case USB_DEV_CTRL_SET_ADDR:
        if (param) {
            temp8 = (uint8_t *)param;
            state->registers->ADDR = *temp8;
            error = kStatus_USB_Success;
        }
        break;
    case USB_DEV_CTRL_GET_SYNCF:
        break;
#if MYNEWT_VAL(USB_KINETIS_LOW_POWER_MODE)
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    case USB_DEV_CTRL_RESUME:
        state->registers->CTL |= USB_CTL_RESUME_MASK;
        startTick = deviceHandle->hwTick;
        while ((deviceHandle->hwTick - startTick) < 10) {
            __ASM("nop");
        }
        state->registers->CTL &= ~USB_CTL_RESUME_MASK;
        error = kStatus_USB_Success;
        break;
#endif /* USB_DEVICE_CONFIG_REMOTE_WAKEUP */
    case USB_DEV_CTRL_SUSPEND:
        error = kStatus_USB_Success;
        break;
#endif /* USB_KINETIS_LOW_POWER_MODE */
    case USB_DEV_CTRL_SET_DFLT_STATUS:
        for (uint8_t count = 0; count < MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS); count++) {
            USB_DeviceKhciEndpointDeinit(state, count | (USB_IN << 7));
            USB_DeviceKhciEndpointDeinit(state, count | (USB_OUT << 7));
        }
        USB_DeviceKhciSetDefaultState(state);
        error = kStatus_USB_Success;
        break;
    case USB_DEV_CTRL_GET_SPEED:
        if (param) {
            temp8 = (uint8_t *)param;
            *temp8 = USB_SPEED_FULL;
            error = kStatus_USB_Success;
        }
        break;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_OTG)
    case USB_DEV_CTRL_GET_OTG_STATUS:
        *((uint8_t *)param) = state->otgStatus;
        break;
    case USB_DEV_CTRL_SET_OTG_STATUS:
        state->otgStatus = *((uint8_t *)param);
        break;
#endif
    case USB_DEV_CTRL_SET_TEST_MODE:
        break;
    default:
        break;
    }

    return error;
}

void
USB0_IRQHandler(void)
{
    stm32_usb_dev_isr();
}

static const usb_device_controller_interface_struct_t stm32_usb_interface = {
    stm32_usb_dev_init,
    stm32_usb_dev_deinit,
    stm32_usb_dev_send,
    stm32_usb_dev_recv,
    stm32_usb_dev_cancel,
    stm32_usb_dev_control,
};

usb_status_t
USB_DeviceErrorNotification(usb_device_struct_t *handle,
                            usb_device_callback_message_struct_t *message)
{
    return handle->devcb(handle, kUSB_DeviceEventError, NULL);
}

void
usb_hal_init_clocks(void)
{
    //TODO
}

const usb_device_controller_interface_struct_t *
usb_hal_controller_interface(void)
{
    return &stm32_usb_interface;
}

void
usb_hal_set_dev_handle(void *handle)
{
    g_handle = handle;
}

void
usb_hal_enable_irq(void)
{
    /* FIXME: make interrupt priority configurable */
    NVIC_SetPriority(USB0_IRQn, 3);
    NVIC_SetVector(USB0_IRQn, (uint32_t)USB0_IRQHandler);
    NVIC_EnableIRQ(USB0_IRQn);
}

void
usb_hal_disable_irq(void)
{
    //TODO: USB_OTG_GlobalTypeDef *USBx
    USB_DisableGlobalInt(*USBx);
}

void
usb_hal_clear_memory(void)
{
    /* Not needed for kinetis */
}




/*
 *
 * FIXME: section copied from hal_pcd driver...
 *
 */

typedef enum 
{
    HAL_PCD_STATE_RESET   = 0x00U,
    HAL_PCD_STATE_READY   = 0x01U,
    HAL_PCD_STATE_ERROR   = 0x02U,
    HAL_PCD_STATE_BUSY    = 0x03U,
    HAL_PCD_STATE_TIMEOUT = 0x04U
} PCD_StateTypeDef;

/* Device LPM suspend state */
typedef enum
{
    LPM_L0 = 0x00U, /* on */
    LPM_L1 = 0x01U, /* LPM L1 sleep */
    LPM_L2 = 0x02U, /* suspend */
    LPM_L3 = 0x03U, /* off */
} PCD_LPM_StateTypeDef;

//typedef USB_OTG_GlobalTypeDef  PCD_TypeDef;
//typedef USB_OTG_CfgTypeDef     PCD_InitTypeDef;
//typedef USB_OTG_EPTypeDef      PCD_EPTypeDef ;                          

typedef struct
{
    USB_OTG_GlobalTypeDef   *Instance;    /*!< Register base address              */
    USB_OTG_CfgTypeDef      Init;         /*!< PCD required parameters            */
    USB_OTG_EPTypeDef       IN_ep[16];    /*!< IN endpoint parameters             */
    USB_OTG_EPTypeDef       OUT_ep[16];   /*!< OUT endpoint parameters            */
    /* FIXME */
    //HAL_LockTypeDef         Lock;         /*!< PCD peripheral status              */
    struct os_mutex         lock;
    __IO PCD_StateTypeDef   State;        /*!< PCD communication state            */
    uint32_t                Setup[12];    /*!< Setup packet buffer                */
    PCD_LPM_StateTypeDef    LPM_State;    /*!< LPM State                          */
    uint32_t                BESL;
    uint32_t                lpm_active;   /*!< Enable or disable the Link Power Management.
                                          This parameter can be set to ENABLE or DISABLE */

    uint32_t battery_charging_active;     /*!< Enable or disable Battery charging.
                                          This parameter can be set to ENABLE or DISABLE */
    void                    *pData;       /*!< Pointer to upper stack Handler */
} PCD_HandleTypeDef;

#define PCD_SPEED_HIGH               0U
#define PCD_SPEED_HIGH_IN_FULL       1U
#define PCD_SPEED_FULL               2U

#define PCD_PHY_ULPI                 1U
#define PCD_PHY_EMBEDDED             2U
#define PCD_PHY_UTMI                 3U

#ifndef USBD_HS_TRDT_VALUE
#define USBD_HS_TRDT_VALUE           9U
#endif
#ifndef USBD_FS_TRDT_VALUE
#define USBD_FS_TRDT_VALUE           5U
#endif

#define __HAL_PCD_UNGATE_PHYCLOCK(__HANDLE__)             *(__IO uint32_t *)((uint32_t)((__HANDLE__)->Instance) + USB_OTG_PCGCCTL_BASE) &= ~(USB_OTG_PCGCCTL_STOPCLK)
#define __HAL_PCD_GATE_PHYCLOCK(__HANDLE__)               *(__IO uint32_t *)((uint32_t)((__HANDLE__)->Instance) + USB_OTG_PCGCCTL_BASE) |= USB_OTG_PCGCCTL_STOPCLK

#define __HAL_PCD_IS_PHY_SUSPENDED(__HANDLE__)            ((*(__IO uint32_t *)((uint32_t)((__HANDLE__)->Instance) + USB_OTG_PCGCCTL_BASE))&0x10)

#define USB_OTG_FS_WAKEUP_EXTI_RISING_EDGE                ((uint32_t)0x08U)
#define USB_OTG_FS_WAKEUP_EXTI_FALLING_EDGE               ((uint32_t)0x0CU)
#define USB_OTG_FS_WAKEUP_EXTI_RISING_FALLING_EDGE        ((uint32_t)0x10U)

#define USB_OTG_HS_WAKEUP_EXTI_RISING_EDGE                ((uint32_t)0x08U)
#define USB_OTG_HS_WAKEUP_EXTI_FALLING_EDGE               ((uint32_t)0x0CU)
#define USB_OTG_HS_WAKEUP_EXTI_RISING_FALLING_EDGE        ((uint32_t)0x10U)

#define USB_OTG_HS_WAKEUP_EXTI_LINE                       ((uint32_t)0x00100000U)  /*!< External interrupt line 20 Connected to the USB HS EXTI Line */
#define USB_OTG_FS_WAKEUP_EXTI_LINE                       ((uint32_t)0x00040000U)  /*!< External interrupt line 18 Connected to the USB FS EXTI Line */

#define __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_IT()    EXTI->IMR |= (USB_OTG_HS_WAKEUP_EXTI_LINE)
#define __HAL_USB_OTG_HS_WAKEUP_EXTI_DISABLE_IT()   EXTI->IMR &= ~(USB_OTG_HS_WAKEUP_EXTI_LINE)
#define __HAL_USB_OTG_HS_WAKEUP_EXTI_GET_FLAG()     EXTI->PR & (USB_OTG_HS_WAKEUP_EXTI_LINE)
#define __HAL_USB_OTG_HS_WAKEUP_EXTI_CLEAR_FLAG()   EXTI->PR = (USB_OTG_HS_WAKEUP_EXTI_LINE)

#define __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_RISING_EDGE() EXTI->FTSR &= ~(USB_OTG_HS_WAKEUP_EXTI_LINE);\
                                                          EXTI->RTSR |= USB_OTG_HS_WAKEUP_EXTI_LINE

#define __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_FALLING_EDGE()  EXTI->FTSR |= (USB_OTG_HS_WAKEUP_EXTI_LINE);\
                                                            EXTI->RTSR &= ~(USB_OTG_HS_WAKEUP_EXTI_LINE)

#define __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_RISING_FALLING_EDGE()   EXTI->RTSR &= ~(USB_OTG_HS_WAKEUP_EXTI_LINE);\
                                                                    EXTI->FTSR &= ~(USB_OTG_HS_WAKEUP_EXTI_LINE;)\
                                                                    EXTI->RTSR |= USB_OTG_HS_WAKEUP_EXTI_LINE;\
                                                                    EXTI->FTSR |= USB_OTG_HS_WAKEUP_EXTI_LINE

#define __HAL_USB_OTG_HS_WAKEUP_EXTI_GENERATE_SWIT()   (EXTI->SWIER |= USB_OTG_FS_WAKEUP_EXTI_LINE) 

#define __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_IT()    EXTI->IMR |= USB_OTG_FS_WAKEUP_EXTI_LINE
#define __HAL_USB_OTG_FS_WAKEUP_EXTI_DISABLE_IT()   EXTI->IMR &= ~(USB_OTG_FS_WAKEUP_EXTI_LINE)
#define __HAL_USB_OTG_FS_WAKEUP_EXTI_GET_FLAG()     EXTI->PR & (USB_OTG_FS_WAKEUP_EXTI_LINE)
#define __HAL_USB_OTG_FS_WAKEUP_EXTI_CLEAR_FLAG()   EXTI->PR = USB_OTG_FS_WAKEUP_EXTI_LINE

#define __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_RISING_EDGE() EXTI->FTSR &= ~(USB_OTG_FS_WAKEUP_EXTI_LINE);\
                                                          EXTI->RTSR |= USB_OTG_FS_WAKEUP_EXTI_LINE


#define __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_FALLING_EDGE()  EXTI->FTSR |= (USB_OTG_FS_WAKEUP_EXTI_LINE);\
                                                            EXTI->RTSR &= ~(USB_OTG_FS_WAKEUP_EXTI_LINE)

#define __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_RISING_FALLING_EDGE()  EXTI->RTSR &= ~(USB_OTG_FS_WAKEUP_EXTI_LINE);\
                                                                   EXTI->FTSR &= ~(USB_OTG_FS_WAKEUP_EXTI_LINE);\
                                                                   EXTI->RTSR |= USB_OTG_FS_WAKEUP_EXTI_LINE;\
                                                                   EXTI->FTSR |= USB_OTG_FS_WAKEUP_EXTI_LINE 

#define __HAL_USB_OTG_FS_WAKEUP_EXTI_GENERATE_SWIT()  (EXTI->SWIER |= USB_OTG_FS_WAKEUP_EXTI_LINE)

/* FIXME: from .c code */

HAL_StatusTypeDef stm32_usb_dev_init(PCD_HandleTypeDef *hpcd)
{
    uint32_t i = 0;

    if (!hpcd) {
        return HAL_ERROR;
    }

    assert(hpcd->Instance == USB_OTG_FS || hpcd->Instance == USB_OTG_HS);

    hpcd->State = HAL_PCD_STATE_BUSY;

    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    HAL_PCD_MspInit(hpcd);

    USB_DisableGlobalInt(hpcd->Instance);
    USB_CoreInit(hpcd->Instance, hpcd->Init);
    USB_SetCurrentMode(hpcd->Instance , USB_OTG_DEVICE_MODE);

    for (i = 0; i < 15 ; i++)
    {
        hpcd->IN_ep[i].is_in = 1;
        hpcd->IN_ep[i].num = i;
        hpcd->IN_ep[i].tx_fifo_num = i;

        hpcd->IN_ep[i].type = EP_TYPE_CTRL;
        hpcd->IN_ep[i].maxpacket =  0;
        hpcd->IN_ep[i].xfer_buff = 0;
        hpcd->IN_ep[i].xfer_len = 0;

        hpcd->OUT_ep[i].is_in = 0;
        hpcd->OUT_ep[i].num = i;
        //FIXME: ?
        hpcd->IN_ep[i].tx_fifo_num = i;

        hpcd->OUT_ep[i].type = EP_TYPE_CTRL;
        hpcd->OUT_ep[i].maxpacket = 0;
        hpcd->OUT_ep[i].xfer_buff = 0;
        hpcd->OUT_ep[i].xfer_len = 0;

        hpcd->Instance->DIEPTXF[i] = 0;
    }

    USB_DevInit(hpcd->Instance, hpcd->Init);

    hpcd->State= HAL_PCD_STATE_READY;

    if (hpcd->Init.lpm_enable == 1) {
        HAL_PCDEx_ActivateLPM(hpcd);
    }

#if defined (USB_OTG_GCCFG_BCDEN)
    /* Activate Battery charging */
    if (hpcd->Init.battery_charging_enable ==1) {
        HAL_PCDEx_ActivateBCD(hpcd);
    }
#endif /* USB_OTG_GCCFG_BCDEN */

    USB_DevDisconnect(hpcd->Instance);
    return HAL_OK;
}

HAL_StatusTypeDef
HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd)
{
    if (!hpcd) {
        return HAL_ERROR;
    }

    hpcd->State = HAL_PCD_STATE_BUSY;

    HAL_PCD_Stop(hpcd);
    HAL_PCD_MspDeInit(hpcd);

    hpcd->State = HAL_PCD_STATE_RESET;

    return HAL_OK;
}

__weak void
HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    UNUSED(hpcd);
}

__weak void
HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    UNUSED(hpcd);
}

HAL_StatusTypeDef
HAL_PCD_Start(PCD_HandleTypeDef *hpcd)
{
    __HAL_LOCK(hpcd);
    USB_DevConnect (hpcd->Instance);
    USB_EnableGlobalInt(hpcd->Instance);
    __HAL_UNLOCK(hpcd);
    return HAL_OK;
}

HAL_StatusTypeDef
HAL_PCD_Stop(PCD_HandleTypeDef *hpcd)
{
    __HAL_LOCK(hpcd);
    USB_DisableGlobalInt(hpcd->Instance);
    USB_StopDevice(hpcd->Instance);
    USB_DevDisconnect (hpcd->Instance);
    __HAL_UNLOCK(hpcd);
    return HAL_OK;
}

#define GET_FLAG(HANDLE, INT) ((USB_ReadInterrupts((HANDLE)->Instance) & (INT)) == (INT))
#define CLR_FLAG(HANDLE, INT) (((HANDLE)->Instance->GINTSTS) = (INT))

static void
stm32_usb_dev_isr(PCD_HandleTypeDef *hpcd)
{
    USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
    uint32_t i = 0, ep_intr = 0, epint = 0, epnum = 0;
    uint32_t fifoemptymsk = 0, temp = 0;
    USB_OTG_EPTypeDef *ep = NULL;
    uint32_t hclk = 200000000;

    if (USB_GetMode(hpcd->Instance) == USB_OTG_MODE_DEVICE) {
        if (!USB_ReadInterrupts(hpcd->Instance)) {
            return;
        }

        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_MMIS)) {
            /* incorrect mode, acknowledge the interrupt */
            CLR_FLAG(hpcd, USB_OTG_GINTSTS_MMIS);
        }

        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_OEPINT)) {
            epnum = 0;

            /* Read in the device interrupt bits */
            ep_intr = USB_ReadDevAllOutEpInterrupt(hpcd->Instance);

            while (ep_intr) {
                if (ep_intr & 1) {
                    epint = USB_ReadDevOutEPInterrupt(hpcd->Instance, epnum);

                    if ((epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC) {
                        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_XFRC);

                        if (hpcd->Init.dma_enable == 1) {
                            hpcd->OUT_ep[epnum].xfer_count = hpcd->OUT_ep[epnum].maxpacket - (USBx_OUTEP(epnum)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ);
                            hpcd->OUT_ep[epnum].xfer_buff += hpcd->OUT_ep[epnum].maxpacket;
                        }

                        HAL_PCD_DataOutStageCallback(hpcd, epnum);
                        if (hpcd->Init.dma_enable == 1) {
                            if ((epnum == 0) && (hpcd->OUT_ep[epnum].xfer_len == 0)) {
                                /* this is ZLP, so prepare EP0 for next setup */
                                USB_EP0_OutStart(hpcd->Instance, 1, (uint8_t *)hpcd->Setup);
                            }
                        }
                    }

                    if ((epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP) {
                        /* Inform the upper layer that a setup packet is available */
                        HAL_PCD_SetupStageCallback(hpcd);
                        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STUP);
                    }

                    if ((epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS) {
                        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPDIS);
                    }

                    /* Clear Status Phase Received interrupt */
                    if ((epint & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR) {
                        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
                    }
                }
                epnum++;
                ep_intr >>= 1;
            }
        }

        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_IEPINT)) {
            /* Read in the device interrupt bits */
            ep_intr = USB_ReadDevAllInEpInterrupt(hpcd->Instance);

            epnum = 0;

            while (ep_intr) {
                if (ep_intr & 0x1) {
                    epint = USB_ReadDevInEPInterrupt(hpcd->Instance, epnum);

                    if (( epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC) {
                        fifoemptymsk = 1 << epnum;
                        USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;

                        CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_XFRC);

                        if (hpcd->Init.dma_enable == 1) {
                            hpcd->IN_ep[epnum].xfer_buff += hpcd->IN_ep[epnum].maxpacket;
                        }

                        HAL_PCD_DataInStageCallback(hpcd, epnum);

                        if (hpcd->Init.dma_enable == 1) {
                            /* this is ZLP, so prepare EP0 for next setup */
                            if ((epnum == 0) && (hpcd->IN_ep[epnum].xfer_len == 0)) {
                                /* prepare to rx more setup packets */
                                USB_EP0_OutStart(hpcd->Instance, 1, (uint8_t *)hpcd->Setup);
                            }
                        }
                    }

                    if ((epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC) {
                        CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_TOC);
                    }
                    if ((epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE) {
                        CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_ITTXFE);
                    }
                    if ((epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE) {
                        CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_INEPNE);
                    }
                    if ((epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD) {
                        CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_EPDISD);
                    }
                    if ((epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE) {
                        PCD_WriteEmptyTxFifo(hpcd , epnum);
                    }
                }
                epnum++;
                ep_intr >>= 1;
            }
        }

        /* Handle Resume Interrupt */
        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_WKUINT)) {
            /* Clear the Remote Wake-up Signaling */
            USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;

            if (hpcd->LPM_State == LPM_L1) {
                hpcd->LPM_State = LPM_L0;
                HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L0_ACTIVE);
            } else {
                HAL_PCD_ResumeCallback(hpcd);
            }
            CLR_FLAG(hpcd, USB_OTG_GINTSTS_WKUINT);
        }

        /* Handle Suspend Interrupt */
        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_USBSUSP)) {
            if ((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS) {
                HAL_PCD_SuspendCallback(hpcd);
            }
            CLR_FLAG(hpcd, USB_OTG_GINTSTS_USBSUSP);
        }

        /* Handle LPM Interrupt */
        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_LPMINT)) {
            CLR_FLAG(hpcd, USB_OTG_GINTSTS_LPMINT);
            if (hpcd->LPM_State == LPM_L0) {
                hpcd->LPM_State = LPM_L1;
                hpcd->BESL = (hpcd->Instance->GLPMCFG & USB_OTG_GLPMCFG_BESL) >> 2;
                HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L1_ACTIVE);
            } else {
                HAL_PCD_SuspendCallback(hpcd);
            }
        }

        /* Handle Reset Interrupt */
        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_USBRST)) {
            USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
            USB_FlushTxFifo(hpcd->Instance, 0x10);

            for (i = 0; i < hpcd->Init.dev_endpoints ; i++) {
                USBx_INEP(i)->DIEPINT = 0xFF;
                USBx_OUTEP(i)->DOEPINT = 0xFF;
            }
            USBx_DEVICE->DAINT = 0xFFFFFFFF;
            USBx_DEVICE->DAINTMSK |= 0x10001;

            if (hpcd->Init.use_dedicated_ep1) {
                USBx_DEVICE->DOUTEP1MSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM);
                USBx_DEVICE->DINEP1MSK |= (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);
            } else {
                USBx_DEVICE->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM | USB_OTG_DOEPMSK_OTEPSPRM);
                USBx_DEVICE->DIEPMSK |= (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);
            }

            /* Set Default Address to 0 */
            USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

            /* setup EP0 to receive SETUP packets */
            USB_EP0_OutStart(hpcd->Instance, hpcd->Init.dma_enable, (uint8_t *)hpcd->Setup);

            CLR_FLAG(hpcd, USB_OTG_GINTSTS_USBRST);
        }

        /* Handle Enumeration done Interrupt */
        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_ENUMDNE)) {
            USB_ActivateSetup(hpcd->Instance);
            hpcd->Instance->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;

            if (USB_GetDevSpeed(hpcd->Instance) == USB_OTG_SPEED_HIGH) {
                hpcd->Init.speed = USB_OTG_SPEED_HIGH;
                hpcd->Init.ep0_mps = USB_OTG_HS_MAX_PACKET_SIZE;
                hpcd->Instance->GUSBCFG |= (uint32_t)((USBD_HS_TRDT_VALUE << 10) & USB_OTG_GUSBCFG_TRDT);
            } else {
                hpcd->Init.speed = USB_OTG_SPEED_FULL;
                hpcd->Init.ep0_mps = USB_OTG_FS_MAX_PACKET_SIZE;

                /* The USBTRD is configured according to the tables below, depending on AHB frequency 
                used by application. In the low AHB frequency range it is used to stretch enough the USB response 
                time to IN tokens, the USB turnaround time, so to compensate for the longer AHB read access 
                latency to the Data FIFO */

                /* Get hclk frequency value */
                hclk = HAL_RCC_GetHCLKFreq();

                if (hclk < 14200000) {
                    assert (0);
                } else if (hclk < 15000000) {
                    hpcd->Instance->GUSBCFG |= (uint32_t)((0xF << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 16000000) {
                    hpcd->Instance->GUSBCFG |= (uint32_t)((0xE << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 17200000) {
                    hpcd->Instance->GUSBCFG |= (uint32_t)((0xD << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 18500000) {
                    hpcd->Instance->GUSBCFG |= (uint32_t)((0xC << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 20000000) {
                    hpcd->Instance->GUSBCFG |= (uint32_t)((0xB << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 21800000) {
                    hpcd->Instance->GUSBCFG |= (uint32_t)((0xA << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 24000000) {
                    hpcd->Instance->GUSBCFG |= (uint32_t)((0x9 << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 27700000) {
                    hpcd->Instance->GUSBCFG |= (uint32_t)((0x8 << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 32000000) {
                    hpcd->Instance->GUSBCFG |= (uint32_t)((0x7 << 10) & USB_OTG_GUSBCFG_TRDT);
                } else {
                    hpcd->Instance->GUSBCFG |= (uint32_t)((0x6 << 10) & USB_OTG_GUSBCFG_TRDT);
                }
            }

            HAL_PCD_ResetCallback(hpcd);

            CLR_FLAG(hpcd, USB_OTG_GINTSTS_ENUMDNE);
        }

        /* Handle RxQLevel Interrupt */
        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_RXFLVL)) {
            USB_MASK_INTERRUPT(hpcd->Instance, USB_OTG_GINTSTS_RXFLVL);
            temp = USBx->GRXSTSP;
            ep = &hpcd->OUT_ep[temp & USB_OTG_GRXSTSP_EPNUM];

            if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_DATA_UPDT) {
                if (temp & USB_OTG_GRXSTSP_BCNT) {
                    USB_ReadPacket(USBx, ep->xfer_buff, (temp & USB_OTG_GRXSTSP_BCNT) >> 4);
                    ep->xfer_buff += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
                    ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
                }
            } else if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_SETUP_UPDT) {
                USB_ReadPacket(USBx, (uint8_t *)hpcd->Setup, 8);
                ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
            }
            USB_UNMASK_INTERRUPT(hpcd->Instance, USB_OTG_GINTSTS_RXFLVL);
        }

        /* Handle SOF Interrupt */
        if(ET_FLAG(hpcd, USB_OTG_GINTSTS_SOF)) {
            HAL_PCD_SOFCallback(hpcd);
            CLR_FLAG(hpcd, USB_OTG_GINTSTS_SOF);
        }

        /* Handle Incomplete ISO IN Interrupt */
        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_IISOIXFR)) {
            HAL_PCD_ISOINIncompleteCallback(hpcd, epnum);
            CLR_FLAG(hpcd, USB_OTG_GINTSTS_IISOIXFR);
        }

        /* Handle Incomplete ISO OUT Interrupt */
        if(GET_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)) {
            HAL_PCD_ISOOUTIncompleteCallback(hpcd, epnum);
            CLR_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
        }

        /* Handle Connection event Interrupt */
        if(GET_FLAG(hpcd, USB_OTG_GINTSTS_SRQINT)) {
            HAL_PCD_ConnectCallback(hpcd);
            CLR_FLAG(hpcd, USB_OTG_GINTSTS_SRQINT);
        }

        /* Handle Disconnection event Interrupt */
        if (GET_FLAG(hpcd, USB_OTG_GINTSTS_OTGINT)) {
            temp = hpcd->Instance->GOTGINT;

            if ((temp & USB_OTG_GOTGINT_SEDET) == USB_OTG_GOTGINT_SEDET) {
                HAL_PCD_DisconnectCallback(hpcd);
            }
            hpcd->Instance->GOTGINT |= temp;
        }
    }
}

__weak void
HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    UNUSED(hpcd);
    UNUSED(epnum);
}

__weak void
HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    UNUSED(hpcd);
    UNUSED(epnum);
}

__weak void
HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    UNUSED(hpcd);
}

__weak void
HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    UNUSED(hpcd);
}

__weak void
HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    UNUSED(hpcd);
}

__weak void
HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    UNUSED(hpcd);
}

__weak void
HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    UNUSED(hpcd);
}

__weak void
HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    UNUSED(hpcd);
    UNUSED(epnum);
}

__weak void
HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    UNUSED(hpcd);
    UNUSED(epnum);
}

__weak void
HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    UNUSED(hpcd);
}

__weak void
HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    UNUSED(hpcd);
}

HAL_StatusTypeDef
HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd)
{
    __HAL_LOCK(hpcd);
    USB_DevConnect(hpcd->Instance);
    __HAL_UNLOCK(hpcd);
    return HAL_OK;
}

HAL_StatusTypeDef
HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd)
{
    __HAL_LOCK(hpcd);
    USB_DevDisconnect(hpcd->Instance);
    __HAL_UNLOCK(hpcd);
    return HAL_OK;
}

HAL_StatusTypeDef
HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address)
{
    __HAL_LOCK(hpcd);
    USB_SetDevAddress(hpcd->Instance, address);
    __HAL_UNLOCK(hpcd);
    return HAL_OK;
}



#if 0
static int
USB_DeviceKhciEndpointInit(usb_device_khci_state_struct_t *state,
                           usb_device_endpoint_init_struct_t *epInit)
{
    return 0;
}
#endif

int
stm32_usb_dev_ep_init(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type)
{
    USB_OTG_EPTypeDef *ep;

    if (ep_addr & 0x80) {
        ep = &hpcd->IN_ep[ep_addr & 0x7f];
    } else {
        ep = &hpcd->OUT_ep[ep_addr];
    }
    ep->num = ep_addr & 0x7f;

    ep->is_in = (0x80 & ep_addr) != 0;
    ep->maxpacket = ep_mps;
    ep->type = ep_type;
    if (ep->is_in) {
        ep->tx_fifo_num = ep->num;
    }

    if (ep_type == EP_TYPE_BULK) {
        ep->data_pid_start = 0;
    }

    __HAL_LOCK(hpcd);
    USB_ActivateEndpoint(hpcd->Instance, ep);
    __HAL_UNLOCK(hpcd);
    return 0;
}

HAL_StatusTypeDef
HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    USB_OTG_EPTypeDef *ep;

    if (ep_addr & 0x80) {
        ep = &hpcd->IN_ep[ep_addr & 0x7f];
    } else {
        ep = &hpcd->OUT_ep[ep_addr];
    }
    ep->num   = ep_addr & 0x7F;

    ep->is_in = (0x80 & ep_addr) != 0;

    __HAL_LOCK(hpcd);
    USB_DeactivateEndpoint(hpcd->Instance, ep);
    __HAL_UNLOCK(hpcd);
    return HAL_OK;
}

static int
stm32_usb_dev_ep_recv(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *buf, uint32_t len)
{
    USB_OTG_EPTypeDef *ep;

    if (!buf) {
        return -1;
    }

    ep = &hpcd->OUT_ep[ep_addr & 0x7F];

    ep->xfer_buff = buf;
    ep->xfer_len = len;
    ep->xfer_count = 0;
    ep->is_in = 0;
    ep->num = ep_addr & 0x7f;

    if (hpcd->Init.dma_enable == 1) {
        ep->dma_addr = (uint32_t)buf;
    }

    if ((ep_addr & 0x7F) == 0) {
        USB_EP0StartXfer(hpcd->Instance, ep, hpcd->Init.dma_enable);
    } else {
        USB_EPStartXfer(hpcd->Instance, ep, hpcd->Init.dma_enable);
    }

    return 0;
}

HAL_StatusTypeDef
stm32_usb_dev_ep_send(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *buf, uint32_t len)
{
    USB_OTG_EPTypeDef *ep;

    if (!buf) {
        return -1;
    }

    ep = &hpcd->IN_ep[ep_addr & 0x7f];

    ep->xfer_buff = buf;
    ep->xfer_len = len;
    ep->xfer_count = 0;
    ep->is_in = 1;
    ep->num = ep_addr & 0x7F;

    if (hpcd->Init.dma_enable == 1) {
        ep->dma_addr = (uint32_t)buf;
    }

    if ((ep_addr & 0x7F) == 0) {
        USB_EP0StartXfer(hpcd->Instance, ep, hpcd->Init.dma_enable);
    } else {
        USB_EPStartXfer(hpcd->Instance, ep, hpcd->Init.dma_enable);
    }

    return 0;
}

uint16_t
HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    return hpcd->OUT_ep[ep_addr & 0x0f].xfer_count;
}

int
stm32_usb_dev_ep_stall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    USB_OTG_EPTypeDef *ep;

    /*
     * FIXME: need to cancel first?
     *
     * usb_stm32_cancel(state, ep);
     */

    if (0x80 & ep_addr) {
        ep = &hpcd->IN_ep[ep_addr & 0x7F];
    } else {
        ep = &hpcd->OUT_ep[ep_addr];
    }

    ep->is_stall = 1;
    ep->num = ep_addr & 0x7F;
    ep->is_in = (ep_addr & 0x80) == 0x80;

    __HAL_LOCK(hpcd);
    USB_EPSetStall(hpcd->Instance, ep);
    if((ep_addr & 0x7F) == 0) {
        USB_EP0_OutStart(hpcd->Instance, hpcd->Init.dma_enable, (uint8_t *)hpcd->Setup);
    }
    __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

HAL_StatusTypeDef
HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    USB_OTG_EPTypeDef *ep;

    if (ep_addr & 0x80) {
        ep = &hpcd->IN_ep[ep_addr & 0x7F];
    } else {
        ep = &hpcd->OUT_ep[ep_addr];
    }

    ep->is_stall = 0;
    ep->num = ep_addr & 0x7F;
    ep->is_in = (ep_addr & 0x80) == 0x80;

    __HAL_LOCK(hpcd);
    USB_EPClearStall(hpcd->Instance, ep);
    __HAL_UNLOCK(hpcd);

    return HAL_OK;
}

static int
stm32_usb_dev_ep_flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    __HAL_LOCK(hpcd);

    if (ep_addr & 0x80) {
        USB_FlushTxFifo(hpcd->Instance, ep_addr & 0x7F);
    } else {
        USB_FlushRxFifo(hpcd->Instance);
    }

    __HAL_UNLOCK(hpcd);

    return 0;
}

#if 0
HAL_StatusTypeDef
HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
    USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;

    if((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS) {
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_RWUSIG;
    }
    return HAL_OK;
}

HAL_StatusTypeDef
HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
    USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;

    USBx_DEVICE->DCTL &= ~(USB_OTG_DCTL_RWUSIG);
    return HAL_OK;
}
#endif

PCD_StateTypeDef
HAL_PCD_GetState(PCD_HandleTypeDef *hpcd)
{
    return hpcd->State;
}

static HAL_StatusTypeDef
PCD_WriteEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum)
{
    USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
    USB_OTG_EPTypeDef *ep;
    int32_t len = 0;
    uint32_t len32b;
    uint32_t fifoemptymsk = 0;

    ep = &hpcd->IN_ep[epnum];
    len = ep->xfer_len - ep->xfer_count;

    if (len > ep->maxpacket) {
        len = ep->maxpacket;
    }

    len32b = (len + 3) / 4;

    while ((USBx_INEP(epnum)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) > len32b &&
           ep->xfer_count < ep->xfer_len &&
           ep->xfer_len != 0) {

        len = ep->xfer_len - ep->xfer_count;

        if (len > ep->maxpacket) {
            len = ep->maxpacket;
        }
        len32b = (len + 3) / 4;

        USB_WritePacket(USBx, ep->xfer_buff, epnum, len, hpcd->Init.dma_enable);

        ep->xfer_buff += len;
        ep->xfer_count += len;
    }

    if (len <= 0) {
        fifoemptymsk = 0x1 << epnum;
        USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;
    }

    return HAL_OK;
}
