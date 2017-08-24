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
#include <dev/dev.h>
#include <hal_usb/hal_usb.h>

#include <syscfg/syscfg.h>
#include <os/os.h>
#include <assert.h>

#include <stm32f7xx_hal_rcc.h>
#include <stm32f7xx_hal_gpio.h>
#include <stm32f7xx_hal_pwr_ex.h>
#include <stm32f7xx_hal_flash_ex.h>
#include <cmsis_nvic.h>

//FIXME:
//
static stm32_usb_dev_state_t g_stm32_usb_dev_state;
static void *g_handle = NULL;

//FIXME: this can be removed?
typedef enum
{
    STATE_RESET   = 0x00,
    STATE_READY   = 0x01,
    STATE_BUSY    = 0x03,
} StateTypeDef;

static void
_stm32_usb_dev_msp_init(void)
{
    GPIO_InitTypeDef gpio;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure DM DP Pins */
    gpio.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Configure VBUS Pin */
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Configure ID pin */
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Enable USB FS Clock */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
}

static void
_stm32_usb_dev_msp_deinit(void)
{
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    __HAL_RCC_SYSCFG_CLK_DISABLE();
}

static int
_stm32_usb_dev_start(usb_dev_ctrl_handle handle)
{
    stm32_usb_dev_state_t *state = NULL;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    state = (stm32_usb_dev_state_t *)handle;

    __HAL_LOCK(state);
    USB_DevConnect(state->Instance);
    USB_EnableGlobalInt(state->Instance);
    __HAL_UNLOCK(state);

    return 0;
}

static int
_stm32_usb_dev_stop(usb_dev_ctrl_handle handle)
{
    stm32_usb_dev_state_t *state = (stm32_usb_dev_state_t *)handle;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    __HAL_LOCK(state);
    USB_DisableGlobalInt(state->Instance);
    USB_StopDevice(state->Instance);
    USB_DevDisconnect (state->Instance);
    __HAL_UNLOCK(state);

    return 0;
}

static void
_init_cb_msg_with_code(usb_dev_cb_msg_t *msg, usb_device_notification_t code)
{
    msg->buf = NULL;
    msg->len = 0;
    msg->code = code;
    msg->isSetup = 0;
}

#if 0
static void
_stm32_usb_dev_reset(stm32_usb_dev_state_t *state)
{
    usb_dev_cb_msg_t msg;

    state->isResetting = 1;

    /* TODO: mcu specific reset */

    _init_cb_msg_with_code(&msg, kUSB_DeviceNotifyBusReset);
    usb_dev_notify(state->dev, &msg);
}

#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
static void
_stm32_usb_dev_sleep(stm32_usb_dev_state_t *state)
{
    usb_dev_cb_msg_t msg;

    /* TODO: mcu code */

    _init_cb_msg_with_code(&msg, kUSB_DeviceNotifySuspend);
    usb_dev_notify(state->dev, &msg);
}

static void
_stm32_usb_dev_resume(stm32_usb_dev_state_t *state)
{
    usb_dev_cb_msg_t msg;

    /* TODO: mcu code */

    _init_cb_msg_with_code(&msg, kUSB_DeviceNotifyResume);
    usb_dev_notify(state->dev, &msg);
}
#endif
#endif

#if 0
static void
_stm32_usb_dev_sof(stm32_usb_dev_state_t *state)
{
    /* TODO: mcu code */
}
#endif

static int
_stm32_usb_dev_set_addr(stm32_usb_dev_state_t *state, uint8_t addr)
{
    //printf("set addr = 0x%02x\n", addr);
    __HAL_LOCK(state);
    USB_SetDevAddress(state->Instance, addr);
    __HAL_UNLOCK(state);
    return 0;
}

static int
_stm32_usb_dev_ep_init(stm32_usb_dev_state_t *state, usb_dev_ep_init_t *epInit)
{
    USB_OTG_EPTypeDef *ep;
    uint8_t ep_addr = epInit->endpointAddress;

    if (ep_addr & 0x80) {
        ep = &state->IN_ep[ep_addr & 0x7f];
    } else {
        ep = &state->OUT_ep[ep_addr];
    }
    ep->num = ep_addr & 0x7f;

    ep->is_in = (0x80 & ep_addr) != 0;
    ep->maxpacket = epInit->maxPacketSize;
    ep->type = epInit->transferType;
    if (ep->is_in) {
        ep->tx_fifo_num = ep->num;
    }

    if (ep->type == USB_ENDPOINT_BULK) {
        ep->data_pid_start = 0;
    }

    __HAL_LOCK(state);
    USB_ActivateEndpoint(state->Instance, ep);
    __HAL_UNLOCK(state);
    return 0;
}

static int
_stm32_usb_dev_ep_deinit(stm32_usb_dev_state_t *state,
                         uint8_t ep_addr)
{
    USB_OTG_EPTypeDef *ep;

    if (ep_addr & 0x80) {
        ep = &state->IN_ep[ep_addr & 0x7f];
    } else {
        ep = &state->OUT_ep[ep_addr];
    }

    __HAL_LOCK(state);
    USB_DeactivateEndpoint(state->Instance, ep);
    __HAL_UNLOCK(state);
    return 0;
}


static usb_status_t
_stm32_usb_dev_recv(usb_dev_ctrl_handle handle,
                    uint8_t ep_addr, uint8_t *buf, uint32_t len)
{
    stm32_usb_dev_state_t *state = (stm32_usb_dev_state_t *)handle;
    USB_OTG_EPTypeDef *ep;

    //printf("_stm32_usb_dev_recv\n");

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    if (!buf) {
        return kStatus_USB_InvalidParameter;
    }

    //printf("usb dev recv2\n");

    ep = &state->OUT_ep[ep_addr & 0x7F];

    ep->xfer_buff = buf;
    ep->xfer_len = len;
    ep->xfer_count = 0;
    ep->is_in = 0;
    ep->num = ep_addr & 0x7f;

    if (state->Init.dma_enable == 1) {
        ep->dma_addr = (uint32_t)buf;
    }

    if ((ep_addr & 0x7F) == 0) {
        USB_EP0StartXfer(state->Instance, ep, state->Init.dma_enable);
    } else {
        USB_EPStartXfer(state->Instance, ep, state->Init.dma_enable);
    }

    return 0;
}

static usb_status_t
_stm32_usb_dev_send(usb_dev_ctrl_handle handle,
                    uint8_t ep_addr,
                    uint8_t *buf,
                    uint32_t len)
{
    stm32_usb_dev_state_t *state = (stm32_usb_dev_state_t *)handle;
    USB_OTG_EPTypeDef *ep;

    //printf("_stm32_usb_dev_send, ep=%d, len=%ld\n", ep_addr, len);

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    if (!buf) {
        return kStatus_USB_InvalidParameter;
    }

    ep = &state->IN_ep[ep_addr & 0x7f];

    ep->xfer_buff = buf;
    ep->xfer_len = len;
    ep->xfer_count = 0;
    ep->is_in = 1;
    ep->num = ep_addr & 0x7F;

    if (state->Init.dma_enable == 1) {
        ep->dma_addr = (uint32_t)buf;
    }

    if (!(ep_addr & 0x7F)) {
        //printf("USB_EP0StartXfer\n");
        USB_EP0StartXfer(state->Instance, ep, state->Init.dma_enable);
    } else {
        USB_EPStartXfer(state->Instance, ep, state->Init.dma_enable);
    }

    return kStatus_USB_Success;
}

static int
_stm32_usb_dev_ep_stall(stm32_usb_dev_state_t *state, uint8_t ep_addr)
{
    USB_OTG_EPTypeDef *ep;

    /*
     * FIXME: need to cancel first?
     *
     * usb_stm32_cancel(state, ep);
     */

    if (0x80 & ep_addr) {
        ep = &state->IN_ep[ep_addr & 0x7F];
    } else {
        ep = &state->OUT_ep[ep_addr];
    }

    ep->is_stall = 1;
    ep->num = ep_addr & 0x7F;
    ep->is_in = (ep_addr & 0x80) == 0x80;

    __HAL_LOCK(state);
    USB_EPSetStall(state->Instance, ep);
    if (!(ep_addr & 0x7F)) {
        USB_EP0_OutStart(state->Instance, state->Init.dma_enable, (uint8_t *)state->Setup);
    }
    __HAL_UNLOCK(state);

  return 0;
}

static int
_stm32_usb_dev_ep_unstall(stm32_usb_dev_state_t *state, uint8_t ep_addr)
{
    USB_OTG_EPTypeDef *ep;

    if (ep_addr & 0x80) {
        ep = &state->IN_ep[ep_addr & 0x7F];
    } else {
        ep = &state->OUT_ep[ep_addr];
    }

    ep->is_stall = 0;
    ep->num = ep_addr & 0x7F;
    ep->is_in = (ep_addr & 0x80) == 0x80;

    __HAL_LOCK(state);
    USB_EPClearStall(state->Instance, ep);
    __HAL_UNLOCK(state);

    return 0;
}

static usb_status_t
_stm32_usb_dev_deinit(usb_dev_ctrl_handle handle)
{
    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    //printf("usb dev deinit\n");

   //FIXME: state->State = STATE_BUSY;

    _stm32_usb_dev_stop(handle);
    _stm32_usb_dev_msp_deinit();

    //FIXME: hpcd->State = STATE_RESET;

    return 0;
}

/*!
 * The function is used to cancel the pending transfer in a specified endpoint.
 */
static usb_status_t
_stm32_usb_dev_cancel(usb_dev_ctrl_handle handle, uint8_t ep)
{
    stm32_usb_dev_state_t *state = NULL;
    //usb_dev_cb_msg_t msg;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    //printf("usb dev cancel\n");

    state = (stm32_usb_dev_state_t *)handle;

    __HAL_LOCK(state);

    if (ep & 0x80) {
        USB_FlushTxFifo(state->Instance, ep & 0x7F);
    } else {
        USB_FlushRxFifo(state->Instance);
    }

    __HAL_UNLOCK(state);

#if 0
    /* Cancel the transfer and notify the up layer when the endpoint is busy. */
    if (khciState->ep_state[index].stateUnion.stateBitField.transferring)
    {
        msg.buf = state->IN_ep[i].xfer_buf;
        msg.len = USB_UNINITIALIZED_VAL_32;
        msg.code = ep;
        msg.isSetup = 0;
        //TODO: mark not transfering
        usb_dev_notify(state->dev, &msg);
    }
#endif
    return kStatus_USB_Success;
}

static usb_status_t
_stm32_usb_dev_control(usb_dev_ctrl_handle handle,
                       usb_device_control_type_t type, void *param)
{
    stm32_usb_dev_state_t *state = NULL;
    usb_status_t error = kStatus_USB_Success;

    //printf("usb dev control %d\n", (uint8_t) type);

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    state = (stm32_usb_dev_state_t *)handle;

    //printf("usb dev control2 %d\n", (uint8_t) type);
    //printf("DSTS=0x%04lx\n", ((volatile USB_OTG_DeviceTypeDef *)((uint32_t)state->Instance + USB_OTG_DEVICE_BASE))->DSTS);
    //printf("GAHBCFG=0x%04lx\n", ((volatile USB_OTG_GlobalTypeDef *)state->Instance)->GAHBCFG);

    switch (type) {
    case USB_DEV_CTRL_RUN:
        _stm32_usb_dev_start(handle);
        error = kStatus_USB_Success;
        break;
    case USB_DEV_CTRL_STOP:
        _stm32_usb_dev_stop(handle);
        error = kStatus_USB_Success;
        break;
    case USB_DEV_CTRL_EP_INIT:
        if (param) {
            error = _stm32_usb_dev_ep_init(state, (usb_dev_ep_init_t *) param);
        }
        break;
    case USB_DEV_CTRL_EP_DEINIT:
        if (param) {
            error = _stm32_usb_dev_ep_deinit(state, *((uint8_t *)param));
        }
        break;
    case USB_DEV_CTRL_EP_STALL:
        if (param) {
            error = _stm32_usb_dev_ep_stall(state, *((uint8_t *)param));
        }
        break;
    case USB_DEV_CTRL_EP_UNSTALL:
        if (param) {
            error = _stm32_usb_dev_ep_unstall(state, *((uint8_t *)param));
        }
        break;
    case USB_DEV_CTRL_GET_STATUS:
        if (param) {
            *((uint16_t *)param) =
                (MYNEWT_VAL(USB_DEVICE_CONFIG_SELF_POWER) <<
                 (USB_REQ_STD_GET_STATUS_DEVICE_SELF_POWERED_SHIFT))
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
                | ((uint16_t)(((uint32_t)deviceHandle->remotewakeup)
                                    << (
                                  USB_REQ_STD_GET_STATUS_DEVICE_REMOTE_WARKUP_SHIFT)))
#endif
            ;
            error = kStatus_USB_Success;
        }
        break;
    case USB_DEV_CTRL_GET_EP_STATUS:
        if (param) {
            usb_dev_ep_status_t *ep_status = (usb_dev_ep_status_t *)param;

            if (USB_EP_NUMBER(ep_status->addr) < MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
#if 0 //TODO
                ep_status->status = (uint16_t)(
                    state->ep_state[
                        (USB_EP_NUMBER(ep_status->addr) << 1)
                        | USB_EP_DIR(ep_status->addr)]
                        .stateUnion.stateBitField.stalled == 1) ?
                    kUSB_DeviceEndpointStateStalled :
                    kUSB_DeviceEndpointStateIdle;
#endif
                ep_status->status = kUSB_DeviceEndpointStateIdle;
                error = kStatus_USB_Success;
            }
        }
        break;
    case USB_DEV_CTRL_SET_ADDR:
        if (param) {
            error = _stm32_usb_dev_set_addr(state, *((uint8_t *)param));
        }
        break;
    case USB_DEV_CTRL_GET_SYNCF:
        error = kStatus_USB_Success;
        break;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    case USB_DEV_CTRL_RESUME:
        startTick = state->dev->hwTick;
        while ((state->dev->hwTick - startTick) < 10) {
            __ASM("nop");
        }
        error = kStatus_USB_Success;
        break;
    case USB_DEV_CTRL_SUSPEND:
        error = kStatus_USB_Success;
        break;
#endif
    case USB_DEV_CTRL_SET_DFLT_STATUS:
        for (uint8_t count = 0; count < MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS); count++) {
            _stm32_usb_dev_ep_deinit(state, count | (USB_IN << 7));
            _stm32_usb_dev_ep_deinit(state, count | (USB_OUT << 7));
        }
        //TODO: USB_DeviceKhciSetDefaultState(state);
        error = kStatus_USB_Success;
        break;
    case USB_DEV_CTRL_GET_SPEED:
        if (param) {
            // *((uint8_t *)param) = USB_SPEED_FULL;
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
    default:
        break;
    }

    //printf("DSTS=0x%04lx\n", ((volatile USB_OTG_DeviceTypeDef *)((uint32_t)state->Instance + USB_OTG_DEVICE_BASE))->DSTS);
    //printf("GAHBCFG=0x%04lx\n", ((volatile USB_OTG_GlobalTypeDef *)state->Instance)->GAHBCFG);

    return error;
}

static HAL_StatusTypeDef
USB_CoreReset(USB_OTG_GlobalTypeDef *USBx)
{
    int i = 0;

    os_time_delay(1);

    USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

    /* Core Soft Reset */
    i = 0;
    do {
        if (++i > 1000000) {
            return HAL_TIMEOUT;
        }
    } while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);

    os_time_delay(1);

    /* Wait for AHB master IDLE state. */
    i = 0;
    do {
        if (++i > 1000000) {
            return HAL_TIMEOUT;
        }
    } while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0);

  return HAL_OK;
}

void
HAL_Delay(uint32_t ms)
{
    /* TODO: change to use os_time_ms_to_ticks */
    os_time_delay(ms);
}

static void SystemClock_Config(void)
{
    //RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 432;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 9;
    RCC_OscInitStruct.PLL.PLLR = 7;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        printf("clock config error\n");
    }

#if 0
    /* Activate the OverDrive to reach the 216 Mhz Frequency */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        printf("clock config error\n");
    }
#endif

#if 0
    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
        printf("clock config error\n");
    }
#endif
}

static usb_status_t
_stm32_usb_dev_init(uint8_t controllerId,
                    usb_device_handle handle,
                    usb_dev_ctrl_handle *ctrl_handle)
{
    int i = 0;
    stm32_usb_dev_state_t *state = NULL;
    USB_OTG_GlobalTypeDef *instance = NULL;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    //printf("usb init\n");

    SystemClock_Config();

    state = &g_stm32_usb_dev_state;
    state->controllerId = controllerId;

    state->dev = (usb_dev_t *)handle;

    state->Init.dev_endpoints = 4;
    state->Init.use_dedicated_ep1 = 0;
    state->Init.ep0_mps = USB_OTG_FS_MAX_PACKET_SIZE;
    state->Init.dma_enable = 0;
    state->Init.low_power_enable = 0;
    state->Init.phy_itface = USB_OTG_EMBEDDED_PHY;
    state->Init.Sof_enable = 0;
    state->Init.speed = USB_OTG_SPEED_FULL;
    state->Init.vbus_sensing_enable = 1; //FIXME: 1;
    state->Init.lpm_enable = 0;

    instance = state->Instance = USB_OTG_FS;

    //FIXME: state->State = STATE_BUSY;

    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    _stm32_usb_dev_msp_init();

    USB_DisableGlobalInt(instance);

    /* Select FS Embedded PHY */
    instance->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

    /* Reset after a PHY select and set Host mode */
    //FIXME: this was copy/pasted, error checking BAD
    USB_CoreReset(instance);

    /* Deactivate the power down*/
    instance->GCCFG = USB_OTG_GCCFG_PWRDWN;

    //USB_CoreInit(instance, state->Init);
    USB_SetCurrentMode(instance, USB_OTG_DEVICE_MODE);

    for (i = 0; i < 15 ; i++) {
        state->IN_ep[i].is_in = 1;
        state->IN_ep[i].num = i;
        state->IN_ep[i].tx_fifo_num = i;

        state->IN_ep[i].type = EP_TYPE_CTRL;
        state->IN_ep[i].maxpacket =  0;
        state->IN_ep[i].xfer_buff = 0;
        state->IN_ep[i].xfer_len = 0;
    }

    for (i = 0; i < 15 ; i++) {
        state->OUT_ep[i].is_in = 0;
        state->OUT_ep[i].num = i;
        //FIXME: this was IN_ep !!!
        state->IN_ep[i].tx_fifo_num = i;

        state->OUT_ep[i].type = EP_TYPE_CTRL;
        state->OUT_ep[i].maxpacket = 0;
        state->OUT_ep[i].xfer_buff = 0;
        state->OUT_ep[i].xfer_len = 0;

        instance->DIEPTXF[i] = 0;
    }

    USB_DevInit(instance, state->Init);

    //FIXME: state->State = STATE_READY;

    //_stm32_usb_dev_control(state, USB_DEV_CTRL_STOP, NULL);

    //FIXME: state->State= STATE_READY;
    USB_DevDisconnect(instance);

#if 0
    instance->GRXFSIZ = 0x80;
    instance->DIEPTXF0_HNPTXFSIZ = (uint32_t)(((uint32_t)0x40 << 16) | instance->GRXFSIZ);
    instance->DIEPTXF[0] = (uint32_t)(((uint32_t)0x80 << 16) |
            (instance->GRXFSIZ + (instance->DIEPTXF0_HNPTXFSIZ >> 16)));
#endif

    *ctrl_handle = state;

    //printf("instance=0x%lx\n", (uint32_t )instance);
    //printf("DSTS=0x%04lx\n", ((USB_OTG_DeviceTypeDef *)((uint32_t )instance + USB_OTG_DEVICE_BASE))->DSTS);

    return 0;
}

static const usb_dev_ctrl_itf_t stm32_usb_interface = {
    _stm32_usb_dev_init,
    _stm32_usb_dev_deinit,
    _stm32_usb_dev_send,
    _stm32_usb_dev_recv,
    _stm32_usb_dev_cancel,
    _stm32_usb_dev_control,
};

usb_status_t
USB_DeviceErrorNotification(usb_dev_t *handle, usb_dev_cb_msg_t *msg)
{
    return handle->devcb(handle, kUSB_DeviceEventError, NULL);
}

static void
_write_empty_tx_fifo(stm32_usb_dev_state_t *state, uint32_t epnum)
{
    USB_OTG_GlobalTypeDef *USBx = state->Instance;
    USB_OTG_EPTypeDef *ep;
    int32_t len = 0;
    uint32_t len32b;
    uint32_t fifoemptymsk = 0;

    printf("_write_empty_tx_fifo\n");

    ep = &state->IN_ep[epnum];
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

        USB_WritePacket(USBx, ep->xfer_buff, epnum, len, state->Init.dma_enable);

        ep->xfer_buff += len;
        ep->xfer_count += len;
    }

    if (len <= 0) {
        fifoemptymsk = 0x1 << epnum;
        USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;
    }
}

void
stm32_usb_dev_isr(void)
{
    uint32_t i = 0, ep_intr = 0, epint = 0, epnum = 0;
    uint32_t fifoemptymsk = 0, temp = 0;
    USB_OTG_EPTypeDef *ep = NULL;
    uint32_t hclk = 200000000;
    stm32_usb_dev_state_t *usb = NULL;
    USB_OTG_GlobalTypeDef *USBx = NULL;
    usb_dev_cb_msg_t msg;

    usb = &g_stm32_usb_dev_state;

#if 0
    if (usb->state != USB_STATE_READY) {
        return;
    }
#endif

    //printf("isr\n");

    /* Required for _ll_usb */
    USBx = usb->Instance;

    if (USB_GetMode(usb->Instance) == USB_OTG_MODE_DEVICE) {
        if (!USB_ReadInterrupts(usb->Instance)) {
            return;
        }

        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_MMIS) {
            /* incorrect mode, acknowledge the interrupt */
            usb->Instance->GINTSTS = USB_OTG_GINTSTS_MMIS;
        }

        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_OEPINT) {
            epnum = 0;

            /* Read in the device interrupt bits */
            ep_intr = USB_ReadDevAllOutEpInterrupt(usb->Instance);

            while (ep_intr) {
                if (ep_intr & 1) {
                    epint = USB_ReadDevOutEPInterrupt(usb->Instance, epnum);

                    if (epint & USB_OTG_DOEPINT_XFRC) {
                        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_XFRC);

                        if (usb->Init.dma_enable == 1) {
                            usb->OUT_ep[epnum].xfer_count = usb->OUT_ep[epnum].maxpacket - (USBx_OUTEP(epnum)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ);
                            usb->OUT_ep[epnum].xfer_buff += usb->OUT_ep[epnum].maxpacket;
                        }

                        /* TODO: data out stage callback */
                        printf("isr - data out\n");
                        msg.len = usb->OUT_ep[epnum].xfer_len;
                        msg.buf = usb->OUT_ep[epnum].xfer_buff;
                        msg.isSetup = 0;
                        msg.code = epnum;
                        usb_dev_notify(usb->dev, &msg);

                        if (usb->Init.dma_enable == 1) {
                            if (!epnum && !usb->OUT_ep[epnum].xfer_len) {
                                /* this is ZLP, so prepare EP0 for next setup */
                                USB_EP0_OutStart(usb->Instance, 1, (uint8_t *)usb->Setup);
                            }
                        }
                    }

                    if (epint & USB_OTG_DOEPINT_STUP) {
                        /* Inform the upper layer that a setup packet is available */

                        /* TODO: setup stage callback */
                        //printf("isr - setup\n");
                        msg.len = 8;
                        msg.buf = (uint8_t *)usb->Setup;
                        msg.isSetup = 1;
                        //usb->dev->isResetting = 1;
                        msg.code = epnum;
                        //printf("isr - len=%ld\n", msg.len);
                        //TODO: point buf to Setup ?
                        usb_dev_notify(usb->dev, &msg);

                        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STUP);
                    }

                    if (epint & USB_OTG_DOEPINT_OTEPDIS) {
                        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPDIS);
                    }

                    /* Clear Status Phase Received interrupt */
                    if (epint & USB_OTG_DOEPINT_OTEPSPR) {
                        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
                    }
                }

                epnum++;
                ep_intr >>= 1;
            }
        }

        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_IEPINT) {
            /* Read in the device interrupt bits */
            ep_intr = USB_ReadDevAllInEpInterrupt(usb->Instance);

            epnum = 0;

            while (ep_intr) {
                if (ep_intr & 0x1) {
                    epint = USB_ReadDevInEPInterrupt(usb->Instance, epnum);

                    if (epint & USB_OTG_DIEPINT_XFRC) {
                        fifoemptymsk = 1 << epnum;
                        USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;

                        CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_XFRC);

                        if (usb->Init.dma_enable == 1) {
                            usb->IN_ep[epnum].xfer_buff += usb->IN_ep[epnum].maxpacket;
                        }

                        /* TODO: data in stage callback */
                        printf("isr - data in\n");

                        if (usb->Init.dma_enable == 1) {
                            /* this is ZLP, so prepare EP0 for next setup */
                            if (!epnum && !usb->IN_ep[epnum].xfer_len) {
                                /* prepare to rx more setup packets */
                                USB_EP0_OutStart(usb->Instance, 1, (uint8_t *)usb->Setup);
                            }
                        }
                    }

                    if (epint & USB_OTG_DIEPINT_TOC) {
                        CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_TOC);
                    }
                    if (epint & USB_OTG_DIEPINT_ITTXFE) {
                        CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_ITTXFE);
                    }
                    if (epint & USB_OTG_DIEPINT_INEPNE) {
                        CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_INEPNE);
                    }
                    if (epint & USB_OTG_DIEPINT_EPDISD) {
                        CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_EPDISD);
                    }
                    if (epint & USB_OTG_DIEPINT_TXFE) {
                        _write_empty_tx_fifo(usb , epnum);
                    }
                }

                epnum++;
                ep_intr >>= 1;
            }
        }

        /* Handle Resume Interrupt */
        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_WKUINT) {
            /* Clear the Remote Wake-up Signaling */
            USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;

            if (usb->LPM_State == LPM_L1) {
                usb->LPM_State = LPM_L0;
                //FIXME: HAL_PCDEx_LPM_Callback(usb, PCD_LPM_L0_ACTIVE);
            } else {
                printf("isr - resume\n");
                _init_cb_msg_with_code(&msg, kUSB_DeviceNotifyResume);
                usb_dev_notify(usb->dev, &msg);
            }
            usb->Instance->GINTSTS = USB_OTG_GINTSTS_WKUINT;
        }

        /* Handle Suspend Interrupt */
        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_USBSUSP) {
            if (USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) {
                //printf("isr - suspend\n");
                _init_cb_msg_with_code(&msg, kUSB_DeviceNotifySuspend);
                usb_dev_notify(usb->dev, &msg);
            }
            usb->Instance->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
        }

        /* Handle LPM Interrupt */
        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_LPMINT) {
            usb->Instance->GINTSTS = USB_OTG_GINTSTS_LPMINT;
#if 0
            if (usb->LPM_State == LPM_L0) {
                usb->LPM_State = LPM_L1;
                usb->BESL = (usb->Instance->GLPMCFG & USB_OTG_GLPMCFG_BESL) >> 2;
                //FIXME: HAL_PCDEx_LPM_Callback(usb, PCD_LPM_L1_ACTIVE);
            } else {
                /* TODO: trigger suspend callback */
            }
#endif
        }

        /* Handle Reset Interrupt */
        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_USBRST) {
            USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
            USB_FlushTxFifo(usb->Instance, 0x10);

            for (i = 0; i < usb->Init.dev_endpoints; i++) {
                USBx_INEP(i)->DIEPINT = 0xFF;
                USBx_OUTEP(i)->DOEPINT = 0xFF;
            }
            USBx_DEVICE->DAINT = 0xFFFFFFFF;
            USBx_DEVICE->DAINTMSK |= 0x10001;

            if (usb->Init.use_dedicated_ep1) {
                USBx_DEVICE->DOUTEP1MSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM);
                USBx_DEVICE->DINEP1MSK |= (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);
            } else {
                USBx_DEVICE->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM | USB_OTG_DOEPMSK_OTEPSPRM);
                USBx_DEVICE->DIEPMSK |= (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);
            }

            /* Set Default Address to 0 */
            USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

            /* setup EP0 to receive SETUP packets */
            USB_EP0_OutStart(usb->Instance, usb->Init.dma_enable, (uint8_t *)usb->Setup);
            //printf("isr - reset\n");

            _init_cb_msg_with_code(&msg, kUSB_DeviceNotifyBusReset);
            usb_dev_notify(usb->dev, &msg);

            usb->Instance->GINTSTS = USB_OTG_GINTSTS_USBRST;
        }

        /* Handle Enumeration done Interrupt */
        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_ENUMDNE) {
            USB_ActivateSetup(usb->Instance);
            usb->Instance->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;

            if (USB_GetDevSpeed(usb->Instance) == USB_OTG_SPEED_HIGH) {
                usb->Init.speed = USB_OTG_SPEED_HIGH;
                usb->Init.ep0_mps = USB_OTG_HS_MAX_PACKET_SIZE;
                /* NOTE: USBD_FS_TRDT==9 */
                usb->Instance->GUSBCFG |= (uint32_t)((9 << 10) & USB_OTG_GUSBCFG_TRDT);
            } else {
                usb->Init.speed = USB_OTG_SPEED_FULL;
                usb->Init.ep0_mps = USB_OTG_FS_MAX_PACKET_SIZE;

                /* The USBTRD is configured according to the tables below, depending on AHB frequency 
                used by application. In the low AHB frequency range it is used to stretch enough the USB response 
                time to IN tokens, the USB turnaround time, so to compensate for the longer AHB read access 
                latency to the Data FIFO */

                /* Get hclk frequency value */
                hclk = HAL_RCC_GetHCLKFreq();

                if (hclk < 14200000) {
                    assert (0);
                } else if (hclk < 15000000) {
                    usb->Instance->GUSBCFG |= (uint32_t)((0xF << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 16000000) {
                    usb->Instance->GUSBCFG |= (uint32_t)((0xE << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 17200000) {
                    usb->Instance->GUSBCFG |= (uint32_t)((0xD << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 18500000) {
                    usb->Instance->GUSBCFG |= (uint32_t)((0xC << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 20000000) {
                    usb->Instance->GUSBCFG |= (uint32_t)((0xB << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 21800000) {
                    usb->Instance->GUSBCFG |= (uint32_t)((0xA << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 24000000) {
                    usb->Instance->GUSBCFG |= (uint32_t)((0x9 << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 27700000) {
                    usb->Instance->GUSBCFG |= (uint32_t)((0x8 << 10) & USB_OTG_GUSBCFG_TRDT);
                } else if (hclk < 32000000) {
                    usb->Instance->GUSBCFG |= (uint32_t)((0x7 << 10) & USB_OTG_GUSBCFG_TRDT);
                } else {
                    usb->Instance->GUSBCFG |= (uint32_t)((0x6 << 10) & USB_OTG_GUSBCFG_TRDT);
                }
            }

            //printf("isr - enumeration done\n");
            //_init_cb_msg_with_code(&msg, kUSB_DeviceNotifyBusReset);
            //usb_dev_notify(usb->dev, &msg);

            usb->Instance->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
        }

        /* Handle RxQLevel Interrupt */
        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_RXFLVL) {
            USB_MASK_INTERRUPT(usb->Instance, USB_OTG_GINTSTS_RXFLVL);
            temp = USBx->GRXSTSP;
            ep = &usb->OUT_ep[temp & USB_OTG_GRXSTSP_EPNUM];

            if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_DATA_UPDT) {
                if (temp & USB_OTG_GRXSTSP_BCNT) {
                    USB_ReadPacket(USBx, ep->xfer_buff, (temp & USB_OTG_GRXSTSP_BCNT) >> 4);
                    ep->xfer_buff += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
                    ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
                }
            } else if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_SETUP_UPDT) {
                USB_ReadPacket(USBx, (uint8_t *)usb->Setup, 8);
                ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
            }
            USB_UNMASK_INTERRUPT(usb->Instance, USB_OTG_GINTSTS_RXFLVL);
        }

        /* Handle SOF Interrupt */
        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_SOF) {
            /* TODO: trigger SOF callback */
            usb->Instance->GINTSTS = USB_OTG_GINTSTS_SOF;
        }

        /* Handle Incomplete ISO IN Interrupt */
        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_IISOIXFR) {
            /* TODO: trigger ISO In incomplete callback */
            usb->Instance->GINTSTS = USB_OTG_GINTSTS_IISOIXFR;
        }

        /* Handle Incomplete ISO OUT Interrupt */
        if(USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT) {
            /* TODO: trigger ISO Out incomplete callback */
            usb->Instance->GINTSTS = USB_OTG_GINTSTS_PXFR_INCOMPISOOUT;
        }

        /* Handle Connection event Interrupt */
        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_SRQINT) {
            //printf("isr - attach\n");
            _init_cb_msg_with_code(&msg, kUSB_DeviceNotifyAttach);
            usb_dev_notify(usb->dev, &msg);
            usb->Instance->GINTSTS = USB_OTG_GINTSTS_SRQINT;
        }

        /* Handle Disconnection event Interrupt */
        if (USB_ReadInterrupts(usb->Instance) & USB_OTG_GINTSTS_OTGINT) {
            temp = usb->Instance->GOTGINT;

            if (temp & USB_OTG_GOTGINT_SEDET) {
                printf("isr - detach\n");
                _init_cb_msg_with_code(&msg, kUSB_DeviceNotifyDetach);
                usb_dev_notify(usb->dev, &msg);
            }
            usb->Instance->GOTGINT |= temp;
        }
    }
}

void
usb_hal_init_clocks(void)
{
    //TODO
}

const usb_dev_ctrl_itf_t *
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
    NVIC_SetPriority(OTG_FS_IRQn, 7);
    if (NVIC_GetVector(OTG_FS_IRQn) != (uint32_t)stm32_usb_dev_isr) {
        NVIC_SetVector(OTG_FS_IRQn, (uint32_t)stm32_usb_dev_isr);
    }
    NVIC_EnableIRQ(OTG_FS_IRQn);
}

void
usb_hal_disable_irq(void)
{
    NVIC_DisableIRQ(OTG_FS_IRQn);
}

void
usb_hal_clear_memory(void)
{
    /* Not needed for kinetis */
}
