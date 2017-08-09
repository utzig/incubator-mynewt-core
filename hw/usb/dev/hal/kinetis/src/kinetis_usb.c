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

#include <usb/usb.h>
#include <dev/dev.h>
#include <hal_usb/hal_usb.h>

#include <syscfg/syscfg.h>
#include <os/os.h>

#include <cmsis_nvic.h>

#include "fsl_device_registers.h"
#include "fsl_common.h"

#define USBx_ISTAT_USBRST         0x01
#define USBx_ISTAT_ERROR          0x02
#define USBx_ISTAT_SOFTOK         0x04
#define USBx_ISTAT_TOKDNE         0x08
#define USBx_ISTAT_SLEEP          0x10
#define USBx_ISTAT_RESUME         0x20
#define USBx_ISTAT_ATTACH         0x40
#define USBx_ISTAT_STALL          0x80

static __attribute__((aligned(512))) uint8_t bdt_buffer[512];

static kinetis_usb_dev_state_t g_kinetis_usb_dev_state;

#define DMA_ALIGN_LEN 64
static uint32_t g_kinetis_usb_dma_buffer[((DMA_ALIGN_LEN - 1) >> 2) + 1];

static void *g_handle = NULL;

static usb_status_t _kinetis_usb_send(usb_device_controller_handle handle,
                 uint8_t endpointAddress,
                 uint8_t *buffer,
                 uint32_t length);
static usb_status_t _kinetis_usb_recv(usb_device_controller_handle khciHandle,
                 uint8_t endpointAddress,
                 uint8_t *buffer,
                 uint32_t length);
static usb_status_t _kinetis_usb_cancel(usb_device_controller_handle khciHandle,
                 uint8_t ep);
static usb_status_t _kinetis_usb_control(usb_device_controller_handle handle,
                 usb_device_control_type_t type, void *param);

static uint32_t
_BDT_MEM(uint32_t base, uint32_t ep, uint32_t dir, uint32_t odd)
{
    uint32_t mem = base & 0xfffffe00;
    mem |= (ep & 0x0f) << 5;
    mem |= (dir & 1) << 4;
    mem |= (odd & 1) << 3;
    return mem;
}
static void
_BDT_SET_ADDR(uint32_t base, uint32_t ep, uint32_t dir, uint32_t odd, uint32_t addr)
{
    uint32_t mem = _BDT_MEM(base, ep, dir, odd);
    *((volatile uint32_t *) mem + 1) = addr;
}

static uint32_t
_BDT_GET_ADDR(uint32_t base, uint32_t ep, uint32_t dir, uint32_t odd)
{
    uint32_t mem = _BDT_MEM(base, ep, dir, odd);
    return *((volatile uint32_t *) mem + 1);
}

static void
_BDT_SET_CONTROL(uint32_t base, uint32_t ep, uint32_t dir, uint32_t odd, uint32_t control)
{
    uint32_t mem = _BDT_MEM(base, ep, dir, odd);
    *((volatile uint32_t *) mem) = control;
}

static uint32_t
_BDT_GET_CONTROL(uint32_t base, uint32_t ep, uint32_t dir, uint32_t odd)
{
    uint32_t mem = _BDT_MEM(base, ep, dir, odd);
    return *((volatile uint32_t *) mem);
}

static void
_init_cb_message_with_code(usb_device_callback_message_struct_t *message,
                           usb_device_notification_t code)
{
    message->buffer = NULL;
    message->code = code;
    message->length = 0;
    message->isSetup = 0;
}

/*
 * Start a transfer by writing the BDT.
 */
static usb_status_t
_kinetis_usb_dev_ep_txfer(kinetis_usb_dev_state_t *state, uint8_t ep,
                          uint8_t dir, uint8_t *buffer, uint32_t len)
{
    uint32_t index = ((uint32_t)ep << 1) | (uint32_t)dir;
    uint32_t odd;
    uint32_t data0;
    uint32_t control;
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);

    state->endpointState[index].stateUnion.stateBitField.transferring = 1;

    odd = state->endpointState[index].stateUnion.stateBitField.bdtOdd;
    data0 = state->endpointState[index].stateUnion.stateBitField.data0;

    _BDT_SET_ADDR((uint32_t) state->bdt, ep, dir, odd, (uint32_t) buffer);

    control = USB_LONG_TO_LITTLE_ENDIAN(
            USB_KHCI_BDT_BC(len) | USB_KHCI_BDT_OWN |
            USB_KHCI_BDT_DTS | USB_KHCI_BDT_DATA01(data0));
    _BDT_SET_CONTROL((uint32_t)state->bdt, ep, dir, odd, control);

    OS_EXIT_CRITICAL(sr);

    state->registers->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
    return kStatus_USB_Success;
}

/*
 * This function is used to prime a buffer in control out pipe to wait for
 * receiving the host's setup packet.
 */
static void
USB_DeviceKhciPrimeNextSetup(kinetis_usb_dev_state_t *state)
{
    kinetis_usb_dev_ep_state_t *epstate;

    epstate = &state->endpointState[(USB_CONTROL_ENDPOINT << 1) | USB_OUT];

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM)
    /* In case of lowpower mode enabled, it requires to put the setup packet buffer(16 bytes) into the USB RAM so
     * that the setup packet would wake up the USB.
     */
    epstate->transferBuffer = (uint8_t *)(state->bdt + 0x200 - 0x10) +
        epstate->stateUnion.stateBitField.bdtOdd * USB_SETUP_PACKET_SIZE;
#else
    epstate->transferBuffer = (uint8_t *)&state->setupPacketBuffer[0] +
        epstate->stateUnion.stateBitField.bdtOdd * USB_SETUP_PACKET_SIZE;
#endif

    epstate->transferDone = 0;
    epstate->transferLength = USB_SETUP_PACKET_SIZE;
    epstate->stateUnion.stateBitField.dmaAlign = 1;
    epstate->stateUnion.stateBitField.data0 = 0;

    _kinetis_usb_dev_ep_txfer(state, USB_CONTROL_ENDPOINT, USB_OUT,
                              epstate->transferBuffer, USB_SETUP_PACKET_SIZE);
}

static void
_kinetis_usb_dev_set_dflt(kinetis_usb_dev_state_t *state)
{
    uint8_t interruptFlag;
    int i;

    state->registers->ERRSTAT = 0xFF;

    /* Setting this bit to 1 resets all the BDT ODD ping/pong fields to 0,
     * which then specifies the EVEN BDT bank.
     */
    state->registers->CTL |= USB_CTL_ODDRST_MASK;

    state->registers->ADDR = 0;

    /* Clear the endpoint state and disable the endpoint */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS); i++) {
        _BDT_SET_CONTROL((uint32_t)state->bdt, i, USB_OUT, 0, 0);
        _BDT_SET_CONTROL((uint32_t)state->bdt, i, USB_OUT, 1, 0);
        _BDT_SET_CONTROL((uint32_t)state->bdt, i, USB_IN, 0, 0);
        _BDT_SET_CONTROL((uint32_t)state->bdt, i, USB_IN, 1, 0);

        state->endpointState[((uint32_t)i << 1) | USB_OUT].stateUnion.state = 0;
        state->endpointState[((uint32_t)i << 1) | USB_IN].stateUnion.state = 0;
        state->registers->ENDPOINT[i].ENDPT = 0x00;
    }

    state->isDmaAlignBufferInusing = 0;
    state->registers->CTL &= ~USB_CTL_ODDRST_MASK;
    state->registers->ERREN = 0xFF;

    interruptFlag = USBx_ISTAT_USBRST | USBx_ISTAT_TOKDNE | USBx_ISTAT_STALL
#if 0
                    | USBx_ISTAT_SOFTOK
#endif
                    ;

#if MYNEWT_VAL(USB_KINETIS_LOW_POWER_MODE)
    interruptFlag |= USBx_ISTAT_SLEEP;
#endif

#if defined(USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING)
    interruptFlag |= USBx_ISTAT_ERROR;
#endif

    state->registers->INTEN = interruptFlag;
    state->isResetting = 0;
    state->registers->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
}

static usb_status_t
_kinetis_usb_dev_ep_init(kinetis_usb_dev_state_t *state,
                         usb_device_endpoint_init_struct_t *epInit)
{
    uint16_t maxPacketSize = epInit->maxPacketSize;
    uint8_t ep = USB_EP_NUMBER(epInit->endpointAddress);
    uint8_t direction = USB_EP_DIR(epInit->endpointAddress);
    uint8_t index = ((uint8_t)((uint32_t)ep << 1)) | (uint8_t)direction;

    //printf("ep=%d, dir=%d\n", ep, direction);

    /* Make the endpoint max packet size align with USB Specification 2.0. */
    if (USB_ENDPOINT_ISOCHRONOUS == epInit->transferType) {
        if (maxPacketSize > USB_DEVICE_MAX_FS_ISO_MAX_PACKET_SIZE) {
            maxPacketSize = USB_DEVICE_MAX_FS_ISO_MAX_PACKET_SIZE;
        }
    } else {
        if (maxPacketSize > USB_DEVICE_MAX_FS_NONE_ISO_MAX_PACKET_SIZE) {
            maxPacketSize = USB_DEVICE_MAX_FS_NONE_ISO_MAX_PACKET_SIZE;
        }
        /* Enable an endpoint to perform handshaking during a transaction to this endpoint. */
        state->registers->ENDPOINT[ep].ENDPT |= USB_ENDPT_EPHSHK_MASK;
    }

    state->endpointState[index].stateUnion.stateBitField.transferring = 0;
    state->endpointState[index].stateUnion.stateBitField.maxPacketSize =
        maxPacketSize;
    state->endpointState[index].stateUnion.stateBitField.data0 = 0;
    state->endpointState[index].stateUnion.stateBitField.stalled = 0;
    state->endpointState[index].stateUnion.stateBitField.zlt = epInit->zlt;
    state->registers->ENDPOINT[ep].ENDPT |=
        (USB_IN == direction) ? USB_ENDPT_EPTXEN_MASK : USB_ENDPT_EPRXEN_MASK;

    /* Prime a transfer to receive next setup packet when the endpoint is control out endpoint. */
    if (ep == USB_CONTROL_ENDPOINT && direction == USB_OUT) {
        USB_DeviceKhciPrimeNextSetup(state);
    }

    return kStatus_USB_Success;
}

static usb_status_t
_kinetis_usb_dev_ep_deinit(kinetis_usb_dev_state_t *state, uint8_t endp)
{
    uint8_t ep = USB_EP_NUMBER(endp);
    uint8_t dir = USB_EP_DIR(endp);
    uint8_t index = ((uint8_t)((uint32_t)ep << 1)) | (uint8_t)dir;

    _kinetis_usb_cancel(state, endp);

    state->registers->ENDPOINT[ep].ENDPT = 0x00;
    state->endpointState[index].stateUnion.stateBitField.maxPacketSize = 0;

    return kStatus_USB_Success;
}

static usb_status_t
_kinetis_usb_dev_ep_stall(kinetis_usb_dev_state_t *state, uint8_t ep)
{
    uint8_t endpoint = USB_EP_NUMBER(ep);
    uint8_t direction = USB_EP_DIR(ep);
    uint8_t index = ((uint8_t)((uint32_t)endpoint << 1)) | (uint8_t)direction;

    /* Cancel the transfer of the endpoint */
    _kinetis_usb_cancel(state, ep);

    /* Set endpoint stall flag. */
    state->endpointState[index].stateUnion.stateBitField.stalled = 1;

    /* Set endpoint stall in BDT. And then if the host send a IN/OUT tanscation, the device will response a STALL state.
     */
    _BDT_SET_CONTROL(
        (uint32_t)state->bdt, endpoint, direction,
        state->endpointState[index].stateUnion.stateBitField.bdtOdd,
        USB_LONG_TO_LITTLE_ENDIAN(
            (uint32_t)(USB_KHCI_BDT_BC(state->endpointState[index].
                                       stateUnion.stateBitField.maxPacketSize)
                       |
                       USB_KHCI_BDT_DTS | USB_KHCI_BDT_STALL |
                       USB_KHCI_BDT_OWN)));

    state->registers->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;

    return kStatus_USB_Success;
}

/*
 * Un-stall a specified endpoint.
 */
static usb_status_t
_kinetis_usb_dev_ep_unstall(kinetis_usb_dev_state_t *state, uint8_t ep)
{
    uint32_t control;
    uint8_t endpoint = USB_EP_NUMBER(ep);
    uint8_t direction = USB_EP_DIR(ep);
    uint8_t index = ((uint8_t)((uint32_t)endpoint << 1)) | (uint8_t)direction;

    state->endpointState[index].stateUnion.stateBitField.stalled = 0;
    state->endpointState[index].stateUnion.stateBitField.data0 = 0;

    /* Clear stall state in BDT */
    for (uint8_t i = 0; i < 2; i++) {
        control = _BDT_GET_CONTROL((uint32_t)state->bdt, endpoint,
                                   direction, i);
        if (control & USB_KHCI_BDT_STALL) {
            _BDT_SET_CONTROL(
                (uint32_t)state->bdt, endpoint, direction, i,
                USB_LONG_TO_LITTLE_ENDIAN(
                    (uint32_t)(USB_KHCI_BDT_BC(state->endpointState[index].
                                               stateUnion.stateBitField.
                                               maxPacketSize) |
                               USB_KHCI_BDT_DTS | USB_KHCI_BDT_DATA01(0))));
        }
    }

    state->registers->ENDPOINT[endpoint].ENDPT &= ~USB_ENDPT_EPSTALL_MASK;

    if ((USB_CONTROL_ENDPOINT == endpoint) && (USB_OUT == direction)) {
        USB_DeviceKhciPrimeNextSetup(state);
    }

    state->registers->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;

    return kStatus_USB_Success;
}

/*
 * Handle the token done interrupt.
 */
static void
_kinetis_usb_dev_tokdone(kinetis_usb_dev_state_t *state)
{
    uint32_t control;
    uint32_t length;
    uint32_t remainingLength;
    uint8_t *bdtBuffer;
    usb_device_callback_message_struct_t message;
    uint8_t endpoint;
    uint8_t direction;
    uint8_t bdtOdd;
    uint8_t isSetup;
    uint8_t index;
    uint8_t stateRegister = state->registers->STAT;

    endpoint = (stateRegister & USB_STAT_ENDP_MASK) >> USB_STAT_ENDP_SHIFT;
    direction = (stateRegister & USB_STAT_TX_MASK) >> USB_STAT_TX_SHIFT;
    bdtOdd = (stateRegister & USB_STAT_ODD_MASK) >> USB_STAT_ODD_SHIFT;
    state->registers->ISTAT = USBx_ISTAT_TOKDNE;

    control = _BDT_GET_CONTROL((uint32_t)state->bdt, endpoint,
                               direction, bdtOdd);

    bdtBuffer = (uint8_t *)_BDT_GET_ADDR((uint32_t)state->bdt, endpoint,
                                         direction, bdtOdd);

    length = ((USB_LONG_FROM_LITTLE_ENDIAN(control)) >> 16) & 0x3ff;

    isSetup =
        (USB_KHCI_BDT_DEVICE_SETUP_TOKEN ==
         ((uint8_t)(((USB_LONG_FROM_LITTLE_ENDIAN(control)) >> 2) & 0x0f))) ?
        1 : 0;

    index = ((uint8_t)((uint32_t)endpoint << 1)) | (uint8_t)direction;

    if (!state->endpointState[index].stateUnion.stateBitField.transferring) {
        return;
    }

    if (isSetup) {
        state->setupBufferIndex = bdtOdd;
    }

    if (direction == USB_IN) {
        state->endpointState[index].transferDone += length;

        remainingLength = state->endpointState[index].transferLength -
                          state->endpointState[index].transferDone;

        state->endpointState[index].stateUnion.stateBitField.data0 ^= 1;
        state->endpointState[index].stateUnion.stateBitField.bdtOdd ^= 1;

        /* Whether the transfer is completed or not. */
        /*
         * The transfer is completed when one of the following conditions meet:
         * 1. The remaining length is zero.
         * 2. The length of current transcation is less than the max packet size of the current pipe.
         */
        if (!remainingLength ||
            (state->endpointState[index].stateUnion.stateBitField.
             maxPacketSize > length)) {
            message.length = state->endpointState[index].transferDone;
            message.buffer = state->endpointState[index].transferBuffer;
            state->endpointState[index].stateUnion.stateBitField.
            transferring = 0;

            /*
             * Whether need to send ZLT when the pipe is control in pipe and the transferred length of current
             * transaction equals to max packet size.
             */
            if (length &&
                (!(length %
                   state->endpointState[index].stateUnion.stateBitField.
                   maxPacketSize))) {
                if (USB_CONTROL_ENDPOINT == endpoint) {
                    usb_setup_struct_t *setup_packet =
                        (usb_setup_struct_t
                         *)(&state->setupPacketBuffer[(
                                                              USB_SETUP_PACKET_SIZE
                                                              * state->
                                                              setupBufferIndex)
                    ]);
                    /*
                     * Send the ZLT and terminate the token done interrupt service when the tranferred length in data
                     * phase
                     * is less than the host request.
                     */
                    if (USB_SHORT_FROM_LITTLE_ENDIAN(setup_packet->wLength) >
                        state->endpointState[index].transferLength) {
                        (void)_kinetis_usb_dev_ep_txfer(state, endpoint,
                                                        USB_IN, NULL, 0);
                        return;
                    }
                } else if (state->endpointState[index].stateUnion.
                           stateBitField.zlt) {
                    (void)_kinetis_usb_dev_ep_txfer(state, endpoint, USB_IN,
                                                    NULL, 0);
                    return;
                }
            }
        } else {
            _kinetis_usb_send(state, endpoint | (USB_IN << 0x07),
                              state->endpointState[index].transferBuffer,
                              remainingLength);
            return;
        }
    } else {
        if (endpoint == USB_CONTROL_ENDPOINT && !length) {
            message.length = 0;
            message.buffer = (uint8_t *)NULL;
        } else {
            if (!state->endpointState[index].stateUnion.stateBitField.dmaAlign) {
                uint8_t *buffer = (uint8_t *)USB_LONG_FROM_LITTLE_ENDIAN(
                    _BDT_GET_ADDR((uint32_t)state->bdt,
                                  endpoint, USB_OUT,
                                  state->endpointState[index].
                                  stateUnion.stateBitField.bdtOdd));
                uint8_t *transferBuffer =
                    state->endpointState[index].transferBuffer +
                    state->endpointState[index].transferDone;
                if (buffer != transferBuffer) {
                    for (uint32_t i = 0; i < length; i++) {
                        transferBuffer[i] = buffer[i];
                    }
                }
                state->isDmaAlignBufferInusing = 0;
            }
            state->endpointState[index].transferDone += length;
            remainingLength =
                state->endpointState[index].transferLength -
                state->endpointState[index].transferDone;

            if (endpoint == USB_CONTROL_ENDPOINT && isSetup) {
                state->endpointState[(USB_CONTROL_ENDPOINT << 1) |
                                         USB_OUT].stateUnion.stateBitField.
                data0 = 1;
                state->endpointState[(USB_CONTROL_ENDPOINT << 1) |
                                         USB_IN].stateUnion.stateBitField.data0
                    = 1;
            } else {
                state->endpointState[index].stateUnion.stateBitField.data0 ^= 1;
            }
            state->endpointState[index].stateUnion.stateBitField.bdtOdd ^= 1;
            if (!state->endpointState[index].transferLength ||
                !remainingLength ||
                (state->endpointState[index].stateUnion.stateBitField.
                 maxPacketSize > length)) {
                message.length = state->endpointState[index].transferDone;
                if (isSetup) {
                    message.buffer = bdtBuffer;
                } else {
                    message.buffer =
                        state->endpointState[index].transferBuffer;
                }
                state->endpointState[index].stateUnion.stateBitField.
                transferring = 0;
            } else {
                _kinetis_usb_recv(state, endpoint | (USB_OUT << 0x07),
                                  state->endpointState[index].transferBuffer,
                                  remainingLength);
                return;
            }
        }
    }

    message.isSetup = isSetup;
    message.code = endpoint | (uint8_t)(((uint32_t)direction << 0x07));

    usb_dev_notify(state->dev, &message);

    state->registers->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
}

static void
_kinetis_usb_dev_reset(kinetis_usb_dev_state_t *state)
{
    usb_device_callback_message_struct_t message;
    volatile USB_Type *regs = state->registers;

    state->isResetting = 1;
    regs->ISTAT = USBx_ISTAT_USBRST;

#if MYNEWT_VAL(USB_KINETIS_LOW_POWER_MODE)
    regs->ISTAT = USBx_ISTAT_SLEEP;
    regs->USBCTRL &= ~USB_USBCTRL_SUSP_MASK;
#endif

    _init_cb_message_with_code(&message, kUSB_DeviceNotifyBusReset);
    usb_dev_notify(state->dev, &message);
}

/* The USB suspend and resume signals need to be detected and handled when the
 * low power or remote wakeup function enabled.
 */
#if MYNEWT_VAL(USB_KINETIS_LOW_POWER_MODE)
static void
_kinetis_usb_dev_sleep(kinetis_usb_dev_state_t *state)
{
    usb_device_callback_message_struct_t message;
    volatile USB_Type *regs = state->registers;

    regs->INTEN |= USBx_ISTAT_RESUME;
    regs->USBTRC0 |= USB_USBTRC0_USBRESMEN_MASK;
    regs->USBCTRL |= USB_USBCTRL_SUSP_MASK;

    regs->INTEN &= ~((uint32_t)USBx_ISTAT_SLEEP);

    regs->ISTAT = USBx_ISTAT_SLEEP;
    regs->ISTAT = USBx_ISTAT_RESUME;

    _init_cb_message_with_code(&message, kUSB_DeviceNotifySuspend);
    usb_dev_notify(state->deviceHandle, &message);
}

static void
_kinetis_usb_dev_resume(kinetis_usb_dev_state_t *state)
{
    usb_device_callback_message_struct_t message;
    volatile USB_Type *regs = state->registers;

    regs->USBCTRL &= ~USB_USBCTRL_SUSP_MASK;
    regs->INTEN |= USBx_ISTAT_SLEEP;
    regs->INTEN &= ~((uint32_t)USBx_ISTAT_RESUME);
    regs->USBTRC0 &= ~USB_USBTRC0_USBRESMEN_MASK;

    regs->ISTAT = USBx_ISTAT_RESUME;
    regs->ISTAT = USBx_ISTAT_SLEEP;

    _init_cb_message_with_code(&message, kUSB_DeviceNotifyResume);
    usb_dev_notify(state->deviceHandle, &message);
}
#endif /* USB_KINETIS_LOW_POWER_MODE */

#if defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && \
    defined(FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED)
static void
_kinetis_usb_dev_vbus_rising(kinetis_usb_dev_state_t *state)
{
    usb_device_callback_message_struct_t message;
    volatile USB_Type *regs = state->regs;

    regs->MISCCTRL &= ~USB_MISCCTRL_VREDG_EN_MASK;
    regs->MISCCTRL |= USB_MISCCTRL_VREDG_EN_MASK;

    _init_cb_message_with_code(&message, kUSB_DeviceNotifyAttach);
    usb_dev_notify(state->deviceHandle, &message);
}

static void
_kinetis_usb_dev_vbus_falling(kinetis_usb_dev_state_t *state)
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
kinetis_usb_dev_sof(kinetis_usb_dev_state_t *state)
{
    state->registers->ISTAT = USBx_ISTAT_SOFTOK;
    state->registers->ISTAT = USBx_ISTAT_RESUME;
}
#endif

static void
_kinetis_usb_dev_stall(kinetis_usb_dev_state_t *state)
{
    while (state->registers->ISTAT & USBx_ISTAT_STALL) {
        state->registers->ISTAT = USBx_ISTAT_STALL;
    }

    /* Un-stall the control in and out pipe when the control in or out pipe stalled. */
    if ((state->endpointState[(USB_CONTROL_ENDPOINT << 1) |
                                  USB_IN].stateUnion.stateBitField.stalled) ||
        (state->endpointState[(USB_CONTROL_ENDPOINT << 1) | USB_OUT]
         .stateUnion.stateBitField.stalled)) {
        _kinetis_usb_dev_ep_unstall(state, USB_CONTROL_ENDPOINT |
             (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
        _kinetis_usb_dev_ep_unstall(state, USB_CONTROL_ENDPOINT |
             (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
    }
}

#if defined(USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING)
static void
_kinetis_usb_dev_error(kinetis_usb_dev_state_t *state)
{
    usb_device_callback_message_struct_t message;

    state->registers->ISTAT = USBx_ISTAT_ERROR;

    _init_cb_message_with_code(&message, kUSB_DeviceNotifyError);
    usb_dev_notify(state->deviceHandle, &message);
}
#endif

static usb_status_t
_kinetis_usb_init(uint8_t controllerId,
                  usb_device_handle handle,
                  usb_device_controller_handle *khandle)
{
    kinetis_usb_dev_state_t *state;
    volatile USB_Type *regs;

    state = &g_kinetis_usb_dev_state;
    state->controllerId = controllerId;

    state->registers = (volatile USB_Type *) USB0_BASE;
    regs = state->registers;

    state->dmaAlignBuffer = (uint8_t *) &g_kinetis_usb_dma_buffer[0];

    regs->ISTAT = 0xff;

#if defined(USB_DEVICE_CONFIG_OTG)
    state->otgStatus = 0;
#else
    _kinetis_usb_control(state, USB_DEV_CTRL_STOP, NULL);
#endif

    state->bdt = bdt_buffer;

    regs->BDTPAGE1 = (((uint32_t)state->bdt) >> 8) & 0xFF;
    regs->BDTPAGE2 = (((uint32_t)state->bdt) >> 16) & 0xFF;
    regs->BDTPAGE3 = (((uint32_t)state->bdt) >> 24) & 0xFF;

#if defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && \
    defined(FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED)
    regs->MISCCTRL |= USB_MISCCTRL_VREDG_EN_MASK | USB_MISCCTRL_VFEDG_EN_MASK;
#endif

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM)
    regs->CLK_RECOVER_CTRL |= USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_MASK;
    regs->KEEP_ALIVE_CTRL =
        USB_KEEP_ALIVE_CTRL_KEEP_ALIVE_EN_MASK |
        USB_KEEP_ALIVE_CTRL_OWN_OVERRD_EN_MASK |
        USB_KEEP_ALIVE_CTRL_WAKE_INT_EN_MASK |
        FSL_FEATURE_USB_KHCI_KEEP_ALIVE_MODE_CONTROL;
    regs->KEEP_ALIVE_WKCTRL = 0x1;
    PMC->REGSC |= PMC_REGSC_BGEN_MASK | PMC_REGSC_VLPO_MASK;
#endif

    _kinetis_usb_dev_set_dflt(state);

    *khandle = state;
    state->dev = (usb_dev_t *)handle;

    return 0;
}

static usb_status_t
_kinetis_usb_deinit(usb_device_controller_handle handle)
{
    volatile USB_Type *regs = NULL;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    regs = ((kinetis_usb_dev_state_t *) handle)->registers;

    regs->ISTAT = 0xFF;
    regs->INTEN &= ~0xFF;
    regs->ADDR = 0;
    regs->CTL = 0x00;
    regs->USBCTRL |= USB_USBCTRL_PDE_MASK | USB_USBCTRL_SUSP_MASK;

    return 0;
}

/*!
 * @brief Send data through a specified endpoint.
 *
 * This function sends data through a specified endpoint.
 *
 * @note The return value just means if the sending request is successful or not; the transfer done is notified by the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for one specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer could begin only when the previous transfer is done (get notification through the endpoint
 * callback).
 */
static usb_status_t
_kinetis_usb_send(usb_device_controller_handle handle,
                  uint8_t endpointAddress,
                  uint8_t *buffer,
                  uint32_t length)
{
    kinetis_usb_dev_state_t *state = (kinetis_usb_dev_state_t *)handle;
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
        error = _kinetis_usb_dev_ep_txfer(state, USB_EP_NUMBER(endpointAddress),
            USB_IN,
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
}

/*!
 * This function Receives data through a specified endpoint.
 *
 * @note The return value just means if the receiving request is successful or
 * not; the transfer done is notified by the corresponding callback function.
 * Currently, only one transfer request can be supported for one specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for
 * one specific endpoint, the application should implement a queue in the
 * application level. The subsequent transfer could begin only when the previous
 * transfer is done (get notification through the endpoint callback).
 */
static usb_status_t
_kinetis_usb_recv(usb_device_controller_handle handle,
                  uint8_t ep,
                  uint8_t *buffer,
                  uint32_t len)
{
    kinetis_usb_dev_state_t *state = (kinetis_usb_dev_state_t *)handle;
    uint32_t index = (USB_EP_NUMBER(ep) << 1) | USB_OUT;
    usb_status_t error = kStatus_USB_Error;

    if (!len && USB_EP_NUMBER(ep) == USB_CONTROL_ENDPOINT) {
        state->endpointState[index].stateUnion.stateBitField.transferring = 0;
        USB_DeviceKhciPrimeNextSetup(state);
    } else {
        /* Save the transfer information */
        if (!state->endpointState[index].stateUnion.stateBitField.transferring) {
            state->endpointState[index].transferDone = 0;
            state->endpointState[index].transferBuffer = buffer;
            state->endpointState[index].transferLength = len;
        }
        state->endpointState[index].stateUnion.stateBitField.dmaAlign = 1;

        /* Data length needs to less than max packet size in each call. */
        if (len >
            state->endpointState[index].stateUnion.stateBitField.maxPacketSize) {
            len = state->endpointState[index].stateUnion.stateBitField.maxPacketSize;
        }

        buffer = (uint8_t *)(
                (uint32_t)buffer + (uint32_t)state->endpointState[index].transferDone);

        if (state->dmaAlignBuffer && !state->isDmaAlignBufferInusing &&
            (DMA_ALIGN_LEN >= len) &&
            ((len & 0x03) || (((uint32_t)buffer) & 0x03))) {
            state->endpointState[index].stateUnion.stateBitField.dmaAlign = 0;
            buffer = state->dmaAlignBuffer;
            state->isDmaAlignBufferInusing = 1;
        }

        /* Receive data when the device is not resetting. */
        if (!state->isResetting) {
            error = _kinetis_usb_dev_ep_txfer(state, USB_EP_NUMBER(ep), USB_OUT,
                                              buffer, len);
        }
    }
    return error;
}

static usb_status_t
_kinetis_usb_cancel(usb_device_controller_handle handle, uint8_t ep)
{
    kinetis_usb_dev_state_t *state = (kinetis_usb_dev_state_t *)handle;
    usb_device_callback_message_struct_t message;
    //FIXME: should not be USB_EP_DIR???
    uint8_t index = (USB_EP_NUMBER(ep) << 1) | USB_EP_DIR(ep);

    /* Cancel the transfer and notify the up layer when the endpoint is busy. */
    if (state->endpointState[index].stateUnion.stateBitField.transferring) {
        message.length = USB_UNINITIALIZED_VAL_32;
        message.buffer = state->endpointState[index].transferBuffer;
        message.code = ep;
        message.isSetup = 0;
        state->endpointState[index].stateUnion.stateBitField.transferring = 0;
        usb_dev_notify(state->dev, &message);
    }
    return kStatus_USB_Success;
}

static usb_status_t
_kinetis_usb_control(usb_device_controller_handle handle,
                     usb_device_control_type_t type, void *param)
{
    kinetis_usb_dev_state_t *state = NULL;
    volatile USB_Type *regs = NULL;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    usb_dev_t *dev;
    uint64_t startTick;
#endif
    usb_status_t err = kStatus_USB_Error;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    state = (kinetis_usb_dev_state_t *) handle;
    regs = state->registers;

#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    dev = (usb_dev_t *) state->dev;
#endif

    switch (type) {
    case USB_DEV_CTRL_RUN:
        regs->USBCTRL = 0;
#if MYNEWT_VAL(FSL_FEATURE_USB_KHCI_OTG_ENABLED)
        if (regs->OTGCTL & USB_OTGCTL_OTGEN_MASK) {
            regs->OTGCTL |= USB_OTGCTL_DPHIGH_MASK;
        }
#endif
        regs->CONTROL |= USB_CONTROL_DPPULLUPNONOTG_MASK;
        regs->CTL |= USB_CTL_USBENSOFEN_MASK;

        err = kStatus_USB_Success;
        break;
    case USB_DEV_CTRL_STOP:
#if MYNEWT_VAL(FSL_FEATURE_USB_KHCI_OTG_ENABLED)
        if (regs->OTGCTL & USB_OTGCTL_OTGEN_MASK) {
            regs->OTGCTL &= ~USB_OTGCTL_DPHIGH_MASK;
        }
#endif
        regs->CONTROL &= ~USB_CONTROL_DPPULLUPNONOTG_MASK;
        err = kStatus_USB_Success;
        break;
    case USB_DEV_CTRL_EP_INIT:
        if (param) {
            err = _kinetis_usb_dev_ep_init(state,
                    (usb_device_endpoint_init_struct_t *) param);
        }
        break;
    case USB_DEV_CTRL_EP_DEINIT:
        if (param) {
            err = _kinetis_usb_dev_ep_deinit(state, *((uint8_t *) param));
        }
        break;
    case USB_DEV_CTRL_EP_STALL:
        if (param) {
            err = _kinetis_usb_dev_ep_stall(state, *((uint8_t *) param));
        }
        break;
    case USB_DEV_CTRL_EP_UNSTALL:
        if (param) {
            err = _kinetis_usb_dev_ep_unstall(state, *((uint8_t *) param));
        }
        break;
    case USB_DEV_CTRL_GET_STATUS:
        if (param) {
            *((uint16_t *) param) =
                (MYNEWT_VAL(USB_DEVICE_CONFIG_SELF_POWER) <<
                 (USB_REQUEST_STANDARD_GET_STATUS_DEVICE_SELF_POWERED_SHIFT))
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
                | ((uint16_t)(((uint32_t)devhandle->remotewakeup)
                                    << (
                                  USB_REQUEST_STANDARD_GET_STATUS_DEVICE_REMOTE_WARKUP_SHIFT)))
#endif
            ;
            err = kStatus_USB_Success;
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
                err = kStatus_USB_Success;
            }
        }
        break;
    case USB_DEV_CTRL_SET_ADDR:
        if (param) {
            regs->ADDR = *((uint8_t *)param);
            err = kStatus_USB_Success;
        }
        break;
    case USB_DEV_CTRL_GET_SYNCF:
        break;
#if MYNEWT_VAL(USB_KINETIS_LOW_POWER_MODE)
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    case USB_DEV_CTRL_RESUME:
        regs->CTL |= USB_CTL_RESUME_MASK;
        startTick = devhandle->hwTick;
        while ((devhandle->hwTick - startTick) < 10) {
            __ASM("nop");
        }
        regs->CTL &= ~USB_CTL_RESUME_MASK;
        err = kStatus_USB_Success;
        break;
#endif /* USB_DEVICE_CONFIG_REMOTE_WAKEUP */
    case USB_DEV_CTRL_SUSPEND:
        err = kStatus_USB_Success;
        break;
#endif /* USB_KINETIS_LOW_POWER_MODE */
    case USB_DEV_CTRL_SET_DFLT_STATUS:
        for (uint8_t count = 0; count < MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS); count++) {
            _kinetis_usb_dev_ep_deinit(state, count | (USB_IN << 7));
            _kinetis_usb_dev_ep_deinit(state, count | (USB_OUT << 7));
        }
        _kinetis_usb_dev_set_dflt(state);
        err = kStatus_USB_Success;
        break;
    case USB_DEV_CTRL_GET_SPEED:
        if (param) {
            *((uint8_t *)param) = USB_SPEED_FULL;
            err = kStatus_USB_Success;
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

    return err;
}

/*!
 * The function is used to handle the KHCI device interrupt.
 *
 * @param deviceHandle    The device handle got from USB_DeviceInit.
 */
static void
kinetis_usb_dev_isr(void)
{
    usb_dev_t *dev = (usb_dev_t *) g_handle;
    kinetis_usb_dev_state_t *state;
    volatile USB_Type *regs = NULL;
    uint8_t status;

    if (!dev) {
        return;
    }

    state = (kinetis_usb_dev_state_t *) dev->controllerHandle;
    regs = state->registers;

    status = regs->ISTAT;

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM)
    if (regs->KEEP_ALIVE_CTRL & USB_KEEP_ALIVE_CTRL_WAKE_INT_STS_MASK) {
        regs->KEEP_ALIVE_CTRL |= USB_KEEP_ALIVE_CTRL_WAKE_INT_STS_MASK;
    }
    if (regs->ISTAT & USB_ISTAT_SOFTOK_MASK) {
        regs->ISTAT = USB_ISTAT_SOFTOK_MASK;
    }
#endif

#if defined(USB_DEVICE_CONFIG_KHCI_ERROR_HANDLING)
    if (status & USBx_ISTAT_ERROR) {
        _kinetis_usb_dev_error(state);
    }
#endif

    if (status & USBx_ISTAT_TOKDNE) {
        _kinetis_usb_dev_tokdone(state);
    }

    if (status & USBx_ISTAT_USBRST) {
        _kinetis_usb_dev_reset(state);
    }

#if MYNEWT_VAL(USB_KINETIS_LOW_POWER_MODE)
    if (status & USBx_ISTAT_SLEEP) {
        _kinetis_usb_dev_sleep(state);
    }

    if (status & USBx_ISTAT_RESUME) {
        _kinetis_usb_dev_resume(state);
    }

    if (regs->USBTRC0 & USB_USBTRC0_USB_RESUME_INT_MASK) {
        _kinetis_usb_dev_resume(state);
    }
#endif

    if (status & USBx_ISTAT_STALL) {
        _kinetis_usb_dev_stall(state);
    }

#if defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && \
    defined(FSL_FEATURE_USB_KHCI_VBUS_DETECT_ENABLED)
    if (regs->USBTRC0 & USB_USBTRC0_VREDG_DET_MASK) {
        _kinetis_usb_dev_vbus_rising(state);
    }

    if (regs->USBTRC0 & USB_USBTRC0_VFEDG_DET_MASK) {
        _kinetis_usb_dev_vbus_falling(state);
    }
#endif

#if 0
    if (status & USBx_ISTAT_SOFTOK) {
        kinetis_usb_dev_sof(state);
    }
#endif

#if defined(FSL_FEATURE_USB_KHCI_IRC48M_MODULE_CLOCK_ENABLED)
    status = regs->CLK_RECOVER_INT_STATUS;
    if (status) {
        /* USB RECOVER interrupt is happenned */
        if (status & USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_MASK) {
            /* Indicates that the USB clock recovery algorithm has detected
             * that the frequency trim adjustment needed for the IRC48M output
             * clock is outside the available TRIM_FINE adjustment range for
             * the IRC48M module.
             */
        }
        regs->CLK_RECOVER_INT_STATUS = status;
    }
#endif
}

void
USB0_IRQHandler(void)
{
    kinetis_usb_dev_isr();
}

static const usb_device_controller_interface_struct_t kinetis_interface = {
    _kinetis_usb_init,
    _kinetis_usb_deinit,
    _kinetis_usb_send,
    _kinetis_usb_recv,
    _kinetis_usb_cancel,
    _kinetis_usb_control,
};

usb_status_t
USB_DeviceErrorNotification(usb_dev_t *dev,
                            usb_device_callback_message_struct_t *message)
{
    return dev->devcb(dev, kUSB_DeviceEventError, NULL);
}

void
usb_hal_init_clocks(void)
{
    //SystemCoreClockUpdate();
    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, 48000000);
}

const usb_device_controller_interface_struct_t *
usb_hal_controller_interface(void)
{
    return &kinetis_interface;
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
usb_hal_clear_memory(void)
{
    /* Not needed for kinetis */
}
