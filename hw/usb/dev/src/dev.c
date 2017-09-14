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
#include <dev/ch9.h>

#include <hal_usb/hal_usb.h>

#include <os/os.h>

static usb_dev_t s_UsbDevice[MYNEWT_VAL(USB_DEVICE_CONFIG_NUM)];
static struct os_eventq notification_queue;
static struct os_event ev;
static usb_dev_cb_msg_t message;

//FIXME
//static int mhead = 0;
//static int mtail = 0;
//static usb_dev_cb_msg_t messages[16];

/*!
 * This function allocates a device handle.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param handle          It is out parameter, is used to return pointer of the device handle to the caller.
 */
static usb_status_t
_usb_device_alloc_handle(uint8_t controllerId, usb_dev_t **dev)
{
    uint32_t i;
    os_sr_t sr;
    usb_status_t status = kStatus_USB_Busy;

    OS_ENTER_CRITICAL(sr);

    /* Check the controller is initialized or not. */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDevice[i].ctrl_handle &&
                s_UsbDevice[i].controllerId == controllerId) {
            status = kStatus_USB_Error;
            goto done;
        }
    }

    /* Get a free device handle. */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (!s_UsbDevice[i].ctrl_handle) {
            s_UsbDevice[i].controllerId = controllerId;
            *dev = &s_UsbDevice[i];
            status = kStatus_USB_Success;
            goto done;
        }
    }

done:
    OS_EXIT_CRITICAL(sr);
    return status;
}

/*!
 * This function frees a device handle.
 *
 * @param handle          The device handle.
 */
static usb_status_t
_usb_device_free_handle(usb_dev_t *dev)
{
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);
    dev->ctrl_handle = NULL;
    dev->controllerId = 0;
    OS_EXIT_CRITICAL(sr);

    return kStatus_USB_Success;
}

#if 0
static usb_status_t
_usb_device_get_controller_interface(uint8_t controllerId, const usb_dev_ctrl_itf_t **ctrl_itf)
{
    return error;
}
#endif

static usb_status_t
_usb_device_transfer(usb_device_handle handle, uint8_t ep_addr, uint8_t *buf,
                     uint32_t len)
{
    usb_dev_t *dev = (usb_dev_t *)handle;
    uint8_t ep = USB_EP_NUMBER(ep_addr);
    uint8_t dir = USB_EP_DIR(ep_addr);
    usb_status_t err = kStatus_USB_Error;

    if (!dev) {
        return kStatus_USB_InvalidHandle;
    }

    if (!dev->ctrl_itf) {
        return kStatus_USB_ControllerNotFound;
    }

    if (dev->epcbs[(ep << 1) | dir].busy) {
        return kStatus_USB_Busy;
    }

    dev->epcbs[(ep << 1) | dir].busy = 1;

    if (dir) {
#if defined(USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE)
        if (len) {
            USB_CacheFlushLines((void *)buf, len);
        }
#endif
        err = dev->ctrl_itf->send(dev->ctrl_handle, ep_addr, buf, len);
    } else {
#if defined(USB_DEVICE_CONFIG_BUFFER_PROPERTY_CACHEABLE)
        if (len) {
            USB_CacheInvalidateLines((void *)buf, len);
        }
#endif
        err = dev->ctrl_itf->recv(dev->ctrl_handle, ep_addr, buf, len);
    }
    return err;
}

static usb_status_t
_usb_device_control(usb_device_handle handle, usb_device_control_type_t type, void *param)
{
    usb_dev_t *dev = (usb_dev_t *) handle;

    if (!dev) {
        return kStatus_USB_InvalidHandle;
    }

    if (!dev->ctrl_itf) {
        return kStatus_USB_ControllerNotFound;
    }

    return dev->ctrl_itf->control(dev->ctrl_handle, type, param);
}

static usb_status_t
_usb_device_reset_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    int i;

    dev->isResetting = 1;

#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    dev->remotewakeup = 0;
#endif

    _usb_device_control(dev, USB_DEV_CTRL_SET_DFLT_STATUS, NULL);

    dev->state = kUSB_DeviceStateDefault;
    dev->deviceAddress = 0;

    for (i = 0; i < (MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS) * 2); i++) {
        dev->epcbs[i].fn = NULL;
        dev->epcbs[i].param = NULL;
        dev->epcbs[i].busy = 0;
    }

    dev->devcb(dev, kUSB_DeviceEventBusReset, NULL);

    dev->isResetting = 0;
    return kStatus_USB_Success;
}

#if MYNEWT_VAL(USB_DEVICE_CONFIG_LOW_POWER_MODE)
static usb_status_t
_usb_device_suspend_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    return dev->devcb(handle, kUSB_DeviceEventSuspend, NULL);
}

static usb_status_t
_usb_device_resume_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    return dev->devcb(handle, kUSB_DeviceEventResume, NULL);
}
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_DETACH_ENABLE)
static usb_status_t
_usb_device_detach_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    return dev->devcb(handle, kUSB_DeviceEventDetach, NULL);
}

static usb_status_t
_usb_device_attach_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    return dev->devcb(handle, kUSB_DeviceEventAttach, NULL);
}
#endif

static usb_status_t
_usb_device_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    uint8_t ep = USB_EP_NUMBER(msg->code);
    uint8_t dir = USB_EP_DIR(msg->code);
    usb_status_t err = kStatus_USB_Error;
    usb_dev_ep_cb_t *epcb = NULL;
    usb_dev_ep_cb_msg_t cb_msg;

    switch (msg->code) {
    case kUSB_DeviceNotifyBusReset:
        err = _usb_device_reset_notification(dev, msg);
        break;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_LOW_POWER_MODE)
    case kUSB_DeviceNotifySuspend:
        err = _usb_device_suspend_notification(dev, msg);
        break;
    case kUSB_DeviceNotifyResume:
        err = _usb_device_resume_notification(dev, msg);
        break;
#endif

        //FIXME
#if defined(USB_DEVICE_CONFIG_ERROR_HANDLING)
    case kUSB_DeviceNotifyError:
        err = USB_DeviceErrorNotification(dev, msg);
        break;
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_DETACH_ENABLE)
    case kUSB_DeviceNotifyDetach:
        err = _usb_device_detach_notification(dev, msg);
        break;
    case kUSB_DeviceNotifyAttach:
        err = _usb_device_attach_notification(dev, msg);
        break;
#endif

    default:
        if (ep < MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
            epcb = &dev->epcbs[(ep << 1) | dir];
            if (epcb->fn) {
                cb_msg.buf = msg->buf;
                cb_msg.len = msg->len;
                cb_msg.setup = msg->setup;
                if (msg->setup) {
                    dev->epcbs[0].busy = 0;
                    dev->epcbs[1].busy = 0;
                } else {
                    epcb->busy = 0;
                }
                /*
                 * calls into higher-level class function
                 */
                err = epcb->fn(dev, &cb_msg, epcb->param);
            }
        }
        break;
    }
    return err;
}

usb_status_t
usb_dev_notify(void *handle, void *msg)
{
    usb_dev_t *dev = (usb_dev_t *) handle;
    //usb_dev_cb_msg_t *message = (usb_dev_cb_msg_t *)msg;

    memcpy(&message, (usb_dev_cb_msg_t *) msg, sizeof message);

    if (!msg || !handle) {
        return kStatus_USB_InvalidHandle;
    }

    if (!dev->devcb) {
        return kStatus_USB_Error;
    }

    if (dev->isResetting) {
        if (USB_EP_NUMBER(message.code) && !(message.code & 0x70)) {
            return _usb_device_notification(dev, &message);
        }
    }

    //FIXME: are events copied???
    ev.ev_arg = &message;
    os_eventq_put(dev->notificationQueue, &ev);

    return 0;
}

usb_status_t
usb_dev_init(uint8_t controllerId, usb_device_callback_t devcb,
             usb_device_handle *handle)
{
    usb_dev_t *dev = NULL;
    usb_status_t err;
    int i;

    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    err = _usb_device_alloc_handle(controllerId, &dev);
    if (err) {
        return err;
    }

    dev->devcb = devcb;
    dev->controllerId = controllerId;
    dev->deviceAddress = 0;
    dev->isResetting = 0;

    for (i = 0; i < (MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS) * 2); i++) {
        dev->epcbs[i].fn = NULL;
        dev->epcbs[i].param = NULL;
        dev->epcbs[i].busy = 0;
    }

    dev->ctrl_itf = usb_hal_controller_interface();
    if (!dev->ctrl_itf) {
        _usb_device_free_handle(dev);
        return kStatus_USB_ControllerNotFound;
    }
    if (!dev->ctrl_itf->init || !dev->ctrl_itf->deinit || !dev->ctrl_itf->send ||
        !dev->ctrl_itf->recv || !dev->ctrl_itf->cancel || !dev->ctrl_itf->control) {
        _usb_device_free_handle(dev);
        return kStatus_USB_InvalidControllerInterface;
    }

    os_eventq_init(&notification_queue);
    dev->notificationQueue = &notification_queue;

    err = dev->ctrl_itf->init(controllerId, dev, &dev->ctrl_handle);
    if (err != kStatus_USB_Success) {
        usb_device_deinit(dev);
        return err;
    }

    dev->state = kUSB_DeviceStateDefault;
    *handle = dev;

    return err;
}

usb_status_t
usb_device_run(usb_device_handle handle)
{
    return _usb_device_control(handle, USB_DEV_CTRL_RUN, NULL);
}

usb_status_t
usb_device_stop(usb_device_handle handle)
{
    return _usb_device_control(handle, USB_DEV_CTRL_STOP, NULL);
}

usb_status_t
usb_device_deinit(usb_device_handle handle)
{
    usb_dev_t *dev = (usb_dev_t *) handle;

    if (!dev) {
        return kStatus_USB_InvalidHandle;
    }

    if (dev->ctrl_itf) {
        dev->ctrl_itf->deinit(dev->ctrl_handle);
        dev->ctrl_itf = NULL;
    }

    if (dev->notificationQueue) {
        dev->notificationQueue = NULL;
    }

    _usb_device_free_handle(dev);
    return kStatus_USB_Success;
}

usb_status_t
usb_device_send_req(usb_device_handle handle, uint8_t ep, uint8_t *buf, uint32_t len)
{
    return _usb_device_transfer(handle, 0x80 | USB_EP_NUMBER(ep), buf, len);
}

usb_status_t
usb_device_recv_req(usb_device_handle handle, uint8_t ep, uint8_t *buf, uint32_t len)
{
    return _usb_device_transfer(handle, USB_EP_NUMBER(ep), buf, len);
}

usb_status_t
usb_device_cancel(usb_device_handle handle, uint8_t ep)
{
    usb_dev_t *dev = (usb_dev_t *) handle;

    if (!dev) {
        return kStatus_USB_InvalidHandle;
    }

    if (!dev->ctrl_itf) {
        return kStatus_USB_ControllerNotFound;
    }

    return dev->ctrl_itf->cancel(dev->ctrl_handle, ep);
}

usb_status_t
usb_dev_ep_init(usb_device_handle handle, usb_dev_ep_init_t *ep_init,
                usb_dev_ep_cb_t *ep_cb)
{
    usb_dev_t *dev = (usb_dev_t *) handle;
    uint8_t endpoint;
    uint8_t direction;
    uint8_t epidx;

    if (!dev) {
        return kStatus_USB_InvalidHandle;
    }

    if (!ep_init || !ep_cb) {
        return kStatus_USB_InvalidParameter;
    }

    endpoint = USB_EP_NUMBER(ep_init->endpointAddress);
    direction = USB_EP_DIR(ep_init->endpointAddress);

    if (endpoint >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return kStatus_USB_InvalidParameter;
    }

    epidx = (uint8_t)((uint32_t)endpoint << 1) | direction;
    dev->epcbs[epidx].fn = ep_cb->fn;
    dev->epcbs[epidx].param = ep_cb->param;
    dev->epcbs[epidx].busy = 0;

    return _usb_device_control(handle, USB_DEV_CTRL_EP_INIT, ep_init);
}

usb_status_t
usb_dev_ep_deinit(usb_device_handle handle, uint8_t ep_addr)
{
    usb_dev_t *dev = (usb_dev_t *)handle;
    uint8_t ep = USB_EP_NUMBER(ep_addr);
    uint8_t dir = USB_EP_DIR(ep_addr);
    uint8_t epidx;
    usb_status_t err = kStatus_USB_Error;

    if (!dev) {
        return kStatus_USB_InvalidHandle;
    }

    err = _usb_device_control(handle, USB_DEV_CTRL_EP_DEINIT, &ep_addr);
    if (ep >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return kStatus_USB_InvalidParameter;
    }

    epidx = (ep << 1) | dir;
    dev->epcbs[epidx].fn = NULL;
    dev->epcbs[epidx].param = NULL;
    dev->epcbs[epidx].busy = 0;

    return err;
}

usb_status_t
usb_dev_ep_stall(usb_device_handle handle, uint8_t ep_addr)
{
    if (USB_EP_NUMBER(ep_addr) >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return kStatus_USB_InvalidParameter;
    }

    return _usb_device_control(handle, USB_DEV_CTRL_EP_STALL, &ep_addr);
}

usb_status_t
usb_dev_ep_unstall(usb_device_handle handle, uint8_t ep_addr)
{
    if (USB_EP_NUMBER(ep_addr) >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return kStatus_USB_InvalidParameter;
    }

    return _usb_device_control(handle, USB_DEV_CTRL_EP_UNSTALL, &ep_addr);
}

usb_status_t
usb_dev_get_status(usb_device_handle handle, usb_device_status_t type, void *param)
{
    uint8_t *temp8;
    usb_status_t error = kStatus_USB_Error;

    if (!param) {
        return kStatus_USB_InvalidParameter;
    }

    switch (type) {
        case kUSB_DeviceStatusSpeed:
            error =
                _usb_device_control(handle, USB_DEV_CTRL_GET_SPEED, param);
            break;
        case kUSB_DeviceStatusOtg:
            error = _usb_device_control(handle, USB_DEV_CTRL_GET_OTG_STATUS, param);
            break;
        case kUSB_DeviceStatusDeviceState:
            temp8 = (uint8_t *)param;
            error = kStatus_USB_Success;
            *temp8 = ((usb_dev_t *)handle)->state;
            break;
        case kUSB_DeviceStatusAddress:
            temp8 = (uint8_t *)param;
            error = kStatus_USB_Success;
            *temp8 = ((usb_dev_t *)handle)->deviceAddress;
            break;
        case kUSB_DeviceStatusDevice:
            error = _usb_device_control(handle, USB_DEV_CTRL_GET_STATUS, param);
            break;
        case kUSB_DeviceStatusEndpoint:
            error = _usb_device_control(handle, USB_DEV_CTRL_GET_EP_STATUS, param);
            break;
        case kUSB_DeviceStatusSynchFrame:
            error = _usb_device_control(handle, USB_DEV_CTRL_GET_SYNCF, param);
            break;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
        case kUSB_DeviceStatusRemoteWakeup:
            temp8 = (uint8_t *)param;
            error = kStatus_USB_Success;
            *temp8 = ((usb_dev_t *)handle)->remotewakeup;
            break;
#endif
        default:
            break;
    }
    return error;
}

usb_status_t
usb_dev_set_status(usb_device_handle handle, usb_device_status_t type, void *param)
{
    usb_status_t error = kStatus_USB_Error;

    switch (type) {
//FIXME
#if MYNEWT_VAL(USB_DEVICE_CONFIG_EHCI) && MYNEWT_VAL(USB_DEVICE_CONFIG_EHCI_TEST_MODE)
        case kUSB_DeviceStatusTestMode:
            error = _usb_device_control(handle, USB_DEV_CTRL_SET_TEST_MODE, param);
            break;
#endif
        case kUSB_DeviceStatusOtg:
            error = _usb_device_control(handle, USB_DEV_CTRL_SET_OTG_STATUS, param);
            break;
        case kUSB_DeviceStatusDeviceState:
            if (param) {
                error = kStatus_USB_Success;
                ((usb_dev_t *)handle)->state = *(uint8_t *)param;
            }
            break;
        case kUSB_DeviceStatusAddress:
            if (((usb_dev_t *)handle)->state != kUSB_DeviceStateAddressing) {
                if (param) {
                    error = kStatus_USB_Success;
                    ((usb_dev_t *)handle)->deviceAddress = *(uint8_t *)param;
                    ((usb_dev_t *)handle)->state = kUSB_DeviceStateAddressing;
                    //printf("set addr %d\n", ((usb_dev_t *)handle)->deviceAddress);
                }
            } else {
                error = _usb_device_control(handle, USB_DEV_CTRL_SET_ADDR,
                                            &((usb_dev_t *)handle)->deviceAddress);
                //printf("set addr %d\n", error);
            }
            break;
        case kUSB_DeviceStatusBusResume:
            error = _usb_device_control(handle, USB_DEV_CTRL_RESUME, param);
            break;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
        case kUSB_DeviceStatusRemoteWakeup:
            if (param) {
                error = kStatus_USB_Success;
                ((usb_dev_t *)handle)->remotewakeup = *(uint8_t *)param;
            }
            break;
#endif
        case kUSB_DeviceStatusBusSuspend:
            error = _usb_device_control(handle, USB_DEV_CTRL_SUSPEND, param);
            break;
        default:
            break;
    }

    return error;
}

/*
 * This function is used to handle controller message.
 * This function should not be called in application directly.
 */
void
usb_device_task_fn(void *deviceHandle)
{
    usb_dev_t *dev = (usb_dev_t *) deviceHandle;
    struct os_event *ev;

    if (dev) {
        ev = os_eventq_get(dev->notificationQueue);
        _usb_device_notification(dev, ev->ev_arg);
    }
}

#if 0
void
usb_device_get_version(uint32_t *version)
{
    if (version) {
        *version = (uint32_t)USB_MAKE_VERSION(USB_STACK_VERSION_MAJOR,
                                              USB_STACK_VERSION_MINOR,
                                              USB_STACK_VERSION_BUGFIX);
    }
}
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
usb_status_t
usb_device_update_hw_tick(usb_device_handle handle, uint64_t tick)
{
    if (!handle) {
        return kStatus_USB_InvalidHandle;
    }

    ((usb_dev_t *)handle)->hwTick = tick;
    return kStatus_USB_Success;
}
#endif
