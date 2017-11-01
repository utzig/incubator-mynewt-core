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
#include <assert.h>

#include <os/os.h>

static usb_dev_t s_UsbDevice[MYNEWT_VAL(USB_DEVICE_CONFIG_NUM)];

#define CB_SZ 4
typedef struct
{
    usb_dev_cb_msg_t   buf[CB_SZ];
    size_t             sz;
    int                headidx;
    int                tailidx;
    struct os_mutex    mtx;
    struct os_eventq   evtq;
    struct os_event    evs[CB_SZ];
} usb_msg_cb_t;

static usb_msg_cb_t g_usb_msg;

static void
usb_msg_init(void)
{
    usb_msg_cb_t *p = &g_usb_msg;
    os_mutex_init(&p->mtx);
    os_eventq_init(&p->evtq);
    p->sz = 0;
    p->headidx = 0;
    p->tailidx = 0;
}

static void
usb_msg_deinit(void)
{
    usb_msg_cb_t *p = &g_usb_msg;
    p->sz = 0;
    p->headidx = 0;
    p->tailidx = 0;
}

static int
usb_msg_put(usb_dev_cb_msg_t *msg)
{
    usb_msg_cb_t *p = &g_usb_msg;
    int evidx = -1;

    os_mutex_pend(&p->mtx, (uint32_t)-1);
    if (p->sz == CB_SZ) {
        goto out;
    }
    memcpy(&p->buf[p->headidx], msg, sizeof(usb_dev_cb_msg_t));
    evidx = p->headidx;
    p->headidx = (p->headidx + 1) % CB_SZ;
    p->sz++;

out:
    os_mutex_release(&p->mtx);

    //FIXME: I'm doing this to wake the waiting thread, there must be a
    //simpler way to do this!!!
    os_eventq_put(&p->evtq, &p->evs[evidx]);

    return 0;
}

static usb_dev_cb_msg_t *
usb_msg_get(void)
{
    usb_msg_cb_t *p = &g_usb_msg;
    usb_dev_cb_msg_t *msg = NULL;

    os_mutex_pend(&p->mtx, (uint32_t)-1);
    if (p->sz == 0) {
        goto out;
    }
    msg = &p->buf[p->tailidx];
    p->tailidx = (p->tailidx + 1) % CB_SZ;
    p->sz--;

out:
    os_mutex_release(&p->mtx);
    return msg;
}

/*!
 * This function allocates a device handle.
 *
 * @param controllerId   The controller id of the USB IP. Please refer to the enumeration usb_controller_index_t.
 * @param handle          It is out parameter, is used to return pointer of the device handle to the caller.
 */
static int
_usb_device_alloc_handle(uint8_t controllerId, usb_dev_t **dev)
{
    uint32_t i;
    os_sr_t sr;
    int err = USB_BUSY;

    OS_ENTER_CRITICAL(sr);

    /* Check the controller is initialized or not. */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (s_UsbDevice[i].ctrl_handle &&
                s_UsbDevice[i].controllerId == controllerId) {
            err = USB_ERR;
            goto done;
        }
    }

    /* Get a free device handle. */
    for (i = 0; i < MYNEWT_VAL(USB_DEVICE_CONFIG_NUM); i++) {
        if (!s_UsbDevice[i].ctrl_handle) {
            s_UsbDevice[i].controllerId = controllerId;
            *dev = &s_UsbDevice[i];
            err = 0;
            goto done;
        }
    }

done:
    OS_EXIT_CRITICAL(sr);
    return err;
}

static int
_usb_device_free_handle(usb_dev_t *dev)
{
    os_sr_t sr;

    OS_ENTER_CRITICAL(sr);
    dev->ctrl_handle = NULL;
    dev->controllerId = 0;
    OS_EXIT_CRITICAL(sr);

    return 0;
}

#if 0
static int
_usb_device_get_controller_interface(uint8_t controllerId, const usb_dev_ctrl_itf_t **ctrl_itf)
{
    return error;
}
#endif

static int
_usb_device_transfer(usb_device_handle handle, uint8_t ep_addr, uint8_t *buf,
        uint32_t len)
{
    usb_dev_t *dev = (usb_dev_t *)handle;
    uint8_t ep = USB_EP_NUMBER(ep_addr);
    uint8_t dir = USB_EP_DIR(ep_addr);
    int err = USB_ERR;

    if (!dev) {
        return USB_INVALID_HANDLE;
    }

    if (!dev->ctrl_itf) {
        return USB_CTRL_NOT_FOUND;
    }

    if (dev->epcbs[(ep << 1) | dir].busy) {
        return USB_BUSY;
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

static int
_usb_device_control(usb_device_handle handle, usb_device_control_type_t type, void *param)
{
    usb_dev_t *dev = (usb_dev_t *) handle;

    if (!dev) {
        return USB_INVALID_HANDLE;
    }

    if (!dev->ctrl_itf) {
        return USB_CTRL_NOT_FOUND;
    }

    return dev->ctrl_itf->control(dev->ctrl_handle, type, param);
}

static int
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
    return 0;
}

#if MYNEWT_VAL(USB_DEV_LOW_POWER_MODE)
static int
_usb_device_suspend_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    return dev->devcb(handle, kUSB_DeviceEventSuspend, NULL);
}

static int
_usb_device_resume_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    return dev->devcb(handle, kUSB_DeviceEventResume, NULL);
}
#endif

#if MYNEWT_VAL(USB_DEVICE_CONFIG_DETACH_ENABLE)
static int
_usb_device_detach_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    return dev->devcb(handle, kUSB_DeviceEventDetach, NULL);
}

static int
_usb_device_attach_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    return dev->devcb(handle, kUSB_DeviceEventAttach, NULL);
}
#endif

static int
_usb_device_notification(usb_dev_t *dev, usb_dev_cb_msg_t *msg)
{
    uint8_t ep = USB_EP_NUMBER(msg->code);
    uint8_t dir = USB_EP_DIR(msg->code);
    int err = USB_ERR;
    usb_dev_ep_cb_t *epcb = NULL;
    usb_dev_ep_cb_msg_t cb_msg;

    switch (msg->code) {
    case kUSB_DeviceNotifyBusReset:
        err = _usb_device_reset_notification(dev, msg);
        break;
#if MYNEWT_VAL(USB_DEV_LOW_POWER_MODE)
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
            //printf("ep=%d, dir=%d\n", ep, dir);
            epcb = &dev->epcbs[(ep << 1) | dir];
            if (epcb->fn) {
                cb_msg.buf = msg->buf;
                cb_msg.len = msg->len;
                cb_msg.setup = msg->setup;
                cb_msg.is_in = dir;
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

int
usb_dev_notify(void *handle, usb_dev_cb_msg_t *msg)
{
    usb_dev_t *dev = (usb_dev_t *) handle;

    if (!msg || !handle) {
        return USB_INVALID_HANDLE;
    }

    if (!dev->devcb) {
        return USB_ERR;
    }

    if (dev->isResetting) {
        //assert(0);
        if (USB_EP_NUMBER(msg->code) && !(msg->code & 0x70)) {
            /*
             * FIXME: this is not re-entrant... if a second interrupt happens
             * while the first one is being processed, both will be writing to
             * the same var.
             */
            return _usb_device_notification(dev, msg);
        }
    }

    usb_msg_put(msg);

    return 0;
}

int
usb_dev_init(uint8_t controllerId, usb_device_callback_t devcb,
             usb_device_handle *handle)
{
    usb_dev_t *dev = NULL;
    int err;
    int i;

    if (!handle) {
        return USB_INVALID_HANDLE;
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
        return USB_CTRL_NOT_FOUND;
    }
    if (!dev->ctrl_itf->init || !dev->ctrl_itf->deinit || !dev->ctrl_itf->send ||
        !dev->ctrl_itf->recv || !dev->ctrl_itf->cancel || !dev->ctrl_itf->control) {
        _usb_device_free_handle(dev);
        return USB_INV_CTRL_ITF;
    }

    usb_msg_init();

    err = dev->ctrl_itf->init(controllerId, dev, &dev->ctrl_handle);
    if (err) {
        usb_device_deinit(dev);
        return err;
    }

    dev->state = kUSB_DeviceStateDefault;
    *handle = dev;

    return err;
}

int
usb_device_run(usb_device_handle handle)
{
    return _usb_device_control(handle, USB_DEV_CTRL_RUN, NULL);
}

int
usb_device_stop(usb_device_handle handle)
{
    return _usb_device_control(handle, USB_DEV_CTRL_STOP, NULL);
}

int
usb_device_deinit(usb_device_handle handle)
{
    usb_dev_t *dev = (usb_dev_t *)handle;

    if (!dev) {
        return USB_INVALID_HANDLE;
    }

    if (dev->ctrl_itf) {
        dev->ctrl_itf->deinit(dev->ctrl_handle);
        dev->ctrl_itf = NULL;
    }

    usb_msg_deinit();

    _usb_device_free_handle(dev);
    return 0;
}

int
usb_device_send_req(usb_device_handle handle, uint8_t ep, uint8_t *buf, uint32_t len)
{
    return _usb_device_transfer(handle, 0x80 | USB_EP_NUMBER(ep), buf, len);
}

int
usb_device_recv_req(usb_device_handle handle, uint8_t ep, uint8_t *buf, uint32_t len)
{
    return _usb_device_transfer(handle, USB_EP_NUMBER(ep), buf, len);
}

int
usb_device_cancel(usb_device_handle handle, uint8_t ep)
{
    usb_dev_t *dev = (usb_dev_t *)handle;

    if (!dev) {
        return USB_INVALID_HANDLE;
    }

    if (!dev->ctrl_itf) {
        return USB_CTRL_NOT_FOUND;
    }

    return dev->ctrl_itf->cancel(dev->ctrl_handle, ep);
}

int
usb_dev_ep_init(usb_device_handle handle, usb_dev_ep_init_t *ep_init,
                usb_dev_ep_cb_t *ep_cb)
{
    usb_dev_t *dev = (usb_dev_t *)handle;
    uint8_t ep;
    uint8_t dir;
    uint8_t epidx;

    if (!dev) {
        return USB_INVALID_HANDLE;
    }

    if (!ep_init || !ep_cb) {
        return USB_INVALID_PARAM;
    }

    ep = USB_EP_NUMBER(ep_init->endpointAddress);
    dir = USB_EP_DIR(ep_init->endpointAddress);

    if (ep >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return USB_INVALID_PARAM;
    }

    epidx = (uint8_t)((uint32_t)ep << 1) | dir;
    dev->epcbs[epidx].fn = ep_cb->fn;
    dev->epcbs[epidx].param = ep_cb->param;
    dev->epcbs[epidx].busy = 0;

    return _usb_device_control(handle, USB_DEV_CTRL_EP_INIT, ep_init);
}

int
usb_dev_ep_deinit(usb_device_handle handle, uint8_t ep_addr)
{
    usb_dev_t *dev = (usb_dev_t *)handle;
    uint8_t ep = USB_EP_NUMBER(ep_addr);
    uint8_t dir = USB_EP_DIR(ep_addr);
    uint8_t epidx;
    int err = USB_ERR;

    if (!dev) {
        return USB_INVALID_HANDLE;
    }

    err = _usb_device_control(handle, USB_DEV_CTRL_EP_DEINIT, &ep_addr);
    if (ep >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return USB_INVALID_PARAM;
    }

    epidx = (ep << 1) | dir;
    dev->epcbs[epidx].fn = NULL;
    dev->epcbs[epidx].param = NULL;
    dev->epcbs[epidx].busy = 0;

    return err;
}

int
usb_dev_ep_stall(usb_device_handle handle, uint8_t ep_addr)
{
    if (USB_EP_NUMBER(ep_addr) >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return USB_INVALID_PARAM;
    }

    return _usb_device_control(handle, USB_DEV_CTRL_EP_STALL, &ep_addr);
}

int
usb_dev_ep_unstall(usb_device_handle handle, uint8_t ep_addr)
{
    if (USB_EP_NUMBER(ep_addr) >= MYNEWT_VAL(USB_DEVICE_CONFIG_ENDPOINTS)) {
        return USB_INVALID_PARAM;
    }

    return _usb_device_control(handle, USB_DEV_CTRL_EP_UNSTALL, &ep_addr);
}

int
usb_dev_get_status(usb_device_handle handle, usb_device_status_t type, void *param)
{
    int err = 0;

    if (!param) {
        return USB_INVALID_PARAM;
    }

    switch (type) {
    case kUSB_DeviceStatusSpeed:
        err = _usb_device_control(handle, USB_DEV_CTRL_GET_SPEED, param);
        break;
    case kUSB_DeviceStatusOtg:
        err = _usb_device_control(handle, USB_DEV_CTRL_GET_OTG_STATUS, param);
        break;
    case kUSB_DeviceStatusDeviceState:
        *((uint8_t *)param) = ((usb_dev_t *)handle)->state;
        break;
    case kUSB_DeviceStatusAddress:
        *((uint8_t *)param) = ((usb_dev_t *)handle)->deviceAddress;
        break;
    case kUSB_DeviceStatusDevice:
        err = _usb_device_control(handle, USB_DEV_CTRL_GET_STATUS, param);
        break;
    case kUSB_DeviceStatusEndpoint:
        err = _usb_device_control(handle, USB_DEV_CTRL_GET_EP_STATUS, param);
        break;
    case kUSB_DeviceStatusSynchFrame:
        err = _usb_device_control(handle, USB_DEV_CTRL_GET_SYNCF, param);
        break;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    case kUSB_DeviceStatusRemoteWakeup:
        *((uint8_t *)param) = ((usb_dev_t *)handle)->remotewakeup;
        break;
#endif
    default:
        err = USB_ERR;
        break;
    }
    return err;
}

int
usb_dev_set_status(usb_device_handle handle, usb_device_status_t type, void *param)
{
    int err = USB_ERR;

    switch (type) {
//FIXME
#if MYNEWT_VAL(USB_DEVICE_CONFIG_EHCI) && MYNEWT_VAL(USB_DEVICE_CONFIG_EHCI_TEST_MODE)
    case kUSB_DeviceStatusTestMode:
        err = _usb_device_control(handle, USB_DEV_CTRL_SET_TEST_MODE, param);
        break;
#endif
    case kUSB_DeviceStatusOtg:
        err = _usb_device_control(handle, USB_DEV_CTRL_SET_OTG_STATUS, param);
        break;
    case kUSB_DeviceStatusDeviceState:
        if (param) {
            ((usb_dev_t *)handle)->state = *(uint8_t *)param;
            err = 0;
        }
        break;
    case kUSB_DeviceStatusAddress:
#if 1
        if (((usb_dev_t *)handle)->state != kUSB_DeviceStateAddressing) {
            if (param) {
                ((usb_dev_t *)handle)->deviceAddress = *(uint8_t *)param;
                ((usb_dev_t *)handle)->state = kUSB_DeviceStateAddressing;
                err = 0;
            }
        } else {
            err = _usb_device_control(handle, USB_DEV_CTRL_SET_ADDR,
                                        &((usb_dev_t *)handle)->deviceAddress);
        }
#else /* FIXME: the commented block above works on kinetis, below stm32 test */
        if (param) {
                ((usb_dev_t *)handle)->deviceAddress = *(uint8_t *)param;
                ((usb_dev_t *)handle)->state = kUSB_DeviceStateAddressing;
                err = _usb_device_control(handle, USB_DEV_CTRL_SET_ADDR,
                                          &((usb_dev_t *)handle)->deviceAddress);
                if (!err) {
                    ((usb_dev_t *)handle)->state = kUSB_DeviceStateAddress;
                }
        }
#endif
        break;
    case kUSB_DeviceStatusBusResume:
        err = _usb_device_control(handle, USB_DEV_CTRL_RESUME, param);
        break;
#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
    case kUSB_DeviceStatusRemoteWakeup:
        if (param) {
            ((usb_dev_t *)handle)->remotewakeup = *(uint8_t *)param;
            err = 0;
        }
        break;
#endif
    case kUSB_DeviceStatusBusSuspend:
        err = _usb_device_control(handle, USB_DEV_CTRL_SUSPEND, param);
        break;
    default:
        break;
    }

    return err;
}

/*
 * This function is used to handle controller message.
 * This function should not be called in application directly.
 */
bool
usb_device_task_fn(void *deviceHandle)
{
    usb_dev_t *dev = (usb_dev_t *) deviceHandle;
    usb_dev_cb_msg_t *msg;
    bool handled = false;
    //struct os_event *evt;

    if (dev) {
        (void)os_eventq_get(&g_usb_msg.evtq);
        msg = usb_msg_get();
#if 0
        printf("evt=%p, msg=%p\n", evt, msg);
        if (msg) printf("msg->code=%02x\n", msg->code);
#endif
        if (msg) {
            _usb_device_notification(dev, msg);
            handled = true;
        }
    }

    return handled;
}

#if MYNEWT_VAL(USB_DEVICE_CONFIG_REMOTE_WAKEUP)
int
usb_device_update_hw_tick(usb_device_handle handle, uint64_t tick)
{
    if (!handle) {
        return USB_INVALID_HANDLE;
    }

    ((usb_dev_t *)handle)->hwTick = tick;
    return 0;
}
#endif
