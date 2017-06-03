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

#include "os/os.h"
#include "os_priv.h"

#include <hal/hal_bsp.h>
#include <hal/hal_os_tick.h>
#include <mcu/encoding.h>
#include <mcu/platform.h>

struct context_switch_frame {
    uint32_t  pc;
    /* Callee saved registers */
    uint32_t  s0;
    uint32_t  s1;
    uint32_t  s2;
    uint32_t  s3;
    uint32_t  s4; 
    uint32_t  s5;
    uint32_t  s6;
    uint32_t  s7;
    uint32_t  s8;
    uint32_t  s9;
    uint32_t  s10;
    uint32_t  s11;
    /* Caller saved register */
    uint32_t  ra;
    uint32_t  gp;
    uint32_t  tp;
    uint32_t  t0;
    uint32_t  t1; 
    uint32_t  t2;
    uint32_t  t3;
    uint32_t  t4;
    uint32_t  t5;
    uint32_t  t6;
    uint32_t  a0;
    uint32_t  a1;
    uint32_t  a2;
    uint32_t  a3;
    uint32_t  a4;
    uint32_t  a5;
    uint32_t  a6;
    uint32_t  a7;
};

uint32_t mtime_lo(void)
{
    return CLINT_REG(CLINT_MTIME);
}

uint32_t mtime_hi(void)
{
    return CLINT_REG(CLINT_MTIME + 4);
}

uint64_t get_timer_value(void)
{
    while (1) {
        uint32_t hi = mtime_hi();
        uint32_t lo = mtime_lo();
        if (hi == mtime_hi())
            return ((uint64_t)hi << 32) | lo;
    }
}

void set_mtimecmp(uint64_t time)
{
    CLINT_REG(CLINT_MTIMECMP + 4) = -1;
    CLINT_REG(CLINT_MTIMECMP) = (uint32_t) time;
    CLINT_REG(CLINT_MTIMECMP + 4) = (uint32_t) (time >> 32);
}

unsigned long get_timer_freq()
{
  return 32768;
}

/* XXX: determine how to deal with running un-privileged */
/* only priv currently supported */
uint32_t os_flags = OS_RUN_PRIV;

extern struct os_task g_idle_task;

#define OS_TICK_PRIO 0

static int
os_in_isr(void)
{
    // TODO:
    return 0;
}

void
timer_handler(void)
{
    os_time_advance(1);
}

void
os_arch_ctx_sw(struct os_task *t)
{
    if ((os_sched_get_current_task() != 0) && (t != 0)) {
        os_sched_ctx_sw_hook(t);
    }

#if CONTEXT_SWITCH_ON_ECALL
    /*
     * Synchronous context switch does not work right now due to
     * interrupts being disabled in os_sched().
     * The os_sched() function expects that os_arc_ctx_sw() will end before
     * actual context switch which is not the case when ecall us used.
     */
    asm("ecall");
#else
    /*
     * This request software interrupt in case context switch
     * should be done by interrup instead of ECALL
     */
    CLINT_REG(CLINT_MSIP) = 1;
#endif
}

os_sr_t
os_arch_save_sr(void)
{
    uint32_t isr_ctx;

    isr_ctx = clear_csr(mstatus, MSTATUS_MIE) & MSTATUS_MIE;

    return isr_ctx;
}

void
os_arch_restore_sr(os_sr_t isr_ctx)
{
    if (isr_ctx) {
        set_csr(mstatus, MSTATUS_MIE);
    }
}

int
os_arch_in_critical(void)
{
    return !(read_csr(mstatus) & MSTATUS_MIE);
}

/* assumes stack_top will be 8 aligned */

os_stack_t *
os_arch_task_stack_init(struct os_task *t, os_stack_t *stack_top, int size)
{
    struct context_switch_frame *sf;
    uint32_t *reg;

    /* Get stack frame pointer */
    sf = (struct context_switch_frame *) ((uint8_t *) stack_top - sizeof(*sf));
    reg = &sf->a7;

    /* Zero out registers except PC which will be set */
    while (reg != &sf->pc) {
        *reg-- = 0;
    }

    /* Set remaining portions of stack frame */
    sf->pc = (uint32_t) t->t_func;
    sf->a0 = (uint32_t) t->t_arg;

    return (os_stack_t *) sf;
}

void
os_arch_init(void)
{
    os_init_idle_task();
}

os_error_t
os_arch_os_init(void)
{
    os_error_t err = OS_OK;
#if 0
// Code from arm to remain what this section is for

    int i;

    /* Cannot be called within an ISR */
    err = OS_ERR_IN_ISR;
    if (__get_IPSR() == 0) {
        err = OS_OK;

        /* Drop priority for all interrupts */
        for (i = 0; i < sizeof(NVIC->IP); i++) {
            NVIC->IP[i] = -1;
        }

        NVIC_SetVector(SVCall_IRQn, (uint32_t)SVC_Handler);
        NVIC_SetVector(PendSV_IRQn, (uint32_t)PendSV_Handler);
        NVIC_SetVector(SysTick_IRQn, (uint32_t)SysTick_Handler);

        /*
         * Install default interrupt handler, which'll print out system
         * state at the time of the interrupt, and few other regs which
         * should help in trying to figure out what went wrong.
         */
        NVIC_SetVector(NonMaskableInt_IRQn, (uint32_t)os_default_irq_asm);
        NVIC_SetVector(HardFault_IRQn, (uint32_t)os_default_irq_asm);
        NVIC_SetVector(-13, (uint32_t)os_default_irq_asm); /* Hardfault */
        for (i = 0; i < NVIC_NUM_VECTORS - NVIC_USER_IRQ_OFFSET; i++) {
            NVIC_SetVector(i, (uint32_t)os_default_irq_asm);
        }

        /* Set the PendSV interrupt exception priority to the lowest priority */
        NVIC_SetPriority(PendSV_IRQn, PEND_SV_PRIO);

        /* Set the SVC interrupt to priority 0 (highest configurable) */
        NVIC_SetPriority(SVCall_IRQn, SVC_PRIO);

        /* Check if privileged or not */
        if ((__get_CONTROL() & 1) == 0) {
            os_arch_init();
        } else {
            svc_os_arch_init();
        }
    }
#endif
    os_arch_init();

    return err;
}

uint32_t
os_arch_start(void)
{
    os_sr_t sr;
    struct os_task *t;
    struct os_task fake_task;

    /* Get the highest priority ready to run to set the current task */
    t = os_sched_next_task();
    /*
     * First time setup fake os_task struct that only has one pointer for SP
     * Having that will make context switch function work same for first
     * and every other time.
     * This fake SP will be used during initial context switch to store SP
     * that will never be used.
     */
    os_sched_set_current_task(&fake_task);

    OS_ENTER_CRITICAL(sr);
    /* Clean software interrupt, and enable it */
    CLINT_REG(CLINT_MSIP) = 0;
    set_csr(mie, MIP_MSIP);

    /* Intitialize and start system clock timer */
    os_tick_init(OS_TICKS_PER_SEC, OS_TICK_PRIO);

    /* Mark the OS as started, right before we run our first task */
    g_os_started = 1;

    /* Perform context switch */
    os_arch_ctx_sw(t);

    OS_EXIT_CRITICAL(sr);

    /* This should not be reached */
    return (uint32_t) (t->t_arg);
}

os_error_t
os_arch_os_start(void)
{
    os_error_t err;

    err = OS_ERR_IN_ISR;
    if (os_in_isr() == 0) {
        err = OS_OK;
        /* should be in kernel mode here */
        os_arch_start();
    }

    return err;
}

void
external_interrupt_handler(uintptr_t mcause)
{
    // TODO: PLIC code here
}

void
software_interrupt_handler(uintptr_t mcause)
{
}