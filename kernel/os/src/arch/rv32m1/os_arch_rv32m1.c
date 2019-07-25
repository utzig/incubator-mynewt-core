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

#include "os/mynewt.h"
#include "os_priv.h"

#include "fsl_common.h"
#include "fsl_clock.h"

#include <hal/hal_bsp.h>
#include <hal/hal_os_tick.h>

extern void trap_entry();

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

/* XXX: determine how to deal with running un-privileged */
/* only priv currently supported */
uint32_t os_flags = OS_RUN_PRIV;

extern struct os_task g_idle_task;

#define OS_TICK_PRIO 0

static int
os_in_isr(void)
{
    return (int)SystemInISR();
}

void
timer_handler(void)
{
    os_time_advance(1);
}

void SysTick_Handler(void)
{
    timer_handler();
    SystemClearSystickFlag();
}

void
os_arch_ctx_sw(struct os_task *t)
{
    if ((os_sched_get_current_task() != 0) && (t != 0)) {
        os_sched_ctx_sw_hook(t);
    }

    __asm__ volatile ("ecall");
}

os_sr_t
os_arch_save_sr(void)
{
    uint32_t mstatus;

    asm volatile ("csrrci %0, mstatus, 8" : "=r"(mstatus));

    return (mstatus & 8);
}

void
os_arch_restore_sr(os_sr_t isr_ctx)
{
    if (!isr_ctx) {
        asm("csrsi mstatus, 8");
    }
}

int
os_arch_in_critical(void)
{
    uint32_t mstatus;

    asm volatile ("csrr %0, mstatus" : "=r"(mstatus));

    return !(mstatus & 8);
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
    int i;

    /* Disable all interrupts */
    for (i = 0; i < NUMBER_OF_INT_VECTORS; i++) {
        (void)DisableIRQ(i);
    }

    /* Enable interrupts at 0 level */
    //PLIC_REG(PLIC_THRESHOLD_OFFSET) = 0;

    /* Set main trap handler */
    //FIXME write_csr(mtvec, &trap_entry);
    //__ASM volatile("csrw 0x305, %0" :: "r"((uint32_t)&trap_entry));

    os_arch_init();

    return err;
}

uint32_t
os_arch_start(void)
{
    struct os_task *t;
    struct os_task fake_task;

    /* Get the highest priority ready to run to set the current task */
    t = os_sched_next_task();

    /* Initialize and start system clock timer, this enable timer interrupt */
    os_tick_init(OS_TICKS_PER_SEC, OS_TICK_PRIO);

    /* Mark the OS as started, right before we run our first task */
    g_os_started = 1;

    os_sched_set_current_task(&fake_task);

    /* Enable interrupts */
    asm("csrsi mstatus, 8");

    /* Perform context switch */
    os_arch_ctx_sw(t);

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
software_interrupt_handler(uintptr_t mcause)
{
}
