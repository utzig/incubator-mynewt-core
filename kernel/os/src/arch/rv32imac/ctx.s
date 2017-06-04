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

#include <mcu/encoding.h>
#include <mcu/platform.h>
#include <bits.h>

// This is just for now, offsets of registers on stack can be
// computetd probably

    sp_offset = 0
//#define MSTATUS_MPP
    ra_offset = 0x00
    gp_offset = 0x04
    tp_offset = 0x08
    t0_offset = 0x0C
    t1_offset = 0x10 
    t2_offset = 0x14
    t3_offset = 0x18
    t4_offset = 0x1C
    t5_offset = 0x20
    t6_offset = 0x24
    a0_offset = 0x28
    a1_offset = 0x2C
    a2_offset = 0x30
    a3_offset = 0x34
    a4_offset = 0x38
    a5_offset = 0x3C
    a6_offset = 0x40
    a7_offset = 0x44
    caller_saved_size = (a7_offset - ra_offset + 4)

    mepc_offset = 0x00
    s0_offset   = 0x04
    s1_offset   = 0x08
    s2_offset   = 0x0C
    s3_offset   = 0x10
    s4_offset   = 0x14
    s5_offset   = 0x18
    s6_offset   = 0x1C
    s7_offset   = 0x20
    s8_offset   = 0x24
    s9_offset   = 0x28
    s10_offset  = 0x2C
    s11_offset  = 0x30
    callee_saved_size = (s11_offset - mepc_offset + 4)

    .section      .text.trap_entry	
    .align 2
    .global trap_entry
trap_entry:
    addi sp, sp, -caller_saved_size
    sw ra, ra_offset(sp)
    sw gp, gp_offset(sp)
    sw tp, tp_offset(sp)
    sw t0, t0_offset(sp)
    sw t1, t1_offset(sp)
    sw t2, t2_offset(sp)
    sw t3, t3_offset(sp)
    sw t4, t4_offset(sp)
    sw t5, t5_offset(sp)
    sw t6, t6_offset(sp)
    sw a0, a0_offset(sp)
    sw a1, a1_offset(sp)
    sw a2, a2_offset(sp)
    sw a3, a3_offset(sp)
    sw a4, a4_offset(sp)
    sw a5, a5_offset(sp)
    sw a6, a6_offset(sp)
    sw a7, a7_offset(sp)

    csrr a0, mcause           /* a0 = Trap cause */
    csrr a1, mepc             /* a1 = Return address */
    bltz a0, async_interrupt  /* highest bit set => interrupt */
#if CONTEXT_SWITCH_ON_ECALL
    li t0, 11                 /* 11 = ECALL */
    beq t0, a0, context_switch /* ECALL => context switch */
#endif

    /* Save all registers for exception handler */
    call save_callee_responisble_registers
    mv a2, sp
    call handle_trap          /* Call handle_trap(cuase, mepc, sp) */
    call restore_callee_responsible_registers

finish_trap_with_return_address:
    csrw mepc, a0             /* Set return addres to what handle_trap returned */
finish_trap:
    lw gp, gp_offset(sp)
    lw tp, tp_offset(sp)
    lw t3, t3_offset(sp)
    lw t4, t4_offset(sp)
    lw t5, t5_offset(sp)
    lw t6, t6_offset(sp)
    lw a2, a2_offset(sp)
    lw a3, a3_offset(sp)
    lw a4, a4_offset(sp)
    lw a5, a5_offset(sp)
    lw a6, a6_offset(sp)
    lw a7, a7_offset(sp)
    /* In case of context switch that did not found new task to run only
     * fiew registers were actually touched.
     */
fast_finish_contex_switch:
    lw ra, ra_offset(sp)
    lw a0, a0_offset(sp)
    lw a1, a1_offset(sp)
    lw t0, t0_offset(sp)
    lw t1, t1_offset(sp)
    lw t2, t2_offset(sp)
    addi sp, sp, caller_saved_size
    mret

context_switch:
    /* Do context switch only if highest priority task changed */
    lw t2, g_os_run_list     /* Get highest priority task ready to run */
    la t1, g_current_task    /* Get current task address */
    lw t0, (t1)              /* Get current task */
    beq t0, t2, fast_finish_contex_switch  /* No context switch needed */
  
    /* Task needs to be changed, save calle responsible registers */
    call save_callee_responisble_registers

#if CONTEXT_SWITCH_ON_ECALL
    addi a1, a1, 4                /* Get address of instruction after ECALL */
#endif
    sw a1, mepc_offset(sp)

    sw sp, sp_offset(t0)          /* Store SP in os_task */

    lw sp, sp_offset(t2)          /* Switch to SP of highest priority task */

    lw a0, mepc_offset(sp)        /* Restore PC */

    call restore_callee_responsible_registers

    sw t2, (t1)                   /* Set g_current_task */
    j finish_trap_with_return_address

save_callee_responisble_registers:
    addi sp, sp, -callee_saved_size
    sw s0, s0_offset(sp)
    sw s1, s1_offset(sp)
    sw s2, s2_offset(sp)
    sw s3, s3_offset(sp)
    sw s4, s4_offset(sp)
    sw s5, s5_offset(sp)
    sw s6, s6_offset(sp)
    sw s7, s7_offset(sp)
    sw s8, s8_offset(sp)
    sw s9, s9_offset(sp)
    sw s10, s10_offset(sp)
    sw s11, s11_offset(sp)
    ret

restore_callee_responsible_registers:
    lw s0, s0_offset(sp)
    lw s1, s1_offset(sp)
    lw s2, s2_offset(sp)
    lw s3, s3_offset(sp)
    lw s4, s4_offset(sp)
    lw s5, s5_offset(sp)
    lw s6, s6_offset(sp)
    lw s7, s7_offset(sp)
    lw s8, s8_offset(sp)
    lw s9, s9_offset(sp)
    lw s10, s10_offset(sp)
    lw s11, s11_offset(sp)

    /* Change SP to interrupt like */
    addi sp, sp, callee_saved_size
    ret

async_interrupt:
    la ra, finish_trap
    li t0, 0x8000000B         /* External interrupt */
    beq a0, t0, external_interrupt_handler
    li t0, 0x80000007         /* Timer interrupt */
    beq a0, t0, timer_interrupt_handler
    li t0, 0x80000003         /* Software interrupt */
l1:
    bne a0, t0, l1            /* If not software interrupt so what */
    li t0, CLINT_CTRL_ADDR    /* Clear software interrutp */
    sw x0, (t0)
#if CONTEXT_SWITCH_ON_ECALL
    j software_interrupt_handler
#else
    j context_switch
#endif
