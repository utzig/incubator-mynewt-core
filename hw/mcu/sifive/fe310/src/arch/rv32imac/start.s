// See LICENSE for license details.

// See LICENSE for license details.

#include <env/encoding.h>

	.section .init
	.globl _reset_handler
	.type _reset_handler,@function

_reset_handler:
	la gp, _gp
	la sp, _sp

	/* Disable Local/Timer/External interrupts */
	csrr t0, mie
	li t1, ~(MIP_MSIP | MIP_MTIP | MIP_MEIP)
	and t0, t0, t1
	csrw mie, t0
	/* Enable interrupts */
	csrs mstatus, MSTATUS_MIE
	csrr t0, mstatus

	/* Load data section */
	la a0, _data_lma
	la a1, _data
	la a2, _edata
	bgeu a1, a2, 2f
1:
	lw t0, (a0)
	sw t0, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
	bltu a1, a2, 1b
2:

	/* Clear bss section */
	la a0, __bss_start
	la a1, _end
	bgeu a0, a1, 2f
1:
	sw zero, (a0)
	addi a0, a0, 4
	bltu a0, a1, 1b
2:

	la a0, _end
	la a1, _heap_end
	call _sbrkInit

	/* Call global constructors */
	la a0, __libc_fini_array
	call atexit
	call __libc_init_array

#ifndef __riscv_float_abi_soft
	/* Enable FPU */
	li t0, MSTATUS_FS
	csrs mstatus, t0
	csrr t1, mstatus
	and t1, t1, t0
	beqz t1, 1f
	fssr x0
1:
#endif

	call _init
	call _start
	call _fini
	tail exit
