/*
 * Copyright (c) 2016 Jean-Paul Etienne <fractalclone@gmail.com>
 * Copyright (c) 2020 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <ksched.h>
#include <arch/riscv/csr.h>
#include <stdio.h>
#include <core_pmp.h>

#ifdef CONFIG_USERSPACE

/*
 * Glogal variable used to know the current mode running.
 * Is not boolean because it must match the PMP granularity of the arch.
 */
ulong_t is_user_mode;
bool irq_flag;
#endif

void z_thread_entry_wrapper(k_thread_entry_t thread,
			    void *arg1,
			    void *arg2,
			    void *arg3);

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
		     size_t stack_size, k_thread_entry_t thread_func,
		     void *arg1, void *arg2, void *arg3,
		     int priority, unsigned int options)
{
	char *stack_memory = Z_THREAD_STACK_BUFFER(stack);
	ulong_t top_of_stack = (ulong_t)stack_memory + stack_size;
	struct __esf *stack_init;

#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
	const struct soc_esf soc_esf_init = {SOC_ESF_INIT};
#endif

#ifdef CONFIG_THREAD_USERSPACE_LOCAL_DATA
	/* Reserve space on top of stack for local data. */
	ulong_t p_local_data = Z_STACK_PTR_ALIGN(stack_memory + stack_size
		- sizeof(*thread->userspace_local_data));

	thread->userspace_local_data =
		(struct _thread_userspace_local_data *)(p_local_data);

	/* Top of user stack must be moved below the user local data. */
	top_of_stack = p_local_data;
#endif /* CONFIG_THREAD_USERSPACE_LOCAL_DATA */

	z_new_thread_init(thread, stack_memory, stack_size);

	/* Initial stack frame for thread */
	stack_init = (struct __esf *)
		     Z_STACK_PTR_ALIGN(top_of_stack - sizeof(struct __esf));

	/* Setup the initial stack frame */
	stack_init->a0 = (ulong_t)thread_func;
	stack_init->a1 = (ulong_t)arg1;
	stack_init->a2 = (ulong_t)arg2;
	stack_init->a3 = (ulong_t)arg3;
	/*
	 * Following the RISC-V architecture,
	 * the MSTATUS register (used to globally enable/disable interrupt),
	 * as well as the MEPC register (used to by the core to save the
	 * value of the program counter at which an interrupt/exception occcurs)
	 * need to be saved on the stack, upon an interrupt/exception
	 * and restored prior to returning from the interrupt/exception.
	 * This shall allow to handle nested interrupts.
	 *
	 * Given that context switching is performed via a system call exception
	 * within the RISCV architecture implementation, initially set:
	 * 1) MSTATUS to MSTATUS_DEF_RESTORE in the thread stack to enable
	 *    interrupts when the newly created thread will be scheduled;
	 * 2) MEPC to the address of the z_thread_entry_wrapper in the thread
	 *    stack.
	 * Hence, when going out of an interrupt/exception/context-switch,
	 * after scheduling the newly created thread:
	 * 1) interrupts will be enabled, as the MSTATUS register will be
	 *    restored following the MSTATUS value set within the thread stack;
	 * 2) the core will jump to z_thread_entry_wrapper, as the program
	 *    counter will be restored following the MEPC value set within the
	 *    thread stack.
	 */
	stack_init->mstatus = MSTATUS_DEF_RESTORE;

#if defined(CONFIG_PMP_STACK_GUARD) || defined(CONFIG_USERSPACE)
	pmp_init_thread(thread);
#endif /* CONFIG_PMP_STACK_GUARD || CONFIG_USERSPACE */

#if defined(CONFIG_PMP_STACK_GUARD)
	if ((options & K_USER) == 0) {
		/* Enable pmp for machine mode if thread isn't a user*/
		stack_init->mstatus |= MSTATUS_MPRV;
	}
#endif /* CONFIG_PMP_STACK_GUARD */

#if defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
	if ((thread->base.user_options & K_FP_REGS) != 0) {
		stack_init->mstatus |= MSTATUS_FS_INIT;
	}
	stack_init->fp_state = 0;
#endif

	stack_init->mepc = (ulong_t)z_thread_entry_wrapper;

#if defined(CONFIG_USERSPACE)
	thread->arch.priv_stack_start = 0;
	thread->arch.user_sp = 0;
	if ((options & K_USER) != 0) {
		stack_init->mepc = (ulong_t)k_thread_user_mode_enter;
	} else {
		stack_init->mepc = (ulong_t)z_thread_entry_wrapper;
#if defined(CONFIG_PMP_STACK_GUARD)
		init_stack_guard(thread);
#endif /* CONFIG_PMP_STACK_GUARD */
	}
#else
	stack_init->mepc = (ulong_t)z_thread_entry_wrapper;
#if defined(CONFIG_PMP_STACK_GUARD)
	init_stack_guard(thread);
#endif /* CONFIG_PMP_STACK_GUARD */
#endif /* CONFIG_USERSPACE */

#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
	stack_init->soc_context = soc_esf_init;
#endif

	thread->callee_saved.sp = (ulong_t)stack_init;
}

#if defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
int arch_float_disable(struct k_thread *thread)
{
	unsigned int key;

	if (thread != _current) {
		return -EINVAL;
	}

	if (arch_is_in_isr()) {
		return -EINVAL;
	}

	/* Ensure a preemptive context switch does not occur */
	key = irq_lock();

	/* Disable all floating point capabilities for the thread */
	thread->base.user_options &= ~K_FP_REGS;

	/* Clear the FS bits to disable the FPU. */
	__asm__ volatile (
		"mv t0, %0\n"
		"csrrc x0, mstatus, t0\n"
		:
		: "r" (MSTATUS_FS_MASK)
		);

	irq_unlock(key);

	return 0;
}


int arch_float_enable(struct k_thread *thread)
{
	unsigned int key;

	if (thread != _current) {
		return -EINVAL;
	}

	if (arch_is_in_isr()) {
		return -EINVAL;
	}

	/* Ensure a preemptive context switch does not occur */
	key = irq_lock();

	/* Enable all floating point capabilities for the thread. */
	thread->base.user_options |= K_FP_REGS;

	/* Set the FS bits to Initial to enable the FPU. */
	__asm__ volatile (
		"mv t0, %0\n"
		"csrrs x0, mstatus, t0\n"
		:
		: "r" (MSTATUS_FS_INIT)
		);

	irq_unlock(key);

	return 0;
}
#endif /* CONFIG_FPU && CONFIG_FPU_SHARING */

#ifdef CONFIG_USERSPACE

/* Function used by Zephyr to switch a supervisor thread to a user thread */
FUNC_NORETURN void arch_user_mode_enter(k_thread_entry_t user_entry,
	void *p1, void *p2, void *p3)
{
	arch_syscall_invoke5((uintptr_t) arch_user_mode_enter,
		(uintptr_t) user_entry,
		(uintptr_t) p1,
		(uintptr_t) p2,
		(uintptr_t) p3,
		FORCE_SYSCALL_ID);

	CODE_UNREACHABLE;
}

/*
 * User space entry function
 *
 * This function is the entry point to user mode from privileged execution.
 * The conversion is one way, and threads which transition to user mode do
 * not transition back later, unless they are doing system calls.
 */
FUNC_NORETURN void arch_user_mode_enter_syscall(k_thread_entry_t user_entry,
	void *p1, void *p2, void *p3)
{
	ulong_t top_of_user_stack = 0U;
	uintptr_t status;

	/* Set up privileged stack */
	_current->arch.priv_stack_start = (ulong_t)_current->stack_obj;

	_current->stack_info.start =
			(ulong_t)(_current->arch.priv_stack_start +
				CONFIG_PRIVILEGED_STACK_SIZE);

	_current->stack_info.size -= CONFIG_PRIVILEGED_STACK_SIZE;

#ifdef CONFIG_THREAD_USERSPACE_LOCAL_DATA
	top_of_user_stack = Z_STACK_PTR_ALIGN(
				_current->stack_info.start +
				_current->stack_info.size -
				sizeof(_current->userspace_local_data));
#else
	top_of_user_stack = Z_STACK_PTR_ALIGN(
				_current->stack_info.start +
				_current->stack_info.size);
#endif /* CONFIG_THREAD_USERSPACE_LOCAL_DATA */

	/* Set next CPU status to user mode */
	status = csr_read(mstatus);
	status = INSERT_FIELD(status, MSTATUS_MPP, PRV_U);
	status = INSERT_FIELD(status, MSTATUS_MPRV, 0);

	csr_write(mstatus, status);
	csr_write(mepc, z_thread_entry_wrapper);

	/* Set up Physical Memory Protection */
#if defined(CONFIG_PMP_STACK_GUARD)
	init_stack_guard(_current);
#endif

	init_user_accesses(_current);
	configure_user_allowed_stack(_current);

	is_user_mode = true;

	__asm__ volatile ("mv sp, %1"
			  : "=r" (top_of_user_stack)
			  : "r" (top_of_user_stack)
			  : "memory");

	__asm__ volatile ("mv a0, %1"
			  : "=r" (user_entry)
			  : "r" (user_entry)
			  : "memory");

	__asm__ volatile ("mv a1, %1"
			  : "=r" (p1)
			  : "r" (p1)
			  : "memory");

	__asm__ volatile ("mv a2, %1"
			  : "=r" (p2)
			  : "r" (p2)
			  : "memory");

	__asm__ volatile ("mv a3, %1"
			  : "=r" (p3)
			  : "r" (p3)
			  : "memory");

	__asm__ volatile ("mret");

	CODE_UNREACHABLE;
}

#endif /* CONFIG_USERSPACE */
