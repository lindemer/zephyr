/*
 * Copyright (c) 2020 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief RISCV public error handling
 *
 * RISCV-specific kernel error handling interface. Included by riscv/arch.h.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RISCV_ERROR_H_
#define ZEPHYR_INCLUDE_ARCH_RISCV_ERROR_H_

#include <arch/riscv/syscall.h>
#include <arch/riscv/exp.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARCH_EXCEPT(reason_p)	do { \
	user_fault(reason_p); \
	} while (false)

__syscall void user_fault(unsigned int reason);

#include <syscalls/error.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_RISCV_ERROR_H_ */
