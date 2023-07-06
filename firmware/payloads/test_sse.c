/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_string.h>

struct sse_entry_state {
	/** Entry program counter */
	unsigned long pc;
	/** ra register state */
	unsigned long ra;
	/** sp register state */
	unsigned long sp;
	/** gp register state */
	unsigned long gp;
	/** tp register state */
	unsigned long tp;
	/** t0 register state */
	unsigned long t0;
	/** t1 register state */
	unsigned long t1;
	/** t2 register state */
	unsigned long t2;
	/** s0 register state */
	unsigned long s0;
	/** s1 register state */
	unsigned long s1;
	/** a0 register state */
	unsigned long a0;
	/** a1 register state */
	unsigned long a1;
	/** a2 register state */
	unsigned long a2;
	/** a3 register state */
	unsigned long a3;
	/** a4 register state */
	unsigned long a4;
	/** a5 register state */
	unsigned long a5;
	/** a6 register state */
	unsigned long a6;
	/** a7 register state */
	unsigned long a7;
	/** s2 register state */
	unsigned long s2;
	/** s3 register state */
	unsigned long s3;
	/** s4 register state */
	unsigned long s4;
	/** s5 register state */
	unsigned long s5;
	/** s6 register state */
	unsigned long s6;
	/** s7 register state */
	unsigned long s7;
	/** s8 register state */
	unsigned long s8;
	/** s9 register state */
	unsigned long s9;
	/** s10 register state */
	unsigned long s10;
	/** s11 register state */
	unsigned long s11;
	/** t3 register state */
	unsigned long t3;
	/** t4 register state */
	unsigned long t4;
	/** t5 register state */
	unsigned long t5;
	/** t6 register state */
	unsigned long t6;
} __packed;

struct sse_interrupted_state {
	/** Interrupted program counter */
	unsigned long pc;
	/** ra register state */
	unsigned long ra;
	/** sp register state */
	unsigned long sp;
	/** gp register state */
	unsigned long gp;
	/** tp register state */
	unsigned long tp;
	/** t0 register state */
	unsigned long t0;
	/** t1 register state */
	unsigned long t1;
	/** t2 register state */
	unsigned long t2;
	/** s0 register state */
	unsigned long s0;
	/** s1 register state */
	unsigned long s1;
	/** a0 register state */
	unsigned long a0;
	/** a1 register state */
	unsigned long a1;
	/** a2 register state */
	unsigned long a2;
	/** a3 register state */
	unsigned long a3;
	/** a4 register state */
	unsigned long a4;
	/** a5 register state */
	unsigned long a5;
	/** a6 register state */
	unsigned long a6;
	/** a7 register state */
	unsigned long a7;
	/** s2 register state */
	unsigned long s2;
	/** s3 register state */
	unsigned long s3;
	/** s4 register state */
	unsigned long s4;
	/** s5 register state */
	unsigned long s5;
	/** s6 register state */
	unsigned long s6;
	/** s7 register state */
	unsigned long s7;
	/** s8 register state */
	unsigned long s8;
	/** s9 register state */
	unsigned long s9;
	/** s10 register state */
	unsigned long s10;
	/** s11 register state */
	unsigned long s11;
	/** t3 register state */
	unsigned long t3;
	/** t4 register state */
	unsigned long t4;
	/** t5 register state */
	unsigned long t5;
	/** t6 register state */
	unsigned long t6;
	/** Exception mode */
	unsigned long exc_mode;
};

struct sbi_sse_handler_ctx {
	struct sse_entry_state entry;
	struct sse_interrupted_state interrupted;
};


#define SBI_ECALL_OUTVAL(__eid, __fid, __a0, __a1, __a2, __outval)            \
	({                                                                    \
		register unsigned long a0 asm("a0") = (unsigned long)(__a0);  \
		register unsigned long a1 asm("a1") = (unsigned long)(__a1);  \
		register unsigned long a2 asm("a2") = (unsigned long)(__a2);  \
		register unsigned long a6 asm("a6") = (unsigned long)(__fid); \
		register unsigned long a7 asm("a7") = (unsigned long)(__eid); \
		asm volatile("ecall"                                          \
			     : "+r"(a0)                                       \
			     : "r"(a1), "r"(a2), "r"(a6), "r"(a7)             \
			     : "memory");                                     \
		__outval = a1;                                                \
		a0;                                                           \
	})

void sbi_ecall_console_puts(const char *str);

static u8 sse_stack[2][1024];
static int first_time = 1;

static void sse_test_handler(void *arg)
{
	unsigned long out;
	sbi_ecall_console_puts("Handler invoked !\n");

	if (first_time) {
		first_time = 0;
		SBI_ECALL_OUTVAL(SBI_EXT_SSE, SBI_EXT_SSE_INJECT,
				SBI_SSE_EVENT_LOCAL_RAS, 0, 0, out);
	}

	SBI_ECALL_OUTVAL(SBI_EXT_SSE, SBI_EXT_SSE_COMPLETE,
			       SBI_SSE_EVENT_LOCAL_RAS, 0, 0, out);

	out = out;
}

void test_sse(void)
{
	struct sbi_sse_handler_ctx ctx;
	unsigned long out, ret;

	sbi_memset(&ctx, 0, sizeof(ctx));
	ctx.entry.pc = (unsigned long)sse_test_handler;
	ctx.entry.sp = (unsigned long)sse_stack[0];

	sbi_ecall_console_puts("Starting SSE test\n");

	ret = SBI_ECALL_OUTVAL(SBI_EXT_SSE, SBI_EXT_SSE_REGISTER,
			       SBI_SSE_EVENT_LOCAL_RAS, &ctx, 0, out);
	if (ret) {
		sbi_ecall_console_puts("SSE Register failed\n");
		return;
	}

	ret = SBI_ECALL_OUTVAL(SBI_EXT_SSE, SBI_EXT_SSE_ENABLE,
			       SBI_SSE_EVENT_LOCAL_RAS, 0, 0, out);
	if (ret) {
		sbi_ecall_console_puts("SSE Enable failed\n");
		return;
	}

	ret = SBI_ECALL_OUTVAL(SBI_EXT_SSE, SBI_EXT_SSE_INJECT,
			       SBI_SSE_EVENT_LOCAL_RAS, 0, 0, out);
	if (ret) {
		sbi_ecall_console_puts("SSE Inject failed\n");
		return;
	}

	out = out;

	sbi_ecall_console_puts("Finished SSE test\n");
}