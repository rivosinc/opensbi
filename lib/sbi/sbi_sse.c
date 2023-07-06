/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Rivos Systems Inc.
 *
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_fifo.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_ipi.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_sse.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_trap.h>

#include <sbi/sbi_console.h> /* TODO: REMOVE */

enum sse_event_index {
	SSE_LOCAL_RAS = 0,
	SSE_LOCAL_PMU,
	SSE_LOCAL_ASYNC_PF,
	SSE_LOCAL_DEBUG,
	SSE_MAX_LOCAL_EVENTS,
	SSE_GLOBAL_RAS = SSE_MAX_LOCAL_EVENTS,
	SSE_GLOBAL_DEBUG,
	SSE_MAX_EVENTS,
};

struct sse_ipi_inject_data {
	uint32_t event_id;
};

struct sse_event_param {
	uint32_t prio;
	unsigned int hart;
};

struct sbi_sse_event {
	enum sbi_sse_state state;
	bool pending;
	uint32_t event_id;
	struct sbi_sse_handler_ctx *ctx;
	bool from_ecall;
};

struct sse_hart_state {
	struct sbi_sse_event events[SSE_MAX_LOCAL_EVENTS];
};

static struct sbi_sse_event global_events[SSE_MAX_EVENTS - SSE_MAX_LOCAL_EVENTS];
static struct sse_event_param events_param[SSE_MAX_EVENTS];

static unsigned long sse_inject_fifo_off;
static unsigned long sse_inject_fifo_mem_off;
/* Offset of pointer to SSE HART state in scratch space */
static unsigned long shs_ptr_off;

#define sse_get_hart_state_ptr(__scratch)	\
	sbi_scratch_read_type((__scratch), void *, shs_ptr_off)

#define sse_thishart_state_ptr()		\
	sse_get_hart_state_ptr(sbi_scratch_thishart_ptr())

#define sse_set_hart_state_ptr(__scratch, __sse_state)	\
	sbi_scratch_write_type((__scratch), void *, shs_ptr_off, (__sse_state))

static enum sse_event_index sse_event_idx(uint32_t event)
{
	switch (event) {
	case SBI_SSE_EVENT_LOCAL_RAS:
		return SSE_LOCAL_RAS;
	case SBI_SSE_EVENT_LOCAL_PMU:
		return SSE_LOCAL_PMU;
	case SBI_SSE_EVENT_LOCAL_ASYNC_PF:
		return SSE_LOCAL_ASYNC_PF;
	case SBI_SSE_EVENT_LOCAL_DEBUG:
		return SSE_LOCAL_DEBUG;
	case SBI_SSE_EVENT_GLOBAL_RAS:
		return SSE_GLOBAL_RAS;
	case SBI_SSE_EVENT_GLOBAL_DEBUG:
		return SSE_GLOBAL_DEBUG;
	default:
		return SSE_MAX_EVENTS;
	}
}

static bool sse_event_is_global(uint32_t event)
{
	return event >= SSE_MAX_LOCAL_EVENTS;
}

static void sse_event_set_state(struct sbi_sse_event *e, enum sbi_sse_state state)
{
	enum sbi_sse_state prev_state = e->state;

	e->state = state;
	switch (state) {
		case SSE_STATE_UNUSED:
			if (prev_state == SSE_STATE_REGISTERED)
				return;
		break;
		case SSE_STATE_REGISTERED:
			if (prev_state == SSE_STATE_UNUSED ||
			    prev_state == SSE_STATE_ENABLED)
				return;
		break;
		case SSE_STATE_ENABLED:
			if (prev_state == SSE_STATE_REGISTERED ||
			    prev_state == SSE_STATE_RUNNING)
				return;
		break;
		case SSE_STATE_RUNNING:
			if (prev_state == SSE_STATE_ENABLED)
				return;
		break;
	}

	sbi_panic("Invalid SSE state transition: %d -> %d\n", prev_state,
		  state);
}

static struct sbi_sse_event *sse_event_get_by_idx(uint32_t idx)
{
	struct sse_hart_state *shs;

	if (idx < SSE_MAX_LOCAL_EVENTS) {
		shs = sse_thishart_state_ptr();
		return shs->events + idx;
	} else {
		return &global_events[idx - SSE_MAX_LOCAL_EVENTS];
	}
}

static struct sbi_sse_event *sse_event_get(uint32_t event)
{
	enum sse_event_index idx;

	idx = sse_event_idx(event);
	if (idx >= SSE_MAX_EVENTS)
		return NULL;

	return sse_event_get_by_idx(idx);
}

static int sse_event_get_attr(struct sbi_sse_event *e, uint32_t attr_id,
			      unsigned long *out_val)
{
	int ret;

	switch (attr_id) {
	case SBI_SSE_ATTR_STATE:
		*out_val = e->state;
		ret = 0;
		break;
	case SBI_SSE_ATTR_PRIO:
		*out_val = events_param[e->event_id].prio;
		ret = 0;
		break;
	case SBI_SSE_ATTR_ALLOW_INJECT:
		*out_val = 1;
		ret = 0;
		break;
	case SBI_SSE_ATTR_HART_ID:
		*out_val = current_hartid();
		ret = 0;
		break;
	case SBI_SSE_ATTR_PENDING:
		*out_val = e->pending;
		ret = 0;
		break;
	default:
		ret = SBI_EINVAL;
		break;
	}
	return ret;
}

static int sse_event_set_attr(struct sbi_sse_event *e, uint32_t attr_id,
			      unsigned long val)
{
	uint32_t evt = e->event_id;
	int ret;

	switch (attr_id) {
	case SBI_SSE_ATTR_PENDING:
	case SBI_SSE_ATTR_STATE:
	case SBI_SSE_ATTR_ALLOW_INJECT:
		/* Read-only */
		ret = SBI_EDENIED;
		break;
	case SBI_SSE_ATTR_PRIO:
		events_param[evt].prio = (uint32_t)val;
		ret = 0;
		break;
	case SBI_SSE_ATTR_HART_ID:
		if (!sse_event_is_global(evt)) {
			ret = SBI_EDENIED;
			break;
		}

		if (!sbi_domain_is_assigned_hart(sbi_domain_thishart_ptr(),
						 val)) {
			ret = SBI_ERR_INVALID_PARAM;
			break;
		}

		events_param[evt].hart = (uint32_t)val;
		ret = 0;

		break;
	default:
		ret = SBI_EINVAL;
		break;
	}
	return ret;
}

static int sse_event_register(struct sbi_sse_event *e,
			      struct sbi_sse_handler_ctx *ctx)
{
	if (e->state != SSE_STATE_UNUSED)
		return SBI_EINVALID_STATE;
	sse_event_set_state(e, SSE_STATE_REGISTERED);
	e->ctx = ctx;
	return 0;
}

static int sse_event_unregister(struct sbi_sse_event *e)
{
	if (e->state != SSE_STATE_REGISTERED)
		return SBI_EINVALID_STATE;

	sse_event_set_state(e, SSE_STATE_UNUSED);
	e->ctx = NULL;
	return 0;
}

static int sse_event_inject(struct sbi_sse_event *e, struct sbi_sse_event *prev_e,
		      struct sbi_trap_regs *regs)
{
	ulong prev_smode, prev_virt, sie, sstatus;
	struct sse_interrupted_state *i_ctx = &e->ctx->interrupted;
	struct sse_entry_state *e_ctx = &e->ctx->entry;

	sse_event_set_state(e, SSE_STATE_RUNNING);
	e->pending = false;

	if (prev_e) {
		/* We are injected right after another event, copy previous
		 * event context for correct restoration
		 */
		sbi_memcpy(i_ctx, &prev_e->ctx->interrupted,
		       sizeof(struct sse_interrupted_state));
	} else {
		sbi_memcpy(&i_ctx->ra, &regs->ra, sizeof(unsigned long) * 31);

		sstatus = csr_read(CSR_SSTATUS);
		sie = sstatus & SSTATUS_SIE;
		sstatus &= ~SSTATUS_SIE;
		csr_write(CSR_SSTATUS, sstatus);

		prev_smode = (regs->mstatus & MSTATUS_MPP) >> MSTATUS_MPP_SHIFT;
	#if __riscv_xlen == 32
		prev_virt = (regs->mstatusH & MSTATUSH_MPV) ? 1 : 0;
	#else
		prev_virt = (regs->mstatus & MSTATUS_MPV) ? 1 : 0;
	#endif

		i_ctx->exc_mode = prev_smode << EXC_MODE_PP_SHIFT;
		i_ctx->exc_mode |= prev_virt << EXC_MODE_PV_SHIFT;
		i_ctx->exc_mode |= sie ? EXC_MODE_SSTATUS_SPIE : 0;
		i_ctx->pc = regs->mepc;
		if (e->from_ecall) {
			/* In case of injection from ecall, we need to skip
			 * ecall insn when returning and also return a
			 * meaningful value to ecall
			 */
			i_ctx->pc += 4;
			i_ctx->a0 = SBI_OK;
		}
	}

	sbi_memcpy(&regs->ra, &e_ctx->ra, sizeof(unsigned long) * 31);
	regs->mepc = e_ctx->pc;

	regs->mstatus &= ~MSTATUS_MPP;
	regs->mstatus |= (PRV_S << MSTATUS_MPP_SHIFT);

	return SBI_EJUMP;
}

static int sse_event_complete(struct sbi_sse_event *e,
			      struct sbi_trap_regs *regs)
{
	ulong sstatus, spie, pv;
	struct sse_interrupted_state *i_ctx = &e->ctx->interrupted;

	sbi_memcpy(&regs->ra, &i_ctx->ra, sizeof(unsigned long) * 31);

	sstatus = csr_read(CSR_SSTATUS);
	sstatus &= ~(SSTATUS_SIE | SSTATUS_SPIE);

	spie = sstatus & SSTATUS_SPIE;
	sstatus |= (spie) ? SSTATUS_SIE : 0;

	spie = (i_ctx->exc_mode & EXC_MODE_SSTATUS_SPIE);
	sstatus |= (spie) ? SSTATUS_SPIE : 0;
	csr_write(CSR_SSTATUS, sstatus);

	/* Restore previous virtualization state */
	pv = i_ctx->exc_mode & EXC_MODE_PV;
#if __riscv_xlen == 32
	regs->mstatusH &= ~MSTATUSH_MPV;
	regs->mstatusH |= (pv) ? MSTATUSH_MPV : 0UL;
#else
	regs->mstatus &= ~MSTATUS_MPV;
	regs->mstatus |= (pv) ? MSTATUS_MPV : 0UL;
#endif

	regs->mstatus &= ~MSTATUS_MPP;
	if (i_ctx->exc_mode & EXC_MODE_PP)
		regs->mstatus |= (PRV_S << MSTATUS_MPP_SHIFT);

	regs->mepc = i_ctx->pc;

	return SBI_EJUMP;
}

static int sse_process_pending(struct sbi_sse_event *prev_e,
			       struct sbi_trap_regs *regs)
{
	int i;
	struct sbi_sse_event *e;

	for (i = 0; i < SSE_MAX_EVENTS; i++) {
		e = sse_event_get_by_idx(i);
		if (!e->pending || e->state != SSE_STATE_ENABLED)
			continue;

		return sse_event_inject(e, prev_e, regs);
	}

	return SBI_OK;
}

static void sse_ipi_inject_process(struct sbi_scratch *scratch,
				   struct sbi_trap_regs *regs)
{
	struct sbi_sse_event *e;
	struct sse_ipi_inject_data evt;
	struct sbi_fifo *sse_inject_fifo_r =
			sbi_scratch_offset_ptr(scratch, sse_inject_fifo_off);

	/* This can be the case when sbi_exit() is called */
	if (!regs)
		return;

	/* Mark all queued events as pending */
	while(!sbi_fifo_dequeue(sse_inject_fifo_r, &evt)) {
		e = sse_event_get(evt.event_id);
		if (!e)
			continue;

		e->pending = true;
	}

	sse_process_pending(NULL, regs);
}

static struct sbi_ipi_event_ops sse_ipi_inject_ops = {
	.name = "IPI_SSE_INJECT",
	.process = sse_ipi_inject_process,
};

static u32 sse_ipi_inject_event = SBI_IPI_EVENT_MAX;

static int sse_ipi_inject_trigger(uint32_t event_id, unsigned long hartid)
{
	int ret;
	struct sbi_scratch *remote_scratch = NULL;
	struct sse_ipi_inject_data evt = {event_id};
	struct sbi_fifo *sse_inject_fifo_r;

	remote_scratch = sbi_hartid_to_scratch(hartid);
	if (!remote_scratch)
		return SBI_EINVAL;
	sse_inject_fifo_r = sbi_scratch_offset_ptr(remote_scratch, sse_inject_fifo_off);

	ret = sbi_fifo_enqueue(sse_inject_fifo_r, &evt);
	if (ret)
		return SBI_EFAIL;

	ret = sbi_ipi_send_many(BIT(hartid), 0, sse_ipi_inject_event, NULL);
	if (ret)
		return SBI_EFAIL;

	return SBI_OK;
}

int sbi_sse_complete(uint32_t event_id, uint32_t status, uint32_t flags,
		     struct sbi_trap_regs *regs)
{
	struct sbi_sse_event *e;
	int ret;

	e = sse_event_get(event_id);
	if (!e)
		return SBI_EINVAL;

	if (e->state != SSE_STATE_RUNNING)
		return SBI_EINVALID_STATE;

	if (flags & SBI_SSE_COMPLETE_FLAG_EVENT_DISABLE)
		sse_event_set_state(e, SSE_STATE_REGISTERED);
	else
		sse_event_set_state(e, SSE_STATE_ENABLED);

	ret = sse_process_pending(e, regs);
	if (ret == SBI_EJUMP)
		return ret;

	return sse_event_complete(e, regs);
}

int sbi_sse_enable(uint32_t event_id)
{
	struct sbi_sse_event *e;

	e = sse_event_get(event_id);
	if (!e)
		return SBI_EINVAL;

	if (e->state != SSE_STATE_REGISTERED)
		return SBI_EINVALID_STATE;

	sse_event_set_state(e, SSE_STATE_ENABLED);

	return SBI_OK;
}

int sbi_sse_disable(uint32_t event_id)
{
	struct sbi_sse_event *e;

	e = sse_event_get(event_id);
	if (!e)
		return SBI_EINVAL;

	if (e->state != SSE_STATE_ENABLED)
		return SBI_EINVALID_STATE;

	sse_event_set_state(e, SSE_STATE_REGISTERED);

	return SBI_OK;
}

int sbi_sse_inject(uint32_t event_id, unsigned long hartid,
		   struct sbi_trap_regs *regs, bool from_ecall)
{
	struct sbi_sse_event *e;

	e = sse_event_get(event_id);
	if (!e)
		return SBI_EINVAL;

	if (hartid != current_hartid())
		return sse_ipi_inject_trigger(event_id, hartid);

	if (e->state == SSE_STATE_RUNNING) {
		/* Event will be scheduled later on after another event and
		 * thus, will return after this event completed.
		 */
		e->from_ecall = false;
		e->pending = true;

		return SBI_OK;
	}

	if (e->state != SSE_STATE_ENABLED)
		return SBI_EINVALID_STATE;

	e->from_ecall = from_ecall;
	e->pending = true;

	return sse_process_pending(NULL, regs);
}

int sbi_sse_get_attr(uint32_t event_id, uint32_t attr_id, unsigned long *out_val)
{
	struct sbi_sse_event *e;
	int ret = SBI_EINVAL;

	e = sse_event_get(event_id);
	if (!e)
		return SBI_EINVAL;

	ret = sse_event_get_attr(e, attr_id, out_val);

	return ret;
}

int sbi_sse_set_attr(uint32_t event_id, uint32_t attr_id, unsigned long val)
{
	struct sbi_sse_event *e;
	int ret = SBI_EINVAL;

	e = sse_event_get(event_id);
	if (!e)
		return SBI_EINVAL;

	ret = sse_event_set_attr(e, attr_id, val);

	return ret;
}

int sbi_sse_register(uint32_t event_id,
			unsigned long phys_lo,
			unsigned long phys_hi)
{
	struct sbi_sse_event *e;
	const unsigned align = __riscv_xlen >> 3;
	ulong smode = (csr_read(CSR_MSTATUS) & MSTATUS_MPP) >>
			MSTATUS_MPP_SHIFT;

	if (phys_lo & (align - 1))
		return SBI_EINVALID_ADDR;

	/*
	 * On RV32, the M-mode can only access the first 4GB of
	 * the physical address space because M-mode does not have
	 * MMU to access full 34-bit physical address space.
	 *
	 * Based on above, we simply fail if the upper 32bits of
	 * the physical address (i.e. a2 register) is non-zero on
	 * RV32.
	 */
	if (phys_hi)
		return SBI_EINVALID_ADDR;

	if (!sbi_domain_check_addr_range(sbi_domain_thishart_ptr(),
					 phys_lo,
					 sizeof(struct sbi_sse_handler_ctx),
					 smode,
					 SBI_DOMAIN_READ|SBI_DOMAIN_WRITE))
		return SBI_EINVALID_ADDR;

	e = sse_event_get(event_id);
	if (!e)
		return SBI_EINVAL;

	return sse_event_register(e, (struct sbi_sse_handler_ctx *)phys_lo);
}

int sbi_sse_unregister(uint32_t event_id)
{
	struct sbi_sse_event *e;

	e = sse_event_get(event_id);
	if (!e)
		return SBI_EINVAL;

	return sse_event_unregister(e);
}

int sbi_sse_init(struct sbi_scratch *scratch, bool cold_boot)
{
	int ret;
	void *sse_inject_mem;
	struct sbi_fifo *sse_inject_q;
	struct sse_hart_state *shs;

	if (cold_boot) {
		shs_ptr_off = sbi_scratch_alloc_offset(sizeof(void *));
		if (!shs_ptr_off) {
			return SBI_ENOMEM;
		}
		sse_inject_fifo_off = sbi_scratch_alloc_offset(sizeof(*sse_inject_q));
		if (!sse_inject_fifo_off) {
			sbi_scratch_free_offset(shs_ptr_off);
			return SBI_ENOMEM;
		}
		sse_inject_fifo_mem_off = sbi_scratch_alloc_offset(
				SSE_MAX_EVENTS * sizeof(struct sse_ipi_inject_data));
		if (!sse_inject_fifo_mem_off) {
			sbi_scratch_free_offset(sse_inject_fifo_off);
			sbi_scratch_free_offset(shs_ptr_off);
			return SBI_ENOMEM;
		}

		ret = sbi_ipi_event_create(&sse_ipi_inject_ops);
		if (ret < 0)
			return ret;
		sse_ipi_inject_event = ret;
	}

	shs = sse_get_hart_state_ptr(scratch);
	if (!shs) {
		shs = sbi_zalloc(sizeof(*shs));
		if (!shs)
			return SBI_ENOMEM;

		sse_set_hart_state_ptr(scratch, shs);
	}

	sse_inject_q = sbi_scratch_offset_ptr(scratch, sse_inject_fifo_off);
	sse_inject_mem = sbi_scratch_offset_ptr(scratch, sse_inject_fifo_mem_off);

	sbi_fifo_init(sse_inject_q, sse_inject_mem,
		      SSE_MAX_EVENTS, sizeof(struct sse_ipi_inject_data));

	return 0;
}
