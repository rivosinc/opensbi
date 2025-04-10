/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Intel Corporation. All rights reserved.
 */

#include <sbi/sbi_ecall.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_mpxy.h>
#include <libfdt.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/mpxy/fdt_mpxy.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_sse.h>
#include <sbi_utils/mailbox/rpmi_msgprot.h>

#define RISCV_MSG_ID_SMM_VERSION		0x1
#define RISCV_MSG_ID_SMM_COMMUNICATE	0x4
#define RISCV_MSG_ID_SMM_EVENT_COMPLETE 0x3
#define RISCV_MSG_SMM_MAX_LEN	64

#define SMM_VERSION_MAJOR        1
#define SMM_VERSION_MAJOR_SHIFT  16
#define SMM_VERSION_MAJOR_MASK   0x7FFF
#define SMM_VERSION_MINOR        0
#define SMM_VERSION_MINOR_SHIFT  0
#define SMM_VERSION_MINOR_MASK   0xFFFF
#define SMM_VERSION_FORM(major, minor) ((major << SMM_VERSION_MAJOR_SHIFT) | \
                                       (minor))
#define SMM_VERSION_COMPILED     SMM_VERSION_FORM(SMM_VERSION_MAJOR, \
                                                SMM_VERSION_MINOR)

// RPMI Messages
#define RPMI_REQFWD_ENABLE_NOTIFICATION       0x1
#define RPMI_REQFWD_RETRIEVE_CURRENT_MESSAGE  0x2
#define RPMI_REQFWD_COMPLETE_CURRENT_MESSAGE  0x3

struct mm_cpu_info {
	u64 mpidr;
	u32 linear_id;
	u32 flags;
};

struct mm_boot_info {
	u64 mm_mem_base;
	u64 mm_mem_limit;
	u64 mm_image_base;
	u64 mm_stack_base;
	u64 mm_heap_base;
	u64 mm_ns_comm_buf_base;
	u64 mm_image_size;
	u64 mm_pcpu_stack_size;
	u64 mm_heap_size;
	u64 mm_ns_comm_buf_size;
	u32 num_mem_region;
	u32 num_cpus;
	u32 mm_channel_id;
	struct mm_cpu_info *cpu_info;
};

struct mm_boot_args {
	struct mm_boot_info boot_info;
	struct mm_cpu_info cpu_info[SBI_HARTMASK_MAX_BITS];
};

static u32 mm_channel_id = 0;
static struct sbi_domain *tdomain = NULL;

static struct sbi_domain *__get_domain(char* name)
{
	// int i;
	struct sbi_domain *dom = NULL;
	sbi_domain_for_each(dom)
	{
		if (!sbi_strcmp(dom->name, name)) {
			return dom;
		}
	}
	return NULL;
}

static int mpxy_mm_setup_bootinfo(const void *fdt, int nodeoff, const struct fdt_match *match)
{
	const u32 *prop_instance, *prop_value;
	u64 base64, size64;
	char name[64];
	int i, len, offset;

	struct mm_boot_args *boot_args = NULL;
	struct mm_boot_info *boot_info = NULL;

	prop_instance = fdt_getprop(fdt, nodeoff, "tdomain-instance", &len);
	if (!prop_instance || len < 4) {
		return SBI_EINVAL;
	}
	offset = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(*prop_instance));
	if (offset < 0) {
		return SBI_EINVAL;
	}
	sbi_memset(name, 0, 64);
	strncpy(name, fdt_get_name(fdt, offset, NULL), sizeof(name));
	tdomain = __get_domain(name);
	if (NULL == tdomain)
		return SBI_EINVAL;

	boot_args = (void *)tdomain->next_arg1;
	boot_info = &boot_args->boot_info;

	prop_value = fdt_getprop(fdt, nodeoff, "num-regions", &len);
	if (!prop_value || len < 4)
		return SBI_EINVAL;
	boot_info->num_mem_region = (unsigned int)fdt32_to_cpu(*prop_value);

	prop_value = fdt_getprop(fdt, nodeoff, "memory-reg", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_mem_base	= base64;
	boot_info->mm_mem_limit	= base64 + size64;

	prop_value = fdt_getprop(fdt, nodeoff, "image-reg", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_image_base	= base64;
	boot_info->mm_image_size	= size64;

	prop_value = fdt_getprop(fdt, nodeoff, "heap-reg", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_heap_base	= base64;
	boot_info->mm_heap_size	= size64;

	prop_value = fdt_getprop(fdt, nodeoff, "stack-reg", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_stack_base	= base64 + size64 -1;

	prop_value = fdt_getprop(fdt, nodeoff, "pcpu-stack-size", &len);
	if (!prop_value || len < 4)
		return SBI_EINVAL;
	boot_info->mm_pcpu_stack_size = (unsigned long)fdt32_to_cpu(*prop_value);

	prop_value = fdt_getprop(fdt, nodeoff, "ns-comm-buf", &len);
	if (!prop_value || len < 16)
		return SBI_EINVAL;
	base64 = fdt32_to_cpu(prop_value[0]);
	base64 = (base64 << 32) | fdt32_to_cpu(prop_value[1]);
	size64 = fdt32_to_cpu(prop_value[2]);
	size64 = (size64 << 32) | fdt32_to_cpu(prop_value[3]);
	boot_info->mm_ns_comm_buf_base	= base64;
	boot_info->mm_ns_comm_buf_size	= size64;
	boot_info->num_cpus = 0;
	sbi_hartmask_for_each_hartindex(i, tdomain->possible_harts) {
		boot_args->cpu_info[i].linear_id = sbi_hartindex_to_hartid(i);
		boot_args->cpu_info[i].flags = 0;
		boot_info->num_cpus += 1;
	}
	boot_info->cpu_info = boot_args->cpu_info;

	prop_value = fdt_getprop(fdt, nodeoff, "riscv,sbi-mpxy-channel-id", &len);
	if (!prop_value || len < 4)
		return SBI_EINVAL;
	mm_channel_id = (unsigned int)fdt32_to_cpu(*prop_value);
	boot_info->mm_channel_id = mm_channel_id;

	return 0;
}

#include <sbi/sbi_fifo.h>
struct mm_msg_comm {
	void* msgbuf;
	u32 msg_len;
	void* respbuf;
	u32 resp_len;
	unsigned long *ack_len;
};

struct mm_get_attributes {
	int status;
	u32 mm_version;
	u32 mm_shmem_addr_low;
	u32 mm_shmem_addr_high;
	u32 mm_shmem_size;
};

#define MM_MSG_BUFFER_SIZE       8
static struct mm_msg_comm mm_msg_buffer[MM_MSG_BUFFER_SIZE] = { 0 };
static SBI_FIFO_DEFINE(mm_msg_fifo, mm_msg_buffer, \
                       MM_MSG_BUFFER_SIZE, sizeof(struct mm_msg_comm));
static struct mm_msg_comm current_msg;

static int mpxy_mm_send_message(struct sbi_mpxy_channel *channel,
				  u32 msg_id, void *msgbuf, u32 msg_len,
			    void *respbuf, u32 resp_max_len,
			    unsigned long *ack_len)
{
	if (RISCV_MSG_ID_SMM_VERSION == msg_id) {
		struct mm_get_attributes attr;
		attr.status	 = 0;
		attr.mm_version = SMM_VERSION_COMPILED;
		attr.mm_shmem_addr_low = 0xFFE00000;
		attr.mm_shmem_size     = 0x200000;
		sbi_memcpy((void *)respbuf, &attr,
			   sizeof(struct mm_get_attributes));
		// offset += sizeof(status);
		// sbi_memcpy((void *)respbuf, &version, sizeof(version));
		// 			offset += sizeof(version);
		if (ack_len)
			*ack_len = sizeof(struct mm_get_attributes);
	} else if (RPMI_REQFWD_RETRIEVE_CURRENT_MESSAGE == msg_id) {
		sbi_fifo_dequeue(&mm_msg_fifo, &current_msg);
		sbi_memcpy(respbuf, current_msg.msgbuf, current_msg.msg_len);
		*ack_len = current_msg.msg_len;
	} else if (RPMI_REQFWD_COMPLETE_CURRENT_MESSAGE == msg_id) {
		if(current_msg.respbuf != NULL) {
			sbi_memcpy(current_msg.respbuf, msgbuf, msg_len);
			current_msg.resp_len = msg_len;
			*ack_len = current_msg.msg_len;
		}
		sbi_domain_context_exit();

	} else if (RISCV_MSG_ID_SMM_COMMUNICATE == msg_id) {
		struct mm_msg_comm msg;
		msg.msgbuf = msgbuf;
		msg.msg_len = msg_len;
		msg.respbuf = respbuf;
		msg.ack_len = ack_len;
		msg.resp_len = 0;
		sbi_fifo_enqueue(&mm_msg_fifo, &msg, true);
		sbi_domain_context_enter(tdomain);
	} else {
		return SBI_EFAIL;
	}

	return SBI_OK;
}

static int mpxy_mm_init(const void *fdt, int nodeoff,
			  const struct fdt_match *match)
{
	int rc;
	struct sbi_mpxy_channel *channel;

	/* Allocate context for MPXY channel */
	channel = sbi_zalloc(sizeof(struct sbi_mpxy_channel));
	if (!channel)
		return SBI_ENOMEM;

	/* Setup MM boot envrionment */
	rc = mpxy_mm_setup_bootinfo(fdt, nodeoff, match);
	if (rc) {
		sbi_free(channel);
		return 0;
	}

	channel->channel_id = mm_channel_id;
	channel->send_message_with_response = mpxy_mm_send_message;
	channel->attrs.msg_data_maxlen = 4096;
	rc = sbi_mpxy_register_channel(channel);
	if (rc) {
		sbi_free(channel);
		return rc;
	}

	return 0;
}

static const struct fdt_match mpxy_mm_match[] = {
	{ .compatible = "riscv,sbi-mpxy-mm", .data = NULL },
	{},
};

struct fdt_driver fdt_mpxy_mm = {
	.match_table = mpxy_mm_match,
	.init = mpxy_mm_init,
};
