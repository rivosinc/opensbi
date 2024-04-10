/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Rivos Inc.
 *
 * Authors:
 *   Clément Léger <cleger@rivosinc.com>
 */

#include <sbi/sbi_bitmap.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_types.h>

#include <sbi/riscv_asm.h>
#include <sbi/riscv_encoding.h>

/** Offset of pointer to FWFT HART state in scratch space */
static unsigned long fwft_ptr_offset;

#define fwft_get_hart_state_ptr(__scratch)				\
	sbi_scratch_read_type((__scratch), void *, fwft_ptr_offset)

#define fwft_thishart_state_ptr()					\
	fwft_get_hart_state_ptr(sbi_scratch_thishart_ptr())

#define fwft_set_hart_state_ptr(__scratch, __phs)			\
	sbi_scratch_write_type((__scratch), void *, fwft_ptr_offset, (__phs))

#define MIS_DELEG (1UL << CAUSE_MISALIGNED_LOAD | 1UL << CAUSE_MISALIGNED_STORE)

#define SUPPORTED_FEATURE_COUNT	2

struct fwft_config;

struct fwft_feature {
	enum sbi_fwft_feature_t id;
	int (*supported)(struct fwft_config *conf);
	int (*set)(struct fwft_config *conf, unsigned long value);
	int (*get)(struct fwft_config *conf, unsigned long *value);
};

struct fwft_config {
	const struct fwft_feature *feature;
	unsigned long flags;
};

struct fwft_hart_state {
	struct fwft_config configs[SUPPORTED_FEATURE_COUNT];
};

static int sbi_misaligned_delegation_supported(struct fwft_config *conf)
{
	if (!misa_extension('S'))
		return SBI_ENOTSUPP;

	return SBI_OK;
}

static int sbi_set_misaligned_delegation(struct fwft_config *conf,
					 unsigned long value)
{
	if (value)
		csr_set(CSR_MEDELEG, MIS_DELEG);
	else
		csr_clear(CSR_MEDELEG, MIS_DELEG);

	return SBI_OK;
}

static int sbi_get_misaligned_delegation(struct fwft_config *conf,
					 unsigned long *value)
{
	*value = (csr_read(CSR_MEDELEG) & MIS_DELEG) != 0;

	return SBI_OK;
}

static int sbi_useed_supported(struct fwft_config *conf)
{
	if (!sbi_hart_has_extension(sbi_scratch_thishart_ptr(),
				    SBI_HART_EXT_ZKR))
		return SBI_ENOTSUPP;

	return SBI_OK;
}

static int sbi_set_useed(struct fwft_config *conf, unsigned long value)
{
	unsigned long mseccfg;

	if (value) {
		/* useed might be hardwired, test it */

		csr_set(CSR_MSECCFG, MSECCFG_USEED);
		mseccfg = csr_read(CSR_MSECCFG);
		if ((mseccfg & MSECCFG_USEED) == 0)
			return SBI_ENOTSUPP;

	} else {
		csr_clear(CSR_MSECCFG, MSECCFG_USEED);
	}

	return SBI_OK;
}

static int sbi_get_useed(struct fwft_config *conf, unsigned long *value)
{
	*value = (csr_read(CSR_MSECCFG) & MSECCFG_USEED) != 0;

	return SBI_OK;
}

static struct fwft_config* get_feature_config(enum sbi_fwft_feature_t feature)
{
	int i;
	struct fwft_hart_state *fhs = fwft_thishart_state_ptr();

	if (feature & SBI_FWFT_GLOBAL_FEATURE_BIT)
		return NULL;

	for (i = 0; i < array_size(fhs->configs); i++){
		if (feature == fhs->configs[i].feature->id)
			return &fhs->configs[i];
	}

	return NULL;
}

int sbi_fwft_set(enum sbi_fwft_feature_t feature, unsigned long value,
		 unsigned long flags)
{
	int ret;
	struct fwft_config* conf;

	conf = get_feature_config(feature);
	if (!conf)
		return SBI_EDENIED;

	if (conf->feature->supported) {
		ret = conf->feature->supported(conf);
		if (ret)
			return ret;
	}

	if ((flags & ~SBI_FWFT_SET_FLAG_LOCK) != 0)
		return SBI_ERR_INVALID_PARAM;

	if (conf->flags & SBI_FWFT_SET_FLAG_LOCK)
		return SBI_EDENIED;

	ret = conf->feature->set(conf, value);
	if (ret)
		return ret;

	conf->flags = flags;

	return SBI_OK;
}

int sbi_fwft_get(enum sbi_fwft_feature_t feature, unsigned long *out_val)
{
	struct fwft_config* conf;

	conf = get_feature_config(feature);
	if (!conf)
		return SBI_EDENIED;

	return conf->feature->get(conf, out_val);
}

static const struct fwft_feature features[] =
{
	{
		.id = SBI_FWFT_MISALIGNED_DELEG,
		.supported = sbi_misaligned_delegation_supported,
		.set = sbi_set_misaligned_delegation,
		.get = sbi_get_misaligned_delegation,
	},
	{
		.id = SBI_FWFT_USEED,
		.supported = sbi_useed_supported,
		.set = sbi_set_useed,
		.get = sbi_get_useed,
	},
};

_Static_assert(
	array_size(features) == SUPPORTED_FEATURE_COUNT,
	"features[] array must be the same size as SUPPORTED_FEATURE_COUNT"
	);

int sbi_fwft_init(struct sbi_scratch *scratch, bool cold_boot)
{
	int i;
	struct fwft_hart_state *fhs;

	if (cold_boot) {
		fwft_ptr_offset = sbi_scratch_alloc_type_offset(void *);
		if (!fwft_ptr_offset)
			return SBI_ENOMEM;
	}

	fhs = fwft_get_hart_state_ptr(scratch);
	if (!fhs) {
		fhs = sbi_zalloc(sizeof(*fhs));
		if (!fhs)
			return SBI_ENOMEM;
		for (i = 0; i < array_size(features); i++)
			fhs->configs->feature = &features[i];

		fwft_set_hart_state_ptr(scratch, fhs);
	}

	return 0;
}
