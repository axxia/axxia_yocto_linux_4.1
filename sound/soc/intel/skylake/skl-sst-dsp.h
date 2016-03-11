/*
 * Skylake SST DSP Support
 *
 * Copyright (C) 2014-15, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __SKL_SST_DSP_H__
#define __SKL_SST_DSP_H__

#include <linux/interrupt.h>
#include <sound/memalloc.h>
#include "skl-sst-cldma.h"

struct sst_dsp;
struct skl_sst;
struct sst_dsp_device;

/* Intel HD Audio General DSP Registers */
#define SKL_ADSP_GEN_BASE		0x0
#define SKL_ADSP_REG_ADSPCS		(SKL_ADSP_GEN_BASE + 0x04)
#define SKL_ADSP_REG_ADSPIC		(SKL_ADSP_GEN_BASE + 0x08)
#define SKL_ADSP_REG_ADSPIS		(SKL_ADSP_GEN_BASE + 0x0C)
#define SKL_ADSP_REG_ADSPIC2		(SKL_ADSP_GEN_BASE + 0x10)
#define SKL_ADSP_REG_ADSPIS2		(SKL_ADSP_GEN_BASE + 0x14)

/* Intel HD Audio Inter-Processor Communication Registers */
#define SKL_ADSP_IPC_BASE		0x40
#define SKL_ADSP_REG_HIPCT		(SKL_ADSP_IPC_BASE + 0x00)
#define SKL_ADSP_REG_HIPCTE		(SKL_ADSP_IPC_BASE + 0x04)
#define SKL_ADSP_REG_HIPCI		(SKL_ADSP_IPC_BASE + 0x08)
#define SKL_ADSP_REG_HIPCIE		(SKL_ADSP_IPC_BASE + 0x0C)
#define SKL_ADSP_REG_HIPCCTL		(SKL_ADSP_IPC_BASE + 0x10)

/*  HIPCI */
#define SKL_ADSP_REG_HIPCI_BUSY		BIT(31)

/* HIPCIE */
#define SKL_ADSP_REG_HIPCIE_DONE	BIT(30)

/* HIPCCTL */
#define SKL_ADSP_REG_HIPCCTL_DONE	BIT(1)
#define SKL_ADSP_REG_HIPCCTL_BUSY	BIT(0)

/* HIPCT */
#define SKL_ADSP_REG_HIPCT_BUSY		BIT(31)

/* FW base IDs */
#define SKL_INSTANCE_ID			0
#define SKL_BASE_FW_MODULE_ID		0

/* Intel HD Audio SRAM Window 1 */
#define SKL_ADSP_SRAM1_BASE		0xA000

#define SKL_ADSP_MMIO_LEN		0x10000

#define SKL_ADSP_W0_STAT_SZ		0x1000

#define SKL_ADSP_W0_UP_SZ		0x1000

#define SKL_ADSP_W1_SZ			0x1000

#define SKL_FW_STS_MASK			0xf

#define SKL_FW_INIT			0x1
#define SKL_FW_RFW_START		0xf

#define SKL_ADSPIC_IPC			1
#define SKL_ADSPIS_IPC			1

/* Core ID of core0 */
#define SKL_DSP_CORE0_ID		0

/* Mask for a given core index, c; c = 0, 1.. number of supported cores - 1 */
#define SKL_DSP_CORE_MASK(c)   (1 << (c))

/*
 * Core 0 mask = SKL_DSP_CORE_MASK(0); Defined separately
 * since it is used often
 */
#define SKL_DSP_CORE0_MASK	1

/* Mask for a given number of cores, nc;
 * in typical use, nc = number of supported cores
 */
#define SKL_DSP_CORES_MASK(nc)     ((1 << (nc)) - 1)

/* ADSPCS - Audio DSP Control & Status */

/*
 * Core Reset - asserted high
 * CRST Mask for a given core mask pattern, cm
 */
#define SKL_ADSPCS_CRST_SHIFT	0
#define SKL_ADSPCS_CRST_MASK(cm)       ((cm) << SKL_ADSPCS_CRST_SHIFT)

/*
 * Core run/stall - when set to '1' core is stalled
 * CSTALL Mask for a given core mask pattern, cm
 */
#define SKL_ADSPCS_CSTALL_SHIFT	8
#define SKL_ADSPCS_CSTALL_MASK(cm)     ((cm) << SKL_ADSPCS_CSTALL_SHIFT)

/*
 * Set Power Active - when set to '1' turn cores on
 * SPA Mask for a given core mask pattern, cm
 */
#define SKL_ADSPCS_SPA_SHIFT	16
#define SKL_ADSPCS_SPA_MASK(cm)        ((cm) << SKL_ADSPCS_SPA_SHIFT)

/*
 * Current Power Active - power status of cores, set by hardware
 * CPA Mask for a given core mask pattern, cm
 */
#define SKL_ADSPCS_CPA_SHIFT	24
#define SKL_ADSPCS_CPA_MASK(cm)        ((cm) << SKL_ADSPCS_CPA_SHIFT)

/** FW Extended Manifest Header id = $AE1 */
#define SKL_EXT_MANIFEST_MAGIC_HEADER_ID   0x31454124

/*DSP notification events*/
#define EVENT_GLB_NOTIFY_PHRASE_DETECTED  4

/* DSP Core state */
enum skl_dsp_states {
	SKL_DSP_RUNNING = 1,
	SKL_DSP_RUNNING_D0I3, /* Running in D0i3 state*/
	SKL_DSP_RESET,
};

struct skl_dsp_fw_ops {
	int (*load_fw)(struct sst_dsp  *ctx);
	/* FW module parser/loader */
	int (*load_library)(struct sst_dsp *ctx);
	int (*parse_fw)(struct sst_dsp *ctx);
	int (*set_state_D0)(struct sst_dsp *ctx, unsigned int core_id);
	int (*set_state_D3)(struct sst_dsp *ctx, unsigned int core_id);
	int (*set_state_D0i3)(struct sst_dsp *ctx);
	int (*set_state_D0i0)(struct sst_dsp *ctx);
	unsigned int (*get_fw_errcode)(struct sst_dsp *ctx);
	int (*load_mod)(struct sst_dsp *ctx, u16 mod_id, char *mod_name);
	int (*unload_mod)(struct sst_dsp *ctx, u16 mod_id);

};

struct skl_dsp_loader_ops {
	int stream_tag;

	int (*alloc_dma_buf)(struct device *dev,
		struct snd_dma_buffer *dmab, size_t size);
	int (*free_dma_buf)(struct device *dev,
		struct snd_dma_buffer *dmab);
	int (*prepare)(struct device *dev, unsigned int format,
				unsigned int byte_size,
				struct snd_dma_buffer *bufp);
	int (*trigger)(struct device *dev, bool start, int stream_tag);

	int (*cleanup)(struct device *dev, struct snd_dma_buffer *dmab,
				 int stream_tag);
};

struct skl_hwd_event {
	bool is_hwd_event;
};

struct skl_notify_data {
	u32 type;
	u32 length;
	char data[0];
};

struct skl_dsp_notify_ops {
	int (*notify_cb)(struct skl_sst *skl, unsigned int event, struct skl_notify_data *notify_data);
};

struct skl_load_module_info {
	u16 mod_id;
	const struct firmware *fw;
};

struct skl_module_table {
	struct skl_load_module_info *mod_info;
	unsigned int usage_cnt;
	struct list_head list;
};

struct skl_ext_manifest_header {
    u32  ext_manifest_id;
    u32  ext_manifest_len;
    u16  ext_manifest_version_major;
    u16  ext_manifest_version_minor;
    u32  ext_manifest_entries;
} __packed;

void skl_cldma_process_intr(struct sst_dsp *ctx);
void skl_cldma_int_disable(struct sst_dsp *ctx);
int skl_cldma_prepare(struct sst_dsp *ctx);

struct sst_dsp *skl_dsp_ctx_init(struct device *dev,
		struct sst_dsp_device *sst_dev, int irq);

unsigned int skl_dsp_get_enabled_cores(struct sst_dsp  *ctx);
void skl_dsp_init_core_state(struct sst_dsp *ctx);
int skl_dsp_enable_core(struct sst_dsp *ctx, unsigned int core_mask);
int skl_dsp_disable_core(struct sst_dsp *ctx, unsigned int core_mask);
int skl_dsp_core_power_up(struct sst_dsp *ctx, unsigned int core_mask);
int skl_dsp_core_power_down(struct sst_dsp *ctx, unsigned int core_mask);
int skl_dsp_core_unset_reset_state(struct sst_dsp *ctx, unsigned int core_mask);
int skl_dsp_start_core(struct sst_dsp *ctx, unsigned int core_mask);

irqreturn_t skl_dsp_sst_interrupt(int irq, void *dev_id);
int skl_dsp_wake(struct sst_dsp *ctx);
int skl_dsp_sleep(struct sst_dsp *ctx);
void skl_dsp_free(struct sst_dsp *dsp);

int skl_dsp_get_core(struct sst_dsp *ctx, unsigned int core_id);
int skl_dsp_put_core(struct sst_dsp *ctx, unsigned int core_id);

int skl_dsp_boot(struct sst_dsp *ctx);
int skl_sst_dsp_init_hw(struct device *dev, void __iomem *mmio_base, int irq,
		struct skl_dsp_loader_ops dsp_ops, struct skl_sst **dsp);
int bxt_sst_dsp_init_hw(struct device *dev, void __iomem *mmio_base, int irq,
		struct skl_dsp_loader_ops dsp_ops, struct skl_sst **dsp);
int skl_sst_dsp_init_fw(struct device *dev, struct skl_sst *ctx);
int bxt_sst_dsp_init_fw(struct device *dev, struct skl_sst *ctx);
void skl_sst_dsp_cleanup(struct device *dev, struct skl_sst *ctx);
void bxt_sst_dsp_cleanup(struct device *dev, struct skl_sst *ctx);

#endif /*__SKL_SST_DSP_H__*/
