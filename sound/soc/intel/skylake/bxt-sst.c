/*
 *  bxt-sst.c - HDA DSP library functions for BXT platform
 *
 *  Copyright (C) 2015 Intel Corp
 *  Author:Rafal Redzimski <rafal.f.redzimski@intel.com>
 *	   Jeeja KP <jeeja.kp@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <asm/cacheflush.h>

#include "../common/sst-dsp.h"
#include "../common/sst-ipc.h"
#include "skl-sst-ipc.h"
#include "skl-tplg-interface.h"
#include "skl-fwlog.h"

#define FW_ROM_INIT_DONE                0x1

#define BXT_FW_ROM_BASEFW_ENTERED_TIMEOUT	3000
#define BXT_ROM_INIT_HIPCIE_TIMEOUT	500
#define BXT_ROM_INIT_DONE_TIMEOUT	500
#define BXT_IPC_PURGE_FW	0x01004000

#define BXT_FW_ROM_BASEFW_ENTERED	0x5
#define BXT_ADSP_SRAM0_BASE	0x80000

/* Trace Buffer Winddow */
#define BXT_ADSP_SRAM2_BASE	0x0C0000
#define BXT_ADSP_W2_SIZE	0x2000
#define BXT_ADSP_WP_DSP0	(BXT_ADSP_SRAM0_BASE+0x30)
#define BXT_ADSP_WP_DSP1	(BXT_ADSP_SRAM0_BASE+0x34)
#define BXT_ADSP_NR_DSP	2

/* Firmware status window */
#define BXT_ADSP_REG_FW_STATUS	BXT_ADSP_SRAM0_BASE
#define BXT_ADSP_ERROR_CODE     (BXT_ADSP_REG_FW_STATUS + 0x4)

#define BXT_INSTANCE_ID 0
#define BXT_BASE_FW_MODULE_ID 0

#define BXT_ADSP_SRAM1_BASE	0xA0000
#define BXT_ADSP_W0_STAT_SZ	0x1000
#define BXT_ADSP_W0_UP_SZ	0x1000
#define BXT_ADSP_W1_SZ  	0x1000

/* Delay before scheduling D0i3 entry */
#define BXT_D0I3_DELAY 5000

static int bxt_load_base_firmware(struct sst_dsp *ctx);
static int bxt_set_dsp_D0(struct sst_dsp *ctx, unsigned int core_id);
static int bxt_set_dsp_D3(struct sst_dsp *ctx, unsigned int core_id);
static int bxt_schedule_dsp_D0i3(struct sst_dsp *ctx);
static int bxt_set_dsp_D0i0(struct sst_dsp *ctx);
static void bxt_set_dsp_D0i3(struct work_struct *work);
static int bxt_load_library(struct sst_dsp *ctx,
		struct skl_dfw_manifest *minfo);

static unsigned int bxt_get_errorcode(struct sst_dsp *ctx)
{
	 return sst_dsp_shim_read(ctx, BXT_ADSP_ERROR_CODE);
}

static struct skl_dsp_fw_ops bxt_fw_ops = {
	.set_state_D0 = bxt_set_dsp_D0,
	.set_state_D3 = bxt_set_dsp_D3,
	.set_state_D0i3 = bxt_schedule_dsp_D0i3,
	.set_state_D0i0 = bxt_set_dsp_D0i0,
	.load_fw = bxt_load_base_firmware,
	.get_fw_errcode = bxt_get_errorcode,
	.load_library = bxt_load_library,
};

static struct sst_ops skl_ops = {
	.irq_handler = skl_dsp_sst_interrupt,
	.write = sst_shim32_write,
	.read = sst_shim32_read,
	.ram_read = sst_memcpy_fromio_32,
	.ram_write = sst_memcpy_toio_32,
	.free = skl_dsp_free,
};

static struct sst_dsp_device skl_dev = {
	.thread = skl_dsp_irq_thread_handler,
	.ops = &skl_ops,
};

int bxt_sst_dsp_init(struct device *dev, void __iomem *mmio_base, int irq,
		struct skl_dsp_loader_ops dsp_ops, struct skl_sst **dsp,
		struct skl_dfw_manifest *minfo)
{
	struct skl_sst *skl;
	struct sst_dsp *sst;
	u32 dsp_wp[] = {BXT_ADSP_WP_DSP0, BXT_ADSP_WP_DSP1};
	int ret = 0;

	dev_dbg(dev, "In %s\n", __func__);

	skl = devm_kzalloc(dev, sizeof(*skl), GFP_KERNEL);

	if (skl == NULL)
		return -ENOMEM;

	skl->dev = dev;
	skl_dev.thread_context = skl;

	skl->dsp = skl_dsp_ctx_init(dev, &skl_dev, irq);
	if (!skl->dsp) {
		dev_err(skl->dev, "%s: no device\n", __func__);
		return -ENODEV;
	}

	sst = skl->dsp;

	sst->dsp_ops = dsp_ops;
	sst->fw_ops = bxt_fw_ops;
	sst->addr.lpe = mmio_base;
	sst->addr.shim = mmio_base;
	sst->addr.sram0_base = BXT_ADSP_SRAM0_BASE;
	sst->addr.sram1_base = BXT_ADSP_SRAM1_BASE;
	sst->addr.w0_stat_sz = BXT_ADSP_W0_STAT_SZ;
	sst->addr.w0_up_sz = BXT_ADSP_W0_UP_SZ;

	sst_dsp_mailbox_init(sst, (BXT_ADSP_SRAM0_BASE + BXT_ADSP_W0_STAT_SZ),
			BXT_ADSP_W0_UP_SZ, BXT_ADSP_SRAM1_BASE, BXT_ADSP_W1_SZ);
	ret = skl_dsp_init_trace_window(sst, dsp_wp, BXT_ADSP_SRAM2_BASE,
				BXT_ADSP_W2_SIZE, BXT_ADSP_NR_DSP);
	if (ret) {
		dev_err(dev, "FW tracing init failed : %x", ret);
		return ret;
	}

	ret = skl_ipc_init(dev, skl);
	if (ret)
		return ret;

	sst->core_info.cores = 2;

	skl->boot_complete = false;
	init_waitqueue_head(&skl->boot_wait);

	INIT_DELAYED_WORK(&skl->d0i3_data.d0i3_work, bxt_set_dsp_D0i3);

	ret = sst->fw_ops.load_fw(sst);
	if (ret < 0) {
		dev_err(dev, "Load base fw failed : %x", ret);
		return ret;
	}

	if (minfo->lib_count > 1) {
		ret = sst->fw_ops.load_library(sst, minfo);
		if (ret < 0) {
			dev_err(dev, "Load Library failed : %x", ret);
			return ret;
		}
	}

	if (dsp)
		*dsp = skl;
	dev_dbg(dev, "Exit %s\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(bxt_sst_dsp_init);


/* First boot sequence has some extra steps due to a ROM bug on BXT.
 * Due to the bug core 0 waits for power status on core 1. The work
 * around is to power up core 1 also momentarily, keep it in reset/stall
 * and then turn it off
 */

#define BXT_ROM_BUG_WA

static int sst_bxt_prepare_fw(struct sst_dsp *ctx, const void *fwdata,
		u32 fwsize)
{

	int ret;
	int i;
	u32 reg;
	int stream_tag;

	dev_dbg(ctx->dev, "starting to prepare host dma: fwsize=%d\n", fwsize);
	stream_tag = ctx->dsp_ops.prepare(ctx->dev, 0x40, fwsize, &ctx->dmab);
	if (stream_tag <= 0) {
		dev_err(ctx->dev, "Failed to prepare DMA engine for FW loading, err: %d\n", stream_tag);
		return stream_tag;
	}

	ctx->dsp_ops.stream_tag = stream_tag;
	memcpy(ctx->dmab.area, fwdata, fwsize);
	/* make sure FW is flushed to DDR */
	clflush_cache_range(ctx->dmab.area, fwsize);

#ifdef BXT_ROM_BUG_WA
	/* Step 1.a: Power up core 0 and core1 (Extra step due to ROM bug) */
	ret = skl_dsp_core_power_up(ctx, SKL_DSP_CORE0_MASK | SKL_DSP_CORE_MASK(1));
	if (ret < 0) {
		dev_err(ctx->dev, "dsp core0/1 power up failed\n");
		goto prepare_fw_load_failed;
	}
#else
	/* Step 1: Power up core0 */
	ret = skl_dsp_core_power_up(ctx, SKL_DSP_CORE0_MASK);
	if (ret < 0)
		goto prepare_fw_load_failed;
#endif

	/* Step 2: Purge FW request */
	sst_dsp_shim_write(ctx, SKL_ADSP_REG_HIPCI, SKL_ADSP_REG_HIPCI_BUSY |
					 BXT_IPC_PURGE_FW | (stream_tag - 1));

	/* Step 3: Unset core0 reset state */
	ret = skl_dsp_core_unset_reset_state(ctx, SKL_DSP_CORE0_MASK);
	if (ret < 0)
		goto prepare_fw_load_failed;

	/* Step 4: unstall/run core0 */
	dev_dbg(ctx->dev, "unstall/run core0...");
	sst_dsp_shim_update_bits_unlocked(ctx, SKL_ADSP_REG_ADSPCS,
			SKL_ADSPCS_CSTALL_MASK(SKL_DSP_CORE0_MASK), 0);

	/* Step 5: Wait for DONE Bit */
	for (i = BXT_ROM_INIT_HIPCIE_TIMEOUT; i > 0; --i) {
		reg = sst_dsp_shim_read(ctx, SKL_ADSP_REG_HIPCIE);

		if (reg & SKL_ADSP_REG_HIPCIE_DONE) {
			sst_dsp_shim_update_bits_forced(ctx, SKL_ADSP_REG_HIPCIE,
					SKL_ADSP_REG_HIPCIE_DONE,
					SKL_ADSP_REG_HIPCIE_DONE);
			break;
		}

		mdelay(1);
	}
	if (!i) {
		dev_err(ctx->dev, "Timeout waiting for HIPCIE done, reg: 0x%x\n", reg);
		/*FIXME */
		sst_dsp_shim_update_bits(ctx, SKL_ADSP_REG_HIPCIE,
				SKL_ADSP_REG_HIPCIE_DONE,
				SKL_ADSP_REG_HIPCIE_DONE);
	}
	dev_dbg(ctx->dev, "******HIPCIE reg: 0x%x\n", reg);

#ifdef BXT_ROM_BUG_WA
	/* Step 5.a: power down core1 (Extra step due to ROM bug) */
	ret = skl_dsp_core_power_down(ctx, SKL_DSP_CORE_MASK(1));
	if (ret < 0) {
		dev_err(ctx->dev, "dsp core1 power down failed\n");
		goto prepare_fw_load_failed;
	}
#endif
	/* Step 6: enable Interrupt */
	skl_ipc_int_enable(ctx);
	skl_ipc_op_int_enable(ctx);

	/* Step 7: Wait for ROM init */
	for (i = BXT_ROM_INIT_DONE_TIMEOUT; i > 0; --i) {
		if (FW_ROM_INIT_DONE ==
			(sst_dsp_shim_read(ctx, BXT_ADSP_REG_FW_STATUS) &
				SKL_FW_STS_MASK)) {
				dev_err(ctx->dev, "ROM loaded, we can continue with FW loading\n");
			break;
		}
		mdelay(1);
	}
	if (!i) {
		dev_err(ctx->dev, "Timeout waiting for ROM init done, reg: 0x%x\n", reg);
		ret = -EIO;
		goto prepare_fw_load_failed;
	}

	return 0;

prepare_fw_load_failed:
	ctx->dsp_ops.cleanup(ctx->dev, &ctx->dmab, stream_tag);
#ifdef BXT_ROM_BUG_WA
	skl_dsp_core_power_down(ctx, SKL_DSP_CORE_MASK(1));
#endif
	skl_dsp_disable_core(ctx, SKL_DSP_CORE0_MASK);
	return ret;
}

static int sst_transfer_fw_host_dma(struct sst_dsp *ctx)
{
	int ret = 0;


	ctx->dsp_ops.trigger(ctx->dev, true, ctx->dsp_ops.stream_tag);
	ret = sst_dsp_register_poll(ctx, BXT_ADSP_REG_FW_STATUS, SKL_FW_STS_MASK,
			BXT_FW_ROM_BASEFW_ENTERED,
			BXT_FW_ROM_BASEFW_ENTERED_TIMEOUT,
			"Firmware boot");

	ctx->dsp_ops.trigger(ctx->dev, false, ctx->dsp_ops.stream_tag);
	ctx->dsp_ops.cleanup(ctx->dev, &ctx->dmab, ctx->dsp_ops.stream_tag);
	return ret;
}

static void bxt_set_dsp_D0i3(struct work_struct *work)
{
	int ret;
	struct skl_ipc_d0ix_msg msg;
	struct skl_sst *skl = container_of(work,
			struct skl_sst, d0i3_data.d0i3_work.work);
	struct sst_dsp *ctx = skl->dsp;

	dev_dbg(ctx->dev, "In %s:\n", __func__);

	/* D0i3 entry allowed only if core 0 alone is running */
	if (SKL_DSP_CORE0_MASK != skl_dsp_get_enabled_cores(ctx)) {
		dev_warn(ctx->dev,
				"D0i3 allowed when only core0 running:Exit\n");
		return;
	}

	msg.instance_id = 0;
	msg.module_id = 0;
	msg.wake = 1;
	msg.streaming = 1;

	ret =  skl_ipc_set_d0ix(&skl->ipc, &msg);

	if (ret < 0) {
		dev_err(ctx->dev, "Failed to set DSP to D0i3 state\n");
		return;
	}
	/* Set Vendor specific register D0I3C.I3 to enable D0i3*/
	if (skl->update_d0i3c)
		skl->update_d0i3c(skl->dev, true);

	ctx->core_info.core_state[SKL_DSP_CORE0_ID] = SKL_DSP_RUNNING_D0I3;
}

static int bxt_schedule_dsp_D0i3(struct sst_dsp *ctx)
{
	struct skl_sst *skl = ctx->thread_context;
	struct skl_d0i3_data *d0i3_data = &skl->d0i3_data;

	/* Schedule D0i3 if DSP the stream counts are appropriate */

	if ((d0i3_data->d0i3_stream_count > 0) &&
			(d0i3_data->non_d0i3_stream_count == 0)) {

		dev_dbg(ctx->dev, "%s: Schedule D0i3\n", __func__);

		schedule_delayed_work(&d0i3_data->d0i3_work,
				msecs_to_jiffies(BXT_D0I3_DELAY));
	}

	return 0;
}

static int bxt_set_dsp_D0i0(struct sst_dsp *ctx)
{
	int ret;
	struct skl_ipc_d0ix_msg msg;
	struct skl_sst *skl = ctx->thread_context;

	dev_dbg(ctx->dev, "In %s:\n", __func__);

	/* First Cancel any pending attempt to put DSP to D0i3 */
	cancel_delayed_work_sync(&skl->d0i3_data.d0i3_work);

	/* If DSP is currently in D0i3, bring it to D0i0 */
	if (ctx->core_info.core_state[SKL_DSP_CORE0_ID] != SKL_DSP_RUNNING_D0I3)
		return 0;

	dev_dbg(ctx->dev, "Set DSP to D0i0\n");

	msg.instance_id = 0;
	msg.module_id = 0;
	msg.streaming = 1;
	msg.wake = 0;

	/* Clear Vendor specific register D0I3C.I3 to disable D0i3*/
	if (skl->update_d0i3c)
		skl->update_d0i3c(skl->dev, false);

	ret =  skl_ipc_set_d0ix(&skl->ipc, &msg);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to set DSP to D0i0\n");
		return ret;
	}

	ctx->core_info.core_state[SKL_DSP_CORE0_ID] = SKL_DSP_RUNNING;

	return 0;
}

int bxt_set_dsp_D0(struct sst_dsp *ctx, unsigned int core_id)
{
	int ret;
	struct skl_ipc_dxstate_info dx;
	struct skl_sst *skl = ctx->thread_context;
	unsigned int core_mask = SKL_DSP_CORE_MASK(core_id);


	dev_dbg(ctx->dev, "In %s : core id = %d\n", __func__, core_id);

#ifdef BXT_ROM_BUG_WA
	/* If core 0 is being turned on, turn on core 1 as well */
	if (core_id == SKL_DSP_CORE0_ID)
		ret = skl_dsp_core_power_up(ctx, core_mask | SKL_DSP_CORE_MASK(1));
	else
		ret = skl_dsp_core_power_up(ctx, core_mask);
#else
	ret = skl_dsp_core_power_up(ctx, core_mask);
#endif
	if (ret < 0)
		goto err;

	if (core_id == SKL_DSP_CORE0_ID) {
		dev_dbg(ctx->dev, "Enable Interrupts\n");
		/* Enable interrupt after SPA is set and before DSP is unstalled */
		skl_ipc_int_enable(ctx);
		skl_ipc_op_int_enable(ctx);
		skl->boot_complete = false;
	}

	ret = skl_dsp_start_core(ctx, core_mask);
	if (ret < 0)
		goto err;

	if (core_id == SKL_DSP_CORE0_ID) {
		ret = wait_event_timeout(skl->boot_wait,
				skl->boot_complete,
				msecs_to_jiffies(SKL_IPC_BOOT_MSECS));

#ifdef BXT_ROM_BUG_WA
		/* If core 1 was turned on to workaround the ROM bug, turn it off */
		skl_dsp_core_power_down(ctx, SKL_DSP_CORE_MASK(1));
#endif
		if (ret == 0) {
			dev_err(ctx->dev, "%s: error DSP boot timeout\n", __func__);
			dev_err(ctx->dev, "Error code=0x%x: FW status=0x%x\n",
					sst_dsp_shim_read(ctx, BXT_ADSP_ERROR_CODE),
					sst_dsp_shim_read(ctx, BXT_ADSP_REG_FW_STATUS));
			dev_err(ctx->dev, "Failed to set core0 to D0 state\n");
			ret = -EIO;
			goto err;
		}
	}

	/*
	 * If any core other than core 0 is being moved to D0, send the
	 * set dx IPC for the core.
	 * NOTE: As per the current BXT FW implementation, set dx can be
	 * sent for core 0 also though set dx is not necessary for core 0. So
	 * the following check for other cores is not strictly needed for BXT.
	 * However, set dx is not being sent for core 0 in the following code
	 * in order not to make any assumption about set dx support for core 0
	 * in FW for platforms that may reuse this driver.
	 */
	if (core_id != SKL_DSP_CORE0_ID) {
		dx.core_mask = core_mask;
		dx.dx_mask = core_mask;

		ret = skl_ipc_set_dx(&skl->ipc, BXT_INSTANCE_ID,
					BXT_BASE_FW_MODULE_ID, &dx);
		if (ret < 0) {
			dev_err(ctx->dev, "Failed to set dsp to D0:core id = %d\n",
					core_id);
			goto err;
		}
	}

	ctx->core_info.core_state[core_id] = SKL_DSP_RUNNING;
	return 0;
err:
	skl_dsp_disable_core(ctx, core_mask);
	return ret;
}

static int bxt_set_dsp_D3(struct sst_dsp *ctx, unsigned int core_id)
{
	int ret;
	struct skl_ipc_dxstate_info dx;
	struct skl_sst *skl = ctx->thread_context;
	unsigned int core_mask = SKL_DSP_CORE_MASK(core_id);

	dev_dbg(ctx->dev, "In %s : core id = %d\n", __func__, core_id);

	dx.core_mask = core_mask;
	dx.dx_mask = 0;

	dev_dbg(ctx->dev, "core mask=%x dx_mask=%x\n",
			dx.core_mask, dx.dx_mask);

	ret = skl_ipc_set_dx(&skl->ipc, BXT_INSTANCE_ID,
				BXT_BASE_FW_MODULE_ID, &dx);
	if (ret < 0)
		dev_err(ctx->dev,
			"Failed to set DSP to D3:core id = %d;Continue reset\n",
			core_id);

	ret = skl_dsp_disable_core(ctx, core_mask);

	if (ret < 0)
		return ret;

	ctx->core_info.core_state[core_id] = SKL_DSP_RESET;
	return 0;

}

static int bxt_load_base_firmware(struct sst_dsp *ctx)
{
	struct skl_ext_manifest_header *hdr;
	u32 size;
	const void *data;
	int ret = 0;
	struct firmware *fw = NULL;
	struct skl_sst *skl = ctx->thread_context;

	dev_dbg(ctx->dev, "In %s\n", __func__);

	ret = request_firmware(&fw, "dsp_fw_release.bin", ctx->dev);
	if (ret < 0) {
		dev_err(ctx->dev, "Request firmware failed %d\n", ret);
		goto sst_load_base_firmware_failed;
	}

	size = fw->size;
	data = fw->data;
	hdr = (struct skl_ext_manifest_header *)fw->data;
	if (hdr->ext_manifest_id == SKL_EXT_MANIFEST_MAGIC_HEADER_ID) {
		dev_dbg(ctx->dev, "Found Extended manifest in FW Binary\n");
		if (hdr->ext_manifest_len >= fw->size) {
			ret = -EINVAL;
			goto sst_load_base_firmware_failed;
		}
		size = fw->size - hdr->ext_manifest_len;
		data = fw->data + hdr->ext_manifest_len;
	}

	ret = sst_bxt_prepare_fw(ctx, data, size);
	/* FIXME: Retry Enabling core and ROM load. Retry seemed to help during
	   A0 Power On.So retain it for now. */
	if (ret < 0) {
		ret = sst_bxt_prepare_fw(ctx, data, size);
		if (ret < 0) {
			dev_err(ctx->dev, "Core En/ROM load fail:%d\n", ret);
			goto sst_load_base_firmware_failed;
		}
	}
	ret = sst_transfer_fw_host_dma(ctx);

	if (ret < 0) {
		dev_err(ctx->dev, "Transfer firmware failed %d\n", ret);
		dev_info(ctx->dev, "Error code=0x%x: FW status=0x%x\n",
				sst_dsp_shim_read(ctx, BXT_ADSP_ERROR_CODE),
				sst_dsp_shim_read(ctx, BXT_ADSP_REG_FW_STATUS));
		skl_dsp_disable_core(ctx, SKL_DSP_CORE0_MASK);
	} else {
		dev_dbg(ctx->dev, "Firmware download successful\n");
		ret = wait_event_timeout(skl->boot_wait, skl->boot_complete,
						msecs_to_jiffies(SKL_IPC_BOOT_MSECS));
		if (ret == 0) {
			dev_err(ctx->dev, "DSP boot failed, FW Ready timed-out\n");
			skl_dsp_disable_core(ctx, SKL_DSP_CORE0_MASK);
			ret = -EIO;
		} else {
			skl_dsp_init_core_state(ctx);
			ret = 0;
		}
	}
sst_load_base_firmware_failed:
	release_firmware(fw);
	dev_dbg(ctx->dev, "Exit %s\n", __func__);
	return ret;
}

static int bxt_load_library(struct sst_dsp *ctx, struct skl_dfw_manifest *minfo)
{
	struct snd_dma_buffer dmab;
	struct skl_sst *skl = ctx->thread_context;
	struct firmware *fw = NULL;
	struct skl_ext_manifest_header *hdr;
	u32 size;
	const void *data;
	int ret = 0, i, dma_id, stream_tag;

	for (i = 1; i < minfo->lib_count; i++) {
		fw = NULL;
		ret = request_firmware(&fw, minfo->lib[i].name, ctx->dev);
		if (ret < 0) {
			dev_err(ctx->dev, "Request firmware failed %d for library: %s\n", ret, minfo->lib[i].name);
			goto load_library_failed;
		}

		size = fw->size;
		data = fw->data;
		hdr = (struct skl_ext_manifest_header *)fw->data;
		if (hdr->ext_manifest_id == SKL_EXT_MANIFEST_MAGIC_HEADER_ID) {
			dev_dbg(ctx->dev, "Found Extended manifest in Library Binary\n");
			/* Check for extended manifest is not grater than FW size*/
			if (hdr->ext_manifest_len >= fw->size) {
				ret = -EINVAL;
				goto load_library_failed;
			}

			size = fw->size - hdr->ext_manifest_len;
			data = (u8 *)fw->data + hdr->ext_manifest_len;
		}

		dev_dbg(ctx->dev, "Starting to preapre host dma for library name \
			: %s of size:%zx\n", minfo->lib[i].name, size);
		stream_tag = ctx->dsp_ops.prepare(ctx->dev, 0x40, size,
						&dmab);
		if (stream_tag <= 0) {
			dev_err(ctx->dev, "Failed to prepare DMA engine for \
				FW loading, err: %x\n", stream_tag);
			ret = stream_tag;
			goto load_library_failed;
		}
		dma_id = stream_tag - 1;
		memcpy(dmab.area, data, size);

		/* make sure Library is flushed to DDR */
		clflush_cache_range(dmab.area, size);

		ctx->dsp_ops.trigger(ctx->dev, true, stream_tag);
		ret = skl_sst_ipc_load_library(&skl->ipc, dma_id, i);
		if (ret < 0) {
			dev_err(ctx->dev, "Load Library  IPC failed: %d for library: %s\n", ret, minfo->lib[i].name);
			dev_info(ctx->dev, "Error code=0x%x: FW status=0x%x\n",
					sst_dsp_shim_read(ctx, BXT_ADSP_ERROR_CODE),
					sst_dsp_shim_read(ctx, BXT_ADSP_REG_FW_STATUS));
		}

		ctx->dsp_ops.trigger(ctx->dev, false, stream_tag);
		ctx->dsp_ops.cleanup(ctx->dev, &dmab, stream_tag);
		release_firmware(fw);
	}

	return ret;

load_library_failed:
	release_firmware(fw);
	return ret;
}

void bxt_sst_dsp_cleanup(struct device *dev, struct skl_sst *ctx)
{
	skl_ipc_free(&ctx->ipc);
	ctx->dsp->cl_dev.ops.cl_cleanup_controller(ctx->dsp);
	if (ctx->dsp->addr.lpe)
		iounmap(ctx->dsp->addr.lpe);

	ctx->dsp->ops->free(ctx->dsp);
}
EXPORT_SYMBOL_GPL(bxt_sst_dsp_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Broxton IPC driver");
