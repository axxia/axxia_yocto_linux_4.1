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

#include "../common/sst-dsp.h"
#include "../common/sst-dsp-priv.h"
#include "../common/sst-ipc.h"
#include "skl-sst-ipc.h"
#include "skl-tplg-interface.h"

#define FW_ROM_INIT_DONE                0x1

#define BXT_FW_ROM_BASEFW_ENTERED_TIMEOUT	300
#define BXT_ROM_INIT_HIPCIE_TIMEOUT	500
#define BXT_ROM_INIT_DONE_TIMEOUT	500
#define BXT_IPC_PURGE_FW	0x01004000

#define BXT_FW_ROM_BASEFW_ENTERED	0x5
#define BXT_ADSP_SRAM0_BASE	0x80000
/* Firmware status window */
#define BXT_ADSP_REG_FW_STATUS	BXT_ADSP_SRAM0_BASE
#define BXT_ADSP_ERROR_CODE     (BXT_ADSP_REG_FW_STATUS + 0x4)

#define BXT_INSTANCE_ID 0
#define BXT_BASE_FW_MODULE_ID 0

#define BXT_ADSP_SRAM1_BASE	0xA0000
#define BXT_ADSP_W0_STAT_SZ	0x800
#define BXT_ADSP_W0_UP_SZ	0x800
#define BXT_ADSP_W1_SZ  0x1000

static int bxt_load_base_firmware(struct sst_dsp *ctx);
static int bxt_set_dsp_D0(struct sst_dsp *ctx);
static int bxt_set_dsp_D3(struct sst_dsp *ctx);
static int bxt_load_library(struct sst_dsp *ctx,
		struct skl_dfw_manifest *minfo);

static unsigned int bxt_get_errorcode(struct sst_dsp *ctx)
{
	 return sst_dsp_shim_read(ctx, BXT_ADSP_ERROR_CODE);
}

static struct skl_dsp_fw_ops bxt_fw_ops = {
	.set_state_D0 = bxt_set_dsp_D0,
	.set_state_D3 = bxt_set_dsp_D3,
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

	ret = skl_ipc_init(dev, skl);
	if (ret)
		return ret;

	skl->boot_complete = false;
	init_waitqueue_head(&skl->boot_wait);

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

static int bxt_load_library(struct sst_dsp *ctx, struct skl_dfw_manifest *minfo)
{
	struct snd_dma_buffer dmab;
	struct skl_sst *skl = ctx->thread_context;
	const struct firmware *fw = NULL;
	int ret = 0, i, dma_id, stream_tag;

	for (i = 1; i < minfo->lib_count; i++) {
		fw = NULL;
		ret = request_firmware(&fw, minfo->lib[i].name, ctx->dev);
		if (ret < 0) {
			dev_err(ctx->dev, "Request firmware failed %d for library: %s\n", ret, minfo->lib[i].name);
			goto load_library_failed;
		}

		dev_dbg(ctx->dev, "Starting to preapre host dma for library name \
			: %s of size:%zx\n", minfo->lib[i].name, fw->size);
		stream_tag = ctx->dsp_ops.prepare(ctx->dev, 0x40, fw->size,
						&dmab);
		if (stream_tag <= 0) {
			dev_err(ctx->dev, "Failed to prepare DMA engine for \
				FW loading, err: %x\n", stream_tag);
			ret = stream_tag;
			goto load_library_failed;
		}
		dma_id = stream_tag - 1;
		memcpy(dmab.area, fw->data, fw->size);

		ctx->dsp_ops.trigger(ctx->dev, true, stream_tag);
		ret = skl_sst_ipc_load_library(&skl->ipc, dma_id, i);
		if (ret < 0)
			dev_err(ctx->dev, "Load Library  IPC failed: %d for library: %s\n", ret, minfo->lib[i].name);

		ctx->dsp_ops.trigger(ctx->dev, false, stream_tag);
		ctx->dsp_ops.cleanup(ctx->dev, &dmab, stream_tag);
		release_firmware(fw);
	}

	return ret;

load_library_failed:
	release_firmware(fw);
	return ret;
}

static int sst_bxt_prepare_fw(struct sst_dsp *ctx, const void *fwdata,
		u32 fwsize)
{

	int ret;
	int i;
	u32 reg;
	int stream_tag;

	dev_dbg(ctx->dev, "starting to prepare host dma fwsize=%x\n", fwsize);
	stream_tag = ctx->dsp_ops.prepare(ctx->dev, 0x40, fwsize, &ctx->dmab);
	if (stream_tag < 0) {
		dev_err(ctx->dev, "Failed to prepare DMA engine for FW loading, err: %x\n", stream_tag);
		return stream_tag;
	}

	ctx->dsp_ops.stream_tag = stream_tag;
	memcpy(ctx->dmab.area, fwdata, fwsize);
	/* Purge FW request */
	sst_dsp_shim_write(ctx, SKL_ADSP_REG_HIPCI, SKL_ADSP_REG_HIPCI_BUSY |
					 BXT_IPC_PURGE_FW | (stream_tag - 1));

	ret = skl_dsp_enable_core(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "Boot dsp core failed ret: %d\n", ret);
		ret = -EIO;
		goto base_fw_load_failed;
	}

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
	/*enable Interrupt */
	skl_ipc_int_enable(ctx);
	skl_ipc_op_int_enable(ctx);

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
		goto base_fw_load_failed;
	}
	return ret;
base_fw_load_failed:
	ctx->dsp_ops.cleanup(ctx->dev, &ctx->dmab, stream_tag);
	skl_dsp_disable_core(ctx);
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

int bxt_set_dsp_D0(struct sst_dsp *ctx)
{
	int ret = 0;
	struct skl_sst *skl = ctx->thread_context;

	dev_dbg(ctx->dev, "In %s:\n", __func__);

	skl->boot_complete = false;

	ret = skl_dsp_enable_core(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "enable dsp core failed ret: %d\n", ret);
		return ret;
	}

	/*enable interrupt*/
	skl_ipc_int_enable(ctx);
	skl_ipc_op_int_enable(ctx);

	ret = wait_event_timeout(skl->boot_wait, skl->boot_complete,
					msecs_to_jiffies(SKL_IPC_BOOT_MSECS));
	if (ret == 0) {
		dev_err(ctx->dev, "ipc: error DSP boot timeout\n");
		dev_err(ctx->dev, "Error code=0x%x: FW status=0x%x\n",
			sst_dsp_shim_read(ctx, BXT_ADSP_ERROR_CODE),
			sst_dsp_shim_read(ctx, BXT_ADSP_REG_FW_STATUS));
		return -EIO;
	}

	skl_dsp_set_state_locked(ctx, SKL_DSP_RUNNING);
	return 0;
}

static int bxt_set_dsp_D3(struct sst_dsp *ctx)
{
	int ret = 0;
	struct skl_ipc_dxstate_info dx;
	struct skl_sst *skl = ctx->thread_context;

	dev_dbg(ctx->dev, "In %s:\n", __func__);

	if (!is_skl_dsp_running(ctx))
		return ret;

	dx.core_mask = SKL_DSP_CORE0_MASK;
	dx.dx_mask = SKL_IPC_D3_MASK;

	dev_dbg(ctx->dev, "core mask=%x dx_mask=%x\n",
				 dx.core_mask, dx.dx_mask);
	ret = skl_ipc_set_dx(&skl->ipc, BXT_INSTANCE_ID, BXT_BASE_FW_MODULE_ID, &dx);

	if (ret < 0) {
		dev_err(ctx->dev, "Failed to set DSP to D3 state\n");
		return ret;
	}

	ret = skl_dsp_disable_core(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "disbale dsp core failed ret: %d\n", ret);
		ret = -EIO;
	}
	skl_dsp_set_state_locked(ctx, SKL_DSP_RESET);
	return 0;
}

static int bxt_load_base_firmware(struct sst_dsp *ctx)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	struct skl_sst *skl = ctx->thread_context;

	dev_dbg(ctx->dev, "In %s\n", __func__);

	ret = request_firmware(&fw, "dsp_fw_release.bin", ctx->dev);
	if (ret < 0) {
		dev_err(ctx->dev, "Request firmware failed %d\n", ret);
		goto sst_load_base_firmware_failed;
	}

	ret = sst_bxt_prepare_fw(ctx, fw->data, fw->size);
	/* FIXME: Retry Enabling core and ROM load. Retry seemed to help during
	   A0 Power On.So retain it for now. */
	if (ret < 0) {
		ret = sst_bxt_prepare_fw(ctx, fw->data, fw->size);
		if (ret < 0) {
			dev_err(ctx->dev, "Core En/ROM load fail:%d\n", ret);
			goto sst_load_base_firmware_failed;
		}
	}
	ret = sst_transfer_fw_host_dma(ctx);

	if (ret < 0) {
		dev_err(ctx->dev, "Transfer firmware failed %d\n", ret);
		skl_dsp_disable_core(ctx);
	} else {
		dev_dbg(ctx->dev, "Firmware download successful\n");
		ret = wait_event_timeout(skl->boot_wait, skl->boot_complete,
						msecs_to_jiffies(SKL_IPC_BOOT_MSECS));
		if (ret == 0) {
			dev_err(ctx->dev, "DSP boot failed, FW Ready timed-out\n");
			skl_dsp_disable_core(ctx);
			ret = -EIO;
		} else {
			skl_dsp_set_state_locked(ctx, SKL_DSP_RUNNING);
			ret = 0;
		}
	}
sst_load_base_firmware_failed:
	release_firmware(fw);
	dev_dbg(ctx->dev, "Exit %s\n", __func__);
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
