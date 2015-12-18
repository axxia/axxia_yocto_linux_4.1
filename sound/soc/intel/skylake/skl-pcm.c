/*
 *  skl-pcm.c -ASoC HDA Platform driver file implementing PCM functionality
 *
 *  Copyright (C) 2014-2015 Intel Corp
 *  Author:  Jeeja KP <jeeja.kp@intel.com>
 *
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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */

#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/moduleparam.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm/cacheflush.h>
#include "skl.h"
#include "skl-topology.h"
#include "skl-sst-ipc.h"
#include "skl-fwlog.h"
#include "skl-probe.h"

#define HDA_MONO 1
#define HDA_STEREO 2
#define HDA_QUAD 4
#define HDA_8_CH 8

/* FIFO size in ms for Deepbuffer */
static int dpb_fifo_ms = 30;
module_param(dpb_fifo_ms, int, 0644);

static struct snd_pcm_hardware azx_pcm_hw = {
	.info =			(SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_BLOCK_TRANSFER |
				 SNDRV_PCM_INFO_MMAP_VALID |
				 SNDRV_PCM_INFO_PAUSE |
				 SNDRV_PCM_INFO_SYNC_START |
				 SNDRV_PCM_INFO_HAS_WALL_CLOCK | /* legacy */
				 SNDRV_PCM_INFO_HAS_LINK_ATIME |
				 SNDRV_PCM_INFO_NO_PERIOD_WAKEUP),
	.formats =		SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_FLOAT_LE |
				SNDRV_PCM_FMTBIT_S24_3LE |
				SNDRV_PCM_FMTBIT_S32_LE,
	.rates =		SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT,
	.rate_min =		8000,
	.rate_max =		192000,
	.channels_min =		1,
	.channels_max =		HDA_QUAD,
	.buffer_bytes_max =	AZX_MAX_BUF_SIZE,
	.period_bytes_min =	128,
	.period_bytes_max =	AZX_MAX_BUF_SIZE / 2,
	.periods_min =		2,
	.periods_max =		AZX_MAX_FRAG,
	.fifo_size =		0,
};

static inline
struct hdac_ext_stream *get_hdac_ext_stream(struct snd_pcm_substream *substream)
{
	return substream->runtime->private_data;
}

static struct hdac_ext_bus *get_bus_ctx(struct snd_pcm_substream *substream)
{
	struct hdac_ext_stream *stream = get_hdac_ext_stream(substream);
	struct hdac_stream *hstream = hdac_stream(stream);
	struct hdac_bus *bus = hstream->bus;

	return hbus_to_ebus(bus);
}

static int skl_substream_alloc_pages(struct hdac_ext_bus *ebus,
				 struct snd_pcm_substream *substream,
				 size_t size)
{
	struct hdac_ext_stream *stream = get_hdac_ext_stream(substream);
	int ret;

	hdac_stream(stream)->bufsize = 0;
	hdac_stream(stream)->period_bytes = 0;
	hdac_stream(stream)->format_val = 0;

	ret = snd_pcm_lib_malloc_pages(substream, size);
	if (ret < 0)
		return ret;
	ebus->bus.io_ops->mark_pages_uc(snd_pcm_get_dma_buf(substream), true);

	return ret;
}

static int skl_substream_free_pages(struct hdac_bus *bus,
				struct snd_pcm_substream *substream)
{
	bus->io_ops->mark_pages_uc(snd_pcm_get_dma_buf(substream), false);

	return snd_pcm_lib_free_pages(substream);
}

static void skl_set_pcm_constrains(struct hdac_ext_bus *ebus,
				 struct snd_pcm_runtime *runtime)
{
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	/* avoid wrap-around with wall-clock */
	snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_BUFFER_TIME,
				     20, 178000000);
}

static enum hdac_ext_stream_type skl_get_host_stream_type(struct hdac_ext_bus *ebus)
{
	if (ebus->ppcap)
		return HDAC_EXT_STREAM_TYPE_HOST;
	else
		return HDAC_EXT_STREAM_TYPE_COUPLED;
}

static unsigned int rates[] = {
	8000,
	11025,
	12000,
	16000,
	22050,
	24000,
	32000,
	44100,
	48000,
	64000,
	88200,
	96000,
	128000,
	176400,
	192000,
};

static struct snd_pcm_hw_constraint_list hw_rates = {
	.count = ARRAY_SIZE(rates),
	.list = rates,
	.mask = 0,
};

static int skl_pcm_open(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct hdac_ext_stream *stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct skl_dma_params *dma_params;

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);

	stream = snd_hdac_ext_stream_assign(ebus, substream,
					skl_get_host_stream_type(ebus));
	if (stream == NULL)
		return -EBUSY;

	skl_set_pcm_constrains(ebus, runtime);

	snd_pcm_hw_constraint_list(runtime, 0,
					SNDRV_PCM_HW_PARAM_RATE,
					&hw_rates);

	/*
	 * disable WALLCLOCK timestamps for capture streams
	 * until we figure out how to handle digital inputs
	 */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		runtime->hw.info &= ~SNDRV_PCM_INFO_HAS_WALL_CLOCK; /* legacy */
		runtime->hw.info &= ~SNDRV_PCM_INFO_HAS_LINK_ATIME;
	}

	runtime->private_data = stream;

	dma_params = kzalloc(sizeof(*dma_params), GFP_KERNEL);
	if (!dma_params)
		return -ENOMEM;

	dma_params->stream_tag = hdac_stream(stream)->stream_tag;
	snd_soc_dai_set_dma_data(dai, substream, dma_params);

	dev_dbg(dai->dev, "stream tag set in dma params=%d\n",
				 dma_params->stream_tag);
	snd_pcm_set_sync(substream);

	skl_tplg_update_d0i3_stream_count(dai, true);

	return 0;
}

static int skl_get_format(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct skl_dma_params *dma_params;
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	int format_val = 0;

	if (ebus->ppcap) {
		struct snd_pcm_runtime *runtime = substream->runtime;

		format_val = snd_hdac_calc_stream_format(runtime->rate,
						runtime->channels,
						runtime->format,
						32, 0);
	} else {
		struct snd_soc_dai *codec_dai = rtd->codec_dai;

		dma_params = snd_soc_dai_get_dma_data(codec_dai, substream);
		if (dma_params)
			format_val = dma_params->format;
	}

	return format_val;
}

static int skl_pcm_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct hdac_ext_stream *stream = get_hdac_ext_stream(substream);
	unsigned int format_val;
	int err;

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);

	format_val = skl_get_format(substream, dai);
	dev_dbg(dai->dev, "stream_tag=%d formatvalue=%d\n",
				hdac_stream(stream)->stream_tag, format_val);
	snd_hdac_stream_reset(hdac_stream(stream));

	err = snd_hdac_stream_set_params(hdac_stream(stream), format_val);
	if (err < 0)
		return err;

	err = snd_hdac_stream_setup(hdac_stream(stream));
	if (err < 0)
		return err;

	hdac_stream(stream)->prepared = 1;

	return err;
}

static void skl_set_cpr_gtw_dma_fifo_size(struct snd_soc_dai *dai,
		struct skl_module_cfg *m_cfg)
{
	if (!strncmp(dai->name, "Deepbuffer Pin", 14))
		m_cfg->dma_buf_dur_ms = dpb_fifo_ms;
	else
		m_cfg->dma_buf_dur_ms = 2;

	dev_dbg(dai->dev, "Setting copier FIFO size =%d for Dai %s\n",
			m_cfg->dma_buf_dur_ms, dai->name);
}

static int skl_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct hdac_ext_stream *stream = get_hdac_ext_stream(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct skl_pipe_params p_params = {0};
	struct skl_module_cfg *m_cfg;
	int ret, dma_id;

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);
	ret = skl_substream_alloc_pages(ebus, substream,
					  params_buffer_bytes(params));
	if (ret < 0)
		return ret;

	dev_dbg(dai->dev, "format_val, rate=%d, ch=%d, format=%d\n",
			runtime->rate, runtime->channels, runtime->format);

	dma_id = hdac_stream(stream)->stream_tag - 1;
	dev_dbg(dai->dev, "dma_id=%d\n", dma_id);

	p_params.s_fmt = snd_pcm_format_width(params_format(params));
	p_params.ch = params_channels(params);
	p_params.s_freq = params_rate(params);
	p_params.host_dma_id = dma_id;
	p_params.stream = substream->stream;

	m_cfg = skl_tplg_fe_get_cpr_module(dai, p_params.stream);
	if (m_cfg) {
		skl_set_cpr_gtw_dma_fifo_size(dai, m_cfg);
		skl_tplg_update_pipe_params(dai->dev,
					m_cfg,
					&p_params,
					params_format(params));
	}

	return 0;
}

static void skl_pcm_close(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct hdac_ext_stream *stream = get_hdac_ext_stream(substream);
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct skl_dma_params *dma_params = NULL;

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);

	snd_hdac_ext_stream_release(stream, skl_get_host_stream_type(ebus));

	dma_params = snd_soc_dai_get_dma_data(dai, substream);
	/*
	 * now we should set this to NULL as we are freeing by the
	 * dma_params
	 */
	snd_soc_dai_set_dma_data(dai, substream, NULL);

	skl_tplg_update_d0i3_stream_count(dai, false);

	kfree(dma_params);
}

static int skl_pcm_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct hdac_ext_stream *stream = get_hdac_ext_stream(substream);

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);

	snd_hdac_stream_cleanup(hdac_stream(stream));
	hdac_stream(stream)->prepared = 0;

	return skl_substream_free_pages(ebus_to_hbus(ebus), substream);
}

static int skl_be_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct skl_pipe_params p_params = {0};

	p_params.s_fmt = snd_pcm_format_width(params_format(params));
	p_params.ch = params_channels(params);
	p_params.s_freq = params_rate(params);
	p_params.stream = substream->stream;

	return skl_tplg_be_update_params(dai, &p_params);
}

static int skl_decoupled_trigger(struct snd_pcm_substream *substream,
		int cmd)
{
	struct hdac_ext_bus *ebus = get_bus_ctx(substream);
	struct hdac_bus *bus = ebus_to_hbus(ebus);
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct hdac_ext_stream *stream;
	int start;
	unsigned long cookie;
	struct hdac_stream *hstr;

	dev_dbg(bus->dev, "In %s cmd=%d streamname=%s\n", __func__, cmd, cpu_dai->name);

	stream = get_hdac_ext_stream(substream);
	hstr = hdac_stream(stream);

	if (!hstr->prepared)
		return -EPIPE;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		start = 1;
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		start = 0;
		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&bus->reg_lock, cookie);

	if (start) {
		snd_hdac_stream_start(hdac_stream(stream), true);
		snd_hdac_stream_timecounter_init(hstr, 0);
	} else {
		snd_hdac_stream_stop(hdac_stream(stream));
	}

	spin_unlock_irqrestore(&bus->reg_lock, cookie);

	return 0;
}

static int skl_pcm_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct skl *skl = get_skl_ctx(dai->dev);
	struct skl_sst *ctx = skl->skl_sst;
	struct skl_module_cfg *mconfig;
	struct hdac_ext_bus *ebus = get_bus_ctx(substream);
	struct hdac_ext_stream *stream = get_hdac_ext_stream(substream);
	int ret;

	mconfig = skl_tplg_fe_get_cpr_module(dai, substream->stream);
	if (!mconfig)
		return -EIO;

	dev_dbg(dai->dev, "In %s cmd=%d\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
		skl_pcm_prepare(substream, dai);
		/*
		 * enable DMA Resume enable bit for the stream, set the dpib
		 * & lpib position to resune before starting the DMA
		 */
		snd_hdac_ext_stream_drsm_enable(ebus, true,
					hdac_stream(stream)->index);
		snd_hdac_ext_stream_set_dpibr(ebus, stream, stream->dpib);
		snd_hdac_ext_stream_set_lpib(stream, stream->lpib);

	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/*
		 * Start HOST DMA and Start FE Pipe.This is to make sure that
		 * there are no underrun/overrun in the case when the FE
		 * pipeline is started but there is a delay in starting the
		 * DMA channel on the host.
		 */
		snd_hdac_ext_stream_decouple(ebus, stream, true);
		ret = skl_decoupled_trigger(substream, cmd);
		if (ret < 0)
			return ret;
		return skl_run_pipe(ctx, mconfig->pipe);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		/*
		 * Stop FE Pipe first and stop DMA. This is to make sure that
		 * there are no underrun/overrun in the case if there is a delay
		 * between the two operations.
		 */
		ret = skl_stop_pipe(ctx, mconfig->pipe);
		if (ret < 0)
			return ret;

		ret = skl_decoupled_trigger(substream, cmd);
		if (cmd == SNDRV_PCM_TRIGGER_SUSPEND) {
			/* save the dpib and lpib positions */
			stream->dpib = readl(ebus->bus.remap_addr +
					AZX_REG_VS_SDXDPIB_XBASE +
					(AZX_REG_VS_SDXDPIB_XINTERVAL *
					hdac_stream(stream)->index));

			stream->lpib = snd_hdac_stream_get_pos_lpib(
							hdac_stream(stream));
			snd_hdac_ext_stream_decouple(ebus, stream, false);
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int skl_link_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct hdac_ext_stream *link_dev;
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct hdac_ext_dma_params *dma_params;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct skl_pipe_params p_params = {0};

	link_dev = snd_hdac_ext_stream_assign(ebus, substream,
					HDAC_EXT_STREAM_TYPE_LINK);
	if (!link_dev)
		return -EBUSY;

	snd_soc_dai_set_dma_data(dai, substream, (void *)link_dev);

	/* set the stream tag in the codec dai dma params  */
	dma_params = (struct hdac_ext_dma_params *)
			snd_soc_dai_get_dma_data(codec_dai, substream);
	if (dma_params)
		dma_params->stream_tag =  hdac_stream(link_dev)->stream_tag;

	p_params.s_fmt = snd_pcm_format_width(params_format(params));
	p_params.ch = params_channels(params);
	p_params.s_freq = params_rate(params);
	p_params.stream = substream->stream;
	p_params.link_dma_id = hdac_stream(link_dev)->stream_tag - 1;

	return skl_tplg_be_update_params(dai, &p_params);
}

static int skl_link_pcm_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct hdac_ext_stream *link_dev =
			snd_soc_dai_get_dma_data(dai, substream);
	unsigned int format_val = 0;
	struct skl_dma_params *dma_params;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct hdac_ext_link *link;

	if (link_dev->link_prepared) {
		dev_dbg(dai->dev, "already stream is prepared - returning\n");
		return 0;
	}

	dma_params  = (struct skl_dma_params *)
			snd_soc_dai_get_dma_data(codec_dai, substream);
	if (dma_params)
		format_val = dma_params->format;
	dev_dbg(dai->dev, "stream_tag=%d formatvalue=%d codec_dai_name=%s\n",
			hdac_stream(link_dev)->stream_tag, format_val, codec_dai->name);

	snd_hdac_ext_link_stream_reset(link_dev);

	snd_hdac_ext_link_stream_setup(link_dev, format_val);

	link = snd_hdac_ext_bus_get_link(ebus, rtd->codec->component.name);
	if (!link)
		return -EINVAL;

	snd_hdac_ext_link_set_stream_id(link, hdac_stream(link_dev)->stream_tag);
	link_dev->link_prepared = 1;

	return 0;
}

static int skl_link_pcm_trigger(struct snd_pcm_substream *substream,
	int cmd, struct snd_soc_dai *dai)
{
	struct hdac_ext_stream *link_dev =
				snd_soc_dai_get_dma_data(dai, substream);

	dev_dbg(dai->dev, "In %s cmd=%d\n", __func__, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		snd_hdac_ext_link_stream_start(link_dev);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		snd_hdac_ext_link_stream_clear(link_dev);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int skl_link_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct hdac_ext_stream *link_dev =
				snd_soc_dai_get_dma_data(dai, substream);
	struct hdac_ext_link *link;

	if (!link_dev)
		return 0;

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);

	link_dev->link_prepared = 0;

	link = snd_hdac_ext_bus_get_link(ebus, rtd->codec->component.name);
	if (!link)
		return -EINVAL;

	snd_soc_dai_set_dma_data(dai, substream, NULL);
	snd_hdac_ext_link_clear_stream_id(link, hdac_stream(link_dev)->stream_tag);
	snd_hdac_ext_stream_release(link_dev, HDAC_EXT_STREAM_TYPE_LINK);
	return 0;
}

static int skl_get_compr_core(struct snd_compr_stream *stream)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_dai *dai = rtd->cpu_dai;

	if (!strcmp(dai->name, "TraceBuffer0 Pin"))
		return 0;
	else if (!strcmp(dai->name, "TraceBuffer1 Pin"))
		return 1;
	else if (!strcmp(dai->name, "TraceBuffer2 Pin"))
		return 2;
	else if (!strcmp(dai->name, "TraceBuffer3 Pin"))
		return 3;
	else
		return INT_MIN;
}

static int skl_is_logging_core(int core)
{
	/*
	 * disabled on core 2 and 3 for cnl for now.
	 * will be added when LogState struct is fixed in fw.
	 */
	if (core == 0 || core == 1)
		return 1;
	else
		return 0;
}

static struct skl_sst *skl_get_sst_compr(struct snd_compr_stream *stream)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct skl *skl = ebus_to_skl(ebus);
	struct skl_sst *sst = skl->skl_sst;

	return sst;
}

static int skl_trace_compr_open(struct snd_compr_stream *substream,
		struct snd_soc_dai *dai)
{
	struct skl_sst *skl_sst = skl_get_sst_compr(substream);
	struct sst_dsp *sst = skl_sst->dsp;
	int core = skl_get_compr_core(substream);
	int ret;

	if (!skl_is_logging_core(core))
		return -EINVAL;

	ret = pm_runtime_get_sync(skl_sst->dev);
	if (ret < 0) {
		dev_err(skl_sst->dev,"trace open:pm_runtime_get failed\n");
		return ret;
	}


	if (core != SKL_DSP_CORE0_ID) {
		ret = skl_dsp_get_core(sst, core);
		if (ret) {
			dev_err(skl_sst->dev,
					"trace open:get core%d failed\n", core);
			pm_runtime_put_sync(skl_sst->dev);
		}
	}


	return ret;
}
static int skl_trace_compr_set_params(struct snd_compr_stream *stream,
					struct snd_compr_params *params,
						struct snd_soc_dai *cpu_dai)
{
	int ret;
	struct skl_sst *skl_sst = skl_get_sst_compr(stream);
	struct sst_dsp *sst = skl_sst->dsp;
	struct sst_generic_ipc *ipc = &skl_sst->ipc;
	int size = params->buffer.fragment_size * params->buffer.fragments;
	int core = skl_get_compr_core(stream);

	if (!skl_is_logging_core(core))
		return -EINVAL;

	size = size / sizeof(u32);
	if (size & (size - 1)) {
		dev_err(sst->dev, "Buffer size must be a power of 2\n");
		return -EINVAL;
	}

	ret = skl_dsp_init_log_buffer(sst, size, core, stream);
	if (ret) {
		dev_err(sst->dev, "set params failed for dsp %d\n", core);
		return ret;
	}

	skl_dsp_get_log_buff(sst, core);
	sst->trace_wind.flags |= BIT(core);
	ret = skl_dsp_enable_logging(ipc, core, 1);
	if (ret < 0) {
		dev_err(sst->dev, "enable logs failed for dsp %d\n", core);
		sst->trace_wind.flags &= ~BIT(core);
		skl_dsp_put_log_buff(sst, core);
		return ret;
	}
	return 0;
}

static int skl_trace_compr_tstamp(struct snd_compr_stream *stream,
					struct snd_compr_tstamp *tstamp,
						struct snd_soc_dai *cpu_dai)
{
	struct skl_sst *skl_sst = skl_get_sst_compr(stream);
	struct sst_dsp *sst = skl_sst->dsp;
	int core = skl_get_compr_core(stream);

	if (!skl_is_logging_core(core))
		return -EINVAL;

	tstamp->copied_total = skl_dsp_log_avail(sst, core);
	return 0;
}

static int skl_trace_compr_copy(struct snd_compr_stream *stream,
				char __user *dest, size_t count)
{
	struct skl_sst *skl_sst = skl_get_sst_compr(stream);
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct sst_dsp *sst = skl_sst->dsp;
	int core = skl_get_compr_core(stream);

	if (skl_is_logging_core(core))
		return skl_dsp_copy_log_user(sst, core, dest, count);
	else
		return skl_probe_compr_copy(stream, dest, count, cpu_dai);
}

static int skl_trace_compr_free(struct snd_compr_stream *stream,
						struct snd_soc_dai *cpu_dai)
{
	struct skl_sst *skl_sst = skl_get_sst_compr(stream);
	struct sst_dsp *sst = skl_sst->dsp;
	struct sst_generic_ipc *ipc = &skl_sst->ipc;
	int core = skl_get_compr_core(stream);
	int is_enabled = sst->trace_wind.flags & BIT(core);

	if (!skl_is_logging_core(core))
		return -EINVAL;
	if (is_enabled) {
		sst->trace_wind.flags &= ~BIT(core);
		skl_dsp_enable_logging(ipc, core, 0);
		skl_dsp_put_log_buff(sst, core);
		skl_dsp_done_log_buffer(sst, core);
	}
	if (core != SKL_DSP_CORE0_ID)
		skl_dsp_put_core(sst, core);
	pm_runtime_put_sync(ipc->dev);
	return 0;
}

static struct snd_compr_ops skl_platform_compr_ops = {
	.copy = skl_trace_compr_copy,
};

static struct snd_soc_cdai_ops skl_probe_compr_ops = {
	.startup = skl_probe_compr_open,
	.shutdown = skl_probe_compr_close,
	.trigger = skl_probe_compr_trigger,
	.ack = skl_probe_compr_ack,
	.pointer = skl_probe_compr_tstamp,
	.set_params = skl_probe_compr_set_params,
};

static struct snd_soc_cdai_ops skl_trace_compr_ops = {
	.startup = skl_trace_compr_open,
	.shutdown = skl_trace_compr_free,
	.pointer = skl_trace_compr_tstamp,
	.set_params = skl_trace_compr_set_params,
};

static struct snd_soc_dai_ops skl_pcm_dai_ops = {
	.startup = skl_pcm_open,
	.shutdown = skl_pcm_close,
	.prepare = skl_pcm_prepare,
	.hw_params = skl_pcm_hw_params,
	.hw_free = skl_pcm_hw_free,
	.trigger = skl_pcm_trigger,
};

static struct snd_soc_dai_ops skl_dmic_dai_ops = {
	.hw_params = skl_be_hw_params,
};

static struct snd_soc_dai_ops skl_be_ssp_dai_ops = {
	.hw_params = skl_be_hw_params,
};

static struct snd_soc_dai_ops skl_link_dai_ops = {
	.prepare = skl_link_pcm_prepare,
	.hw_params = skl_link_hw_params,
	.hw_free = skl_link_hw_free,
	.trigger = skl_link_pcm_trigger,
};

static int skl_dsp_cb_event(struct skl_dsp_notify_params *params)
{
	struct snd_soc_platform *soc_platform = params->skl_sst->platform;
	struct snd_soc_card *card;
	struct snd_kcontrol *kcontrol;

	if (!soc_platform) {
		dev_err(params->skl_sst->dev,
			"%s: Platform not found\n", __func__);
		return -EINVAL;
	}

	card = soc_platform->component.card;
	kcontrol = snd_soc_card_get_kcontrol(card,
				"hwd_in hwd 0 notif params"); /*control name of WoV notification*/
	if (!kcontrol) {
			dev_err(params->skl_sst->dev,
				"hwd notification control not found\n");
			return -EINVAL;
		}
	snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE, &kcontrol->id);
	return 0;
}

struct skl_dsp_notify_ops cb_ops = {
	.notify_cb = skl_dsp_cb_event,
};

static struct snd_soc_dai_driver skl_platform_dai[] = {
{
	.name = "TraceBuffer0 Pin",
	.compress_dai = 1,
	.cops = &skl_trace_compr_ops,
	.capture = {
		.stream_name = "TraceBuffer0 Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_MONO,
	},
},
{
	.name = "TraceBuffer1 Pin",
	.compress_dai = 1,
	.cops = &skl_trace_compr_ops,
	.capture = {
		.stream_name = "TraceBuffer1 Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_MONO,
	},
},
{
	.name = "TraceBuffer2 Pin",
	.compress_dai = 1,
	.cops = &skl_trace_compr_ops,
	.capture = {
		.stream_name = "TraceBuffer2 Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_MONO,
	},
},
{
	.name = "TraceBuffer3 Pin",
	.compress_dai = 1,
	.cops = &skl_trace_compr_ops,
	.capture = {
		.stream_name = "TraceBuffer3 Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_MONO,
	},
},
{
	.name = "System Pin",
	.ops = &skl_pcm_dai_ops,
	.playback = {
		.stream_name = "System Playback",
		.channels_min = HDA_MONO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE |
				SNDRV_PCM_FMTBIT_FLOAT_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "System Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_32000 |
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE |
				SNDRV_PCM_FMTBIT_S32_LE,
	},
},
{
	.name = "Reference Pin",
	.ops = &skl_pcm_dai_ops,
	.capture = {
		.stream_name = "Reference Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_QUAD,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = "Deepbuffer Pin",
	.ops = &skl_pcm_dai_ops,
	.playback = {
		.stream_name = "Deepbuffer Playback",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = "Compress Probe0 Pin",
	.compress_dai = 1,
	.cops = &skl_probe_compr_ops,
	.playback = {
		.stream_name = "Probe Playback",
		.channels_min = HDA_MONO,
	},
},
{
	.name = "Compress Probe1 Pin",
	.compress_dai = 1,
	.cops = &skl_probe_compr_ops,
	.capture = {
			.stream_name = "Probe Capture",
			.channels_min = HDA_MONO,
	},
},
{
	.name = "LowLatency Pin",
	.ops = &skl_pcm_dai_ops,
	.playback = {
		.stream_name = "Low Latency Playback",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = "HDMI Pin",
	.ops = &skl_pcm_dai_ops,
	.playback = {
		.stream_name = "HDMI Playback",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_32000 |	SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
},
{
	.name = "DMIC Pin",
	.ops = &skl_pcm_dai_ops,
	.capture = {
		.stream_name = "DMIC Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_QUAD,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = "System Pin 2",
	.ops = &skl_pcm_dai_ops,
	.playback = {
		.stream_name = "System Playback 2",
		.channels_min = HDA_MONO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE |
			SNDRV_PCM_FMTBIT_FLOAT_LE,
	},
},
{
	.name = "System Pin 3",
	.ops = &skl_pcm_dai_ops,
	.playback = {
		.stream_name = "System Playback 3",
		.channels_min = HDA_MONO,
		.channels_max = HDA_QUAD,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE |
			SNDRV_PCM_FMTBIT_FLOAT_LE,
	},
},
{
	.name = "System Pin 4",
	.ops = &skl_pcm_dai_ops,
	.playback = {
		.stream_name = "System Playback 4",
		.channels_min = HDA_MONO,
		.channels_max = HDA_QUAD,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE |
			SNDRV_PCM_FMTBIT_FLOAT_LE,
	},
},
{
	.name = "System Pin 5",
	.ops = &skl_pcm_dai_ops,
	.capture = {
		.stream_name = "PT Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_8_CH,
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_64000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,

	},
},

/* BE CPU  Dais */
{
	.name = "SSP0 Pin",
	.ops = &skl_be_ssp_dai_ops,
	.playback = {
	.stream_name = "ssp0 Tx",
		.channels_min = HDA_MONO,
		.channels_max = HDA_8_CH,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "ssp0 Rx",
		.channels_min = HDA_MONO,
		.channels_max = HDA_8_CH,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_64000 ,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
},

{
	.name = "SSP1 Pin",
	.ops = &skl_be_ssp_dai_ops,
	.playback = {
		.stream_name = "ssp1 Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_QUAD,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp1 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_QUAD,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "SSP2 Pin",
	.ops = &skl_be_ssp_dai_ops,
	.playback = {
		.stream_name = "ssp2 Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_44100,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp2 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_44100,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "SSP3 Pin",
	.ops = &skl_be_ssp_dai_ops,
	.playback = {
		.stream_name = "ssp3 Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp3 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "SSP4 Pin",
	.ops = &skl_be_ssp_dai_ops,
	.playback = {
		.stream_name = "ssp4 Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_44100,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp4 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_44100,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "SSP5 Pin",
	.ops = &skl_be_ssp_dai_ops,
	.playback = {
		.stream_name = "ssp5 Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp5 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "iDisp Pin",
	.ops = &skl_link_dai_ops,
	.playback = {
		.stream_name = "iDisp Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "DMIC01 Pin",
	.ops = &skl_dmic_dai_ops,
	.capture = {
		.stream_name = "DMIC01 Rx",
		.channels_min = HDA_MONO,
		.channels_max = HDA_QUAD,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = "DMIC23 Pin",
	.ops = &skl_dmic_dai_ops,
	.capture = {
		.stream_name = "DMIC23 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = "HD-Codec Pin",
	.ops = &skl_link_dai_ops,
	.playback = {
		.stream_name = "HD-Codec Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "HD-Codec Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
};

static int skl_platform_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	dev_dbg(rtd->cpu_dai->dev, "In %s:%s\n", __func__,
					dai_link->cpu_dai_name);

	runtime = substream->runtime;
	snd_soc_set_runtime_hwparams(substream, &azx_pcm_hw);

	return 0;
}

static int skl_coupled_trigger(struct snd_pcm_substream *substream,
					int cmd)
{
	struct hdac_ext_bus *ebus = get_bus_ctx(substream);
	struct hdac_bus *bus = ebus_to_hbus(ebus);
	struct hdac_ext_stream *stream;
	struct snd_pcm_substream *s;
	bool start;
	int sbits = 0;
	unsigned long cookie;
	struct hdac_stream *hstr;

	stream = get_hdac_ext_stream(substream);
	hstr = hdac_stream(stream);

	dev_dbg(bus->dev, "In %s cmd=%d\n", __func__, cmd);

	if (!hstr->prepared)
		return -EPIPE;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		start = true;
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		start = false;
		break;

	default:
		return -EINVAL;
	}

	snd_pcm_group_for_each_entry(s, substream) {
		if (s->pcm->card != substream->pcm->card)
			continue;
		stream = get_hdac_ext_stream(s);
		sbits |= 1 << hdac_stream(stream)->index;
		snd_pcm_trigger_done(s, substream);
	}

	spin_lock_irqsave(&bus->reg_lock, cookie);

	/* first, set SYNC bits of corresponding streams */
	snd_hdac_stream_sync_trigger(hstr, true, sbits, AZX_REG_SSYNC);

	snd_pcm_group_for_each_entry(s, substream) {
		if (s->pcm->card != substream->pcm->card)
			continue;
		stream = get_hdac_ext_stream(s);
		if (start)
			snd_hdac_stream_start(hdac_stream(stream), true);
		else
			snd_hdac_stream_stop(hdac_stream(stream));
	}
	spin_unlock_irqrestore(&bus->reg_lock, cookie);

	snd_hdac_stream_sync(hstr, start, sbits);

	spin_lock_irqsave(&bus->reg_lock, cookie);

	/* reset SYNC bits */
	snd_hdac_stream_sync_trigger(hstr, false, sbits, AZX_REG_SSYNC);
	if (start)
		snd_hdac_stream_timecounter_init(hstr, sbits);
	spin_unlock_irqrestore(&bus->reg_lock, cookie);

	return 0;
}

static int skl_platform_pcm_trigger(struct snd_pcm_substream *substream,
					int cmd)
{
	struct hdac_ext_bus *ebus = get_bus_ctx(substream);

	if (!ebus->ppcap)
		return skl_coupled_trigger(substream, cmd);

	return 0;
}

/* calculate runtime delay from LPIB */
static int skl_get_delay_from_lpib(struct hdac_ext_bus *ebus,
				struct hdac_ext_stream *sstream,
				unsigned int pos)
{
	struct hdac_bus *bus = ebus_to_hbus(ebus);
	struct hdac_stream *hstream = hdac_stream(sstream);
	struct snd_pcm_substream *substream = hstream->substream;
	int stream = substream->stream;
	unsigned int lpib_pos = snd_hdac_stream_get_pos_lpib(hstream);
	int delay;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		delay = pos - lpib_pos;
	else
		delay = lpib_pos - pos;

	if (delay < 0) {
		if (delay >= hstream->delay_negative_threshold)
			delay = 0;
		else
			delay += hstream->bufsize;
	}

	delay = (hstream->bufsize == delay) ? 0 : delay;

	if (delay >= hstream->period_bytes) {
		dev_info(bus->dev,
			 "Unstable LPIB (%d >= %d); disabling LPIB delay counting\n",
			 delay, hstream->period_bytes);
		delay = 0;
	}

	return bytes_to_frames(substream->runtime, delay);
}

static unsigned int skl_get_position(struct hdac_ext_stream *hstream,
					int codec_delay)
{
	struct hdac_stream *hstr = hdac_stream(hstream);
	struct snd_pcm_substream *substream = hstr->substream;
	struct hdac_ext_bus *ebus;
	unsigned int pos;
	int delay;

	/* use the position buffer as default */
	pos = snd_hdac_stream_get_pos_posbuf(hdac_stream(hstream));

	if (pos >= hdac_stream(hstream)->bufsize)
		pos = 0;

	if (substream->runtime) {
		ebus = get_bus_ctx(substream);
		delay = skl_get_delay_from_lpib(ebus, hstream, pos)
						 + codec_delay;
		substream->runtime->delay += delay;
	}

	return pos;
}

static snd_pcm_uframes_t skl_platform_pcm_pointer
			(struct snd_pcm_substream *substream)
{
	struct hdac_ext_stream *hstream = get_hdac_ext_stream(substream);
	struct snd_dma_buffer *dmab;
	int ret;

	ret = bytes_to_frames(substream->runtime,
			skl_get_position(hstream, 0));

	dmab = snd_pcm_get_dma_buf(substream);
	clflush_cache_range(dmab->area, dmab->bytes);

	return ret;
}

static u64 skl_adjust_codec_delay(struct snd_pcm_substream *substream,
				u64 nsec)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	u64 codec_frames, codec_nsecs;

	if (!codec_dai->driver->ops->delay)
		return nsec;

	codec_frames = codec_dai->driver->ops->delay(substream, codec_dai);
	codec_nsecs = div_u64(codec_frames * 1000000000LL,
			      substream->runtime->rate);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return nsec + codec_nsecs;

	return (nsec > codec_nsecs) ? nsec - codec_nsecs : 0;
}

static int skl_get_time_info(struct snd_pcm_substream *substream,
			struct timespec *system_ts, struct timespec *audio_ts,
			struct snd_pcm_audio_tstamp_config *audio_tstamp_config,
			struct snd_pcm_audio_tstamp_report *audio_tstamp_report)
{
	struct hdac_ext_stream *sstream = get_hdac_ext_stream(substream);
	struct hdac_stream *hstr = hdac_stream(sstream);
	u64 nsec;

	if ((substream->runtime->hw.info & SNDRV_PCM_INFO_HAS_LINK_ATIME) &&
		(audio_tstamp_config->type_requested == SNDRV_PCM_AUDIO_TSTAMP_TYPE_LINK)) {

		snd_pcm_gettime(substream->runtime, system_ts);

		nsec = timecounter_read(&hstr->tc);
		nsec = div_u64(nsec, 3); /* can be optimized */
		if (audio_tstamp_config->report_delay)
			nsec = skl_adjust_codec_delay(substream, nsec);

		*audio_ts = ns_to_timespec(nsec);

		audio_tstamp_report->actual_type = SNDRV_PCM_AUDIO_TSTAMP_TYPE_LINK;
		audio_tstamp_report->accuracy_report = 1; /* rest of struct is valid */
		audio_tstamp_report->accuracy = 42; /* 24MHzWallClk == 42ns resolution */

	} else {
		audio_tstamp_report->actual_type = SNDRV_PCM_AUDIO_TSTAMP_TYPE_DEFAULT;
	}

	return 0;
}

static struct snd_pcm_ops skl_platform_ops = {
	.open = skl_platform_open,
	.ioctl = snd_pcm_lib_ioctl,
	.trigger = skl_platform_pcm_trigger,
	.pointer = skl_platform_pcm_pointer,
	.get_time_info =  skl_get_time_info,
	.mmap = snd_pcm_lib_default_mmap,
	.page = snd_pcm_sgbuf_ops_page,
};

static void skl_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

#define MAX_PREALLOC_SIZE	(32 * 1024 * 1024)

static int skl_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct hdac_ext_bus *ebus = dev_get_drvdata(dai->dev);
	struct snd_pcm *pcm = rtd->pcm;
	unsigned int size;
	int retval = 0;
	struct skl *skl = ebus_to_skl(ebus);

	if (dai->driver->playback.channels_min ||
		dai->driver->capture.channels_min) {
		/* buffer pre-allocation */
		size = CONFIG_SND_HDA_PREALLOC_SIZE * 1024;
		if (size > MAX_PREALLOC_SIZE)
			size = MAX_PREALLOC_SIZE;
		retval = snd_pcm_lib_preallocate_pages_for_all(pcm,
						SNDRV_DMA_TYPE_DEV_SG,
						snd_dma_pci_data(skl->pci),
						size, MAX_PREALLOC_SIZE);
		if (retval) {
			dev_err(dai->dev, "dma buffer allocationf fail\n");
			return retval;
		}
	}

	return retval;
}

static int skl_get_probe_widget(struct snd_soc_platform *platform,
							struct skl *skl)
{
	struct skl_probe_config *pconfig = &skl->skl_sst->probe_config;
	struct snd_soc_dapm_widget *w;

	list_for_each_entry(w, &platform->component.card->widgets, list) {
		if (is_skl_dsp_widget_type(w) &&
				(strstr(w->name, "probe") != NULL)) {
			pconfig->w = w;

			dev_dbg(platform->dev, "widget type=%d name=%s\n",
							w->id, w->name);
			break;
		}
	}

	pconfig->probe_count = 0;
	pconfig->no_injector = 6;
	pconfig->no_extractor = 8;

	return 0;
}

static int skl_platform_soc_probe(struct snd_soc_platform *platform)
{
	struct hdac_ext_bus *ebus = dev_get_drvdata(platform->dev);
	struct hdac_bus *bus = ebus_to_hbus(ebus);
	int ret = 0;
	struct skl *skl = ebus_to_skl(ebus);
	struct platform_info *dbg_info;
	if (ebus->ppcap) {
		ret = skl_tplg_init(platform, ebus);
		if (ret < 0) {
			dev_dbg(bus->dev, "error failed while initializing topology\n");
			return ret;
		}

		ret = skl_init_dsp_fw(skl);
		if (ret < 0) {
			dev_dbg(bus->dev, "error failed to register dsp\n");
			goto out_free;
		}
		skl->skl_sst->update_d0i3c = skl_update_d0i3c;
		skl->skl_sst->platform = platform;
		skl->skl_sst->notify_ops = cb_ops;
	}

	skl_get_probe_widget(platform, skl);
	dbg_info = kzalloc(sizeof(struct platform_info), GFP_KERNEL);
	if (!dbg_info)
		return -ENOMEM;

	dbg_info->sram0_base = skl->skl_sst->dsp->addr.sram0_base;
	dbg_info->sram1_base = skl->skl_sst->dsp->addr.sram1_base;
	dbg_info->lpe = skl->skl_sst->dsp->addr.lpe;
	dbg_info->w0_stat_sz = skl->skl_sst->dsp->addr.w0_stat_sz;
	dbg_info->w0_up_sz = skl->skl_sst->dsp->addr.w0_up_sz;

	dbg_info->in_base = skl->skl_sst->dsp->mailbox.in_base;
	dbg_info->in_size = skl->skl_sst->dsp->mailbox.in_size;
	dbg_info->out_base = skl->skl_sst->dsp->mailbox.out_base;
	dbg_info->out_size = skl->skl_sst->dsp->mailbox.out_size;

	skl_update_dsp_debug_info(skl->debugfs, dbg_info);
	kfree(dbg_info);

	return ret;
out_free:
	skl->init_failed = 1;
	return ret;
}

static int skl_platform_soc_remove(struct snd_soc_platform *platform)
{
	struct hdac_ext_bus *ebus = dev_get_drvdata(platform->dev);
	struct skl *skl = ebus_to_skl(ebus);

	if (skl->fw) {
		release_firmware(skl->fw);
		skl->fw = NULL;
	}
	return 0;
}
static struct snd_soc_platform_driver skl_platform_drv  = {
	.probe		= skl_platform_soc_probe,
	.remove         = skl_platform_soc_remove,
	.ops		= &skl_platform_ops,
	.compr_ops	= &skl_platform_compr_ops,
	.pcm_new	= skl_pcm_new,
	.pcm_free	= skl_pcm_free,
};

static const struct snd_soc_component_driver skl_component = {
	.name           = "pcm",
};

int skl_platform_register(struct device *dev)
{
	int ret;
	struct hdac_ext_bus *ebus = dev_get_drvdata(dev);
	struct skl *skl = ebus_to_skl(ebus);

	INIT_LIST_HEAD(&skl->ppl_list);

	ret = snd_soc_register_platform(dev, &skl_platform_drv);
	if (ret) {
		dev_err(dev, "soc platform registration failed %d\n", ret);
		return ret;
	}
	ret = snd_soc_register_component(dev, &skl_component,
				skl_platform_dai,
				ARRAY_SIZE(skl_platform_dai));
	if (ret) {
		dev_err(dev, "soc component registration failed %d\n", ret);
		snd_soc_unregister_platform(dev);
	}

	return ret;

}

int skl_platform_unregister(struct device *dev)
{
	snd_soc_unregister_component(dev);
	snd_soc_unregister_platform(dev);
	return 0;
}
