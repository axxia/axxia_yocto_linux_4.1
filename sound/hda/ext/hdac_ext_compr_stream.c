/*
 *  hdac-ext-stream.c - HD-audio extended stream operations.
 *
 *  Copyright (C) 2015 Intel Corp
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
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/hda_register.h>
#include <sound/hdaudio_ext.h>
#include <uapi/sound/compress_offload.h>
#include <sound/compress_offload.h>
#include <sound/compress_driver.h>


static struct hdac_ext_stream *
hdac_ext_host_stream_compr_assign(struct hdac_ext_bus *ebus,
				struct snd_compr_stream *substream)
{
	struct hdac_ext_stream *res = NULL;
	struct hdac_stream *stream = NULL;
	struct hdac_bus *hbus = &ebus->bus;

	if (!ebus->ppcap) {
		dev_err(hbus->dev, "stream type not supported\n");
		return NULL;
	}

	list_for_each_entry(stream, &hbus->stream_list, list) {
		struct hdac_ext_stream *hstream = container_of(stream,
						struct hdac_ext_stream,
						hstream);
		if (stream->direction != substream->direction)
			continue;

		if (!stream->opened) {
			if (!hstream->decoupled)
				snd_hdac_ext_stream_decouple(ebus, hstream, true);
			res = hstream;
			break;
		}
	}
	if (res) {
		spin_lock_irq(&hbus->reg_lock);
		res->hstream.opened = 1;
		res->hstream.running = 0;
		res->hstream.stream = substream;
		spin_unlock_irq(&hbus->reg_lock);
	}
	 pr_err("**PROBE** chosen stream tag = %d, index = %d\n", res->hstream.stream_tag, res->hstream.index);
	return res;
}


struct hdac_ext_stream *snd_hdac_ext_stream_compr_assign(struct hdac_ext_bus *ebus,
					   struct snd_compr_stream *substream,
					   int type)
{
	struct hdac_ext_stream *hstream = NULL;
	struct hdac_stream *stream = NULL;
	struct hdac_bus *hbus = &ebus->bus;

	switch (type) {
	case HDAC_EXT_STREAM_TYPE_HOST:
		return hdac_ext_host_stream_compr_assign(ebus, substream);

	default:
		return NULL;
	}
}
EXPORT_SYMBOL_GPL(snd_hdac_ext_stream_compr_assign);

