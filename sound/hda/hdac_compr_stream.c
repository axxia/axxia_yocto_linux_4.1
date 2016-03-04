/*
 * HD-audio stream operations
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/clocksource.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/hdaudio.h>
#include <sound/hda_register.h>
#include "trace.h"
#include <sound/compress_driver.h>


int snd_hdac_stream_setup_compr_periods(struct hdac_stream *azx_dev)
{
	struct hdac_bus *bus = azx_dev->bus;
	struct snd_compr_stream *substream = azx_dev->stream;
	struct snd_compr_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *dma_buffer_p = runtime->dma_buffer_p;
	__le32 *bdl;
	int i, ofs, periods, period_bytes;
	int pos_adj, pos_align;

	/* reset BDL address */
	snd_hdac_stream_writel(azx_dev, SD_BDLPL, 0);
	snd_hdac_stream_writel(azx_dev, SD_BDLPU, 0);

	period_bytes = azx_dev->period_bytes;
	periods = azx_dev->bufsize / period_bytes;
	/* program the initial BDL entries */
	bdl = (__le32 *)azx_dev->bdl.area;
	pr_err("**PROBE** bdl = %x, azx_dev->bdl.area = %x dma_buffer_p = %x\n",
					bdl, azx_dev->bdl.area, dma_buffer_p);
	ofs = 0;
	azx_dev->frags = 0;

	pos_adj = 0;

	for (i = 0; i < periods; i++) {
		if (i == periods - 1 && pos_adj)
			ofs = setup_bdle(bus, (substream)->runtime->dma_buffer_p,
					 azx_dev, &bdl, ofs,
					 period_bytes - pos_adj, 0);
		else {
			ofs = setup_bdle(bus, (substream)->runtime->dma_buffer_p,
					 azx_dev, &bdl, ofs,
					 period_bytes,
					 !azx_dev->no_period_wakeup);
		}
		if (ofs < 0)
			goto error;
	}
	return 0;

 error:
	dev_err(bus->dev, "Too many BDL entries: buffer=%d, period=%d\n",
		azx_dev->bufsize, period_bytes);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_hdac_stream_setup_compr_periods);

/* snd_hdac_stream_set_params - set stream parameters
 * @azx_dev: HD-audio core stream for which parameters are to be set
 * @format_val: format value parameter
 *
 * Setup the HD-audio core stream parameters from substream of the stream
 * and passed format value
 */

int snd_hdac_stream_set_compr_params(struct hdac_stream *azx_dev,
				 unsigned int format_val)
{

	unsigned int bufsize, period_bytes;
	struct snd_compr_stream *substream = azx_dev->stream;
	struct snd_compr_runtime *runtime;
	int err;

	if (!substream)
		return -EINVAL;
	runtime = substream->runtime;

	bufsize = runtime->buffer_size;
	period_bytes = runtime->fragment_size;

	if (bufsize != azx_dev->bufsize ||
	    period_bytes != azx_dev->period_bytes ||
	    format_val != azx_dev->format_val) {
		azx_dev->bufsize = bufsize;
		azx_dev->period_bytes = period_bytes;
		azx_dev->format_val = format_val;
		err = snd_hdac_stream_setup_compr_periods(azx_dev);
		if (err < 0)
			return err;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(snd_hdac_stream_set_compr_params);
