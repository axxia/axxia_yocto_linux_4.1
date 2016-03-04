

int skl_probe_compr_open(struct snd_compr_stream *substream,
					struct snd_soc_dai *dai);

int skl_probe_compr_set_params(struct snd_compr_stream *substream,
			struct snd_compr_params *params, struct snd_soc_dai *dai);

int skl_probe_compr_tstamp(struct snd_compr_stream *stream,
			struct snd_compr_tstamp *tstamp, struct snd_soc_dai *dai);
void skl_probe_compr_close(struct snd_compr_stream *substream,
					struct snd_soc_dai *dai);
int skl_probe_compr_ack(struct snd_compr_stream *substream, size_t bytes,
					struct snd_soc_dai *dai);
int skl_probe_compr_copy(struct snd_compr_stream *stream, char __user *buf,
					size_t count, struct snd_soc_dai *dai);
int skl_probe_compr_trigger(struct snd_compr_stream *substream, int cmd,
					struct snd_soc_dai *dai);
