#ifndef PERF_EVENT_ACP_H
#define PERF_EVENT_ACP_H

/*
 * Performance counter support for LSI Axxia3400
 *
 * Based on earlier code:
 *
 * perf_event_fsl_emb.h
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/types.h>
#include <asm/hw_irq.h>

#define MAX_HWEVENTS 8

/* event flags */
#define ACP_EVENT_VALID      1


struct acp_pmu {
	const char	*name;
	int		n_counter; /* total number of counters */
	/* Returns event flags */
	u64		(*xlate_event)(u64 event_id);

	int		n_generic;
	int		*generic_events;
	int		(*cache_events)[PERF_COUNT_HW_CACHE_MAX]
			       [PERF_COUNT_HW_CACHE_OP_MAX]
			       [PERF_COUNT_HW_CACHE_RESULT_MAX];
};

int register_acp_pmu(struct acp_pmu *);

#endif
