/*
 * Performance counter support for LSI Axxia3400
 *
 * Based on earlier code:
 *
 * e500mc-pmu.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/perf_event.h>
#include <asm/pmc.h>
#include <asm/reg.h>
#include <asm/cputable.h>
#include <asm/perf_event_acp.h>
#include <asm/reg_acp_pmu.h>

/*
 * Map of generic hardware event types to hardware events
 * Zero if unsupported
 */
static int ppc476_generic_events[] = {
	[PERF_COUNT_HW_CPU_CYCLES]              = CPU_CYCLE_COUNT,
	[PERF_COUNT_HW_INSTRUCTIONS]            = CPU_COMMITTED_INST,
	[PERF_COUNT_HW_CACHE_REFERENCES]        = CPU_DCACHE_HIT,
	[PERF_COUNT_HW_CACHE_MISSES]            = CPU_DTLB_RELOAD,

};

/*
 * Table of generalized cache-related events.
 *
 * 0 means not supported, -1 means nonsensical, other values
 * are event codes.
 */

#define C(x)	PERF_COUNT_HW_CACHE_##x

static int ppc476_cache_events[C(MAX)][C(OP_MAX)][C(RESULT_MAX)] = {

	/*
	 * The PPC476 PMU does have a few cache events but they don't
	 * fit directly into this model. Therefore, we need to combine
	 * several PM events to get the numbers that perf is looking for.
	 *
	 */

	[C(L1D)] = {            /*      RESULT_ACCESS   RESULT_MISS */
		[C(OP_READ)] = {        CPU_DCACHE_HIT, CPU_DTLB_RELOAD  },
		[C(OP_WRITE)] = {       CPU_DCACHE_HIT, CPU_DTLB_RELOAD  },
		[C(OP_PREFETCH)] = {    0,              0                },
	},
	[C(L1I)] = {            /*      RESULT_ACCESS   RESULT_MISS */
		[C(OP_READ)] = {        CPU_ICACHE_HIT, CPU_ITLB_RELOAD  },
		[C(OP_WRITE)] = {       CPU_ICACHE_HIT, CPU_ITLB_RELOAD  },
		[C(OP_PREFETCH)] = {    0,              0                },
	},
	[C(LL)] = {             /*      RESULT_ACCESS   RESULT_MISS */
		[C(OP_READ)] = {        0,              0       },
		[C(OP_WRITE)] = {       0,              0       },
		[C(OP_PREFETCH)] = {    0,              0       },
	},

	/*
	 * There are data/instruction MMU misses, but that's a miss on
	 * the chip's internal level-one TLB which is probably not
	 * what the user wants.  Instead, unified level-two TLB misses
	 * are reported here.
	 */

	[C(DTLB)] = {		/*	RESULT_ACCESS	RESULT_MISS */
		[C(OP_READ)] = {	0,	        0	},
		[C(OP_WRITE)] = {       0,		0	},
		[C(OP_PREFETCH)] = {    0,		0	},
	},
	[C(ITLB)] = {		/*	RESULT_ACCESS	RESULT_MISS */
		[C(OP_READ)] = {	0,	        0	},
		[C(OP_WRITE)] = {	0,		0	},
		[C(OP_PREFETCH)] = {	0,		0	},
	},
	[C(BPU)] = {		/*	RESULT_ACCESS	RESULT_MISS */
		[C(OP_READ)] = {	-1,		-1	},
		[C(OP_WRITE)] = {	-1,		-1	},
		[C(OP_PREFETCH)] = {	-1,		-1	},
	},
	[C(NODE)] = {		/*	RESULT_ACCESS	RESULT_MISS */
		[C(OP_READ)] = {	-1,		-1	},
		[C(OP_WRITE)] = {	-1,		-1	},
		[C(OP_PREFETCH)] = {	-1,		-1	},
	},
};

static int num_events = 32;


static u64 ppc476_xlate_event(u64 event_id)
{
	u32 event_low = (u32)event_id;
	u64 ret;

	if (event_low >= num_events)
		return 0;

	ret = ACP_EVENT_VALID;

	return ret;
}

static struct acp_pmu ppc476_pmu = {
	.name			= "ppc476 family",
	.n_counter		= 8,
	.xlate_event		= ppc476_xlate_event,
	.n_generic		= ARRAY_SIZE(ppc476_generic_events),
	.generic_events		= ppc476_generic_events,
	.cache_events		= &ppc476_cache_events,
};

static int init_ppc476_pmu(void)
{
	if (!cur_cpu_spec->oprofile_cpu_type)
		return -ENODEV;

	if (!strcmp(cur_cpu_spec->oprofile_cpu_type, "ppc/476"))
		num_events = 32;
	else
		return -ENODEV;

	return register_acp_pmu(&ppc476_pmu);
}

early_initcall(init_ppc476_pmu);
