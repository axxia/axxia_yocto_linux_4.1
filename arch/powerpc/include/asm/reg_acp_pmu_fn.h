/*
 * Register read/write for the LSI Axxia3400 Embedded Performance
 * Monitor.
 *
 * Portions derived from CPP platform legacy perf driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifdef __KERNEL__
#ifndef __ASM_POWERPC_REG_ACP_PMU_FN_H__
#define __ASM_POWERPC_REG_ACP_PMU_FN_H__

#include <asm/reg_acp_pmu.h>

/* LSI ACP ppc476  Performance Monitor Registers */

/* common oprofile and perf event read/write pmu registers */

/* Get local counter control register */

static inline u32 get_pmlc(int core, int ctr)
{
	switch (ctr) {
	case 0:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA0);
		break;
	case 1:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA1);
		break;
	case 2:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA2);
		break;
	case 3:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA3);
		break;
	case 4:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA4);
		break;
	case 5:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA5);
		break;
	case 6:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA6);
		break;
	case 7:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA7);
		break;
	default:
		printk(KERN_ERR "oops trying to read PMC%d\n", ctr);
		return 0;
	}
	return mfdcrx(PMUDCRDI(core));
}

/* Set local counter control register */

static inline void set_pmlc(int core, int ctr, u32 pmlc)
{
	switch (ctr) {
	case 0:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA0);
		break;
	case 1:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA1);
		break;
	case 2:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA2);
		break;
	case 3:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA3);
		break;
	case 4:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA4);
		break;
	case 5:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA5);
		break;
	case 6:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA6);
		break;
	case 7:
		mtdcrx(PMUDCRAI(core),  PMRN_PMLCA7);
		break;
	default:
		printk(KERN_ERR "oops trying to read PMC%d\n", ctr);
		return;
	}
	mtdcrx(PMUDCRDI(core), pmlc);
}

/* Get local counter register */

static inline unsigned int ctr_read(int core, unsigned int i)
{
	switch (i) {
	case 0:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA0);
		break;
	case 1:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA1);
		break;
	case 2:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA2);
		break;
	case 3:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA3);
		break;
	case 4:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA4);
		break;
	case 5:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA5);
		break;
	case 6:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA6);
		break;
	case 7:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA7);
		break;
	default:
		return 0;
	}
	return mfdcrx(PMUDCRDI(core));
}

/* Set local counter register */

static inline void ctr_write(int core, unsigned int i, unsigned int val)
{
	switch (i) {
	case 0:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA0);
		break;
	case 1:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA1);
		break;
	case 2:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA2);
		break;
	case 3:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA3);
		break;
	case 4:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA4);
		break;
	case 5:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA5);
		break;
	case 6:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA6);
		break;
	case 7:
		mtdcrx(PMUDCRAI(core),  PMRN_PMCA7);
		break;
	default:
		return;
	}
	mtdcrx(PMUDCRDI(core), val);
}

#endif /* __ASM_POWERPC_REG_ACP_PMU_H__ */
#endif /* __KERNEL__ */
