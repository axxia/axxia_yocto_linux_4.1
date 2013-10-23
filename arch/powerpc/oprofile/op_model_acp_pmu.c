/*
 * Freescale Embedded oprofile support, based on ppc64 oprofile support
 * Copyright (C) 2004 Anton Blanchard <anton@au.ibm.com>, IBM
 *
 * Copyright (c) 2004, 2010 Freescale Semiconductor, Inc
 *
 * Author: Andy Fleming
 * Maintainer: Kumar Gala <galak@kernel.crashing.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/oprofile.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/ptrace.h>
#include <asm/processor.h>
#include <asm/cputable.h>
#include <asm/reg_acp_pmu_fn.h>
#include <asm/page.h>
#include <asm/pmc.h>
#include <asm/oprofile_impl.h>

static unsigned long reset_value[OP_MAX_COUNTER];

static int num_counters;
static int oprofile_running;



static void init_pmc_stop(int core, int ctr)
{
	u32 pmlc = (PMLCA_FC | PMLCA_FCS | PMLCA_FCU |
		    PMLCA_FCM1 | PMLCA_FCM0);

	set_pmlc(core, ctr, pmlc);
}

static void set_pmc_event(int core, int ctr, int event)
{
	u32 pmlc;

	pmlc = get_pmlc(core, ctr);

	pmlc = (pmlc & ~PMLCA_EVENT_MASK) |
		((event << PMLCA_EVENT_SHIFT) &
		 PMLCA_EVENT_MASK);

	set_pmlc(core, ctr, pmlc);
}

static void set_pmc_user_kernel(int core, int ctr, int user, int kernel)
{
	u32 pmlc;

	pmlc = get_pmlc(core, ctr);

	if (user)
		pmlc &= ~PMLCA_FCU;
	else
		pmlc |= PMLCA_FCU;

	if (kernel)
		pmlc &= ~PMLCA_FCS;
	else
		pmlc |= PMLCA_FCS;

	set_pmlc(core, ctr, pmlc);
}

static void set_pmc_marked(int core, int ctr, int mark0, int mark1)
{
	u32 pmlc = get_pmlc(core, ctr);

	if (mark0)
		pmlc &= ~PMLCA_FCM0;
	else
		pmlc |= PMLCA_FCM0;

	if (mark1)
		pmlc &= ~PMLCA_FCM1;
	else
		pmlc |= PMLCA_FCM1;

	set_pmlc(core, ctr, pmlc);
}

static void pmc_start_ctr(int core, int ctr, int enable)
{
	u32 pmlc = get_pmlc(core, ctr);

	pmlc &= ~PMLCA_FC;

	if (enable)
		pmlc |= PMLCA_CE;
	else
		pmlc &= ~PMLCA_CE;

	set_pmlc(core, ctr, pmlc);
}

static void pmc_start_ctrs(int core, int enable, u32 ie_mask)
{
	u32 pmgc0;

	/* Enable interrupt on overflow condition for enabled counters */
	mtdcrx(PMUDCRAI(core), PMRN_PMUIE0);
	if (enable)
		mtdcrx(PMUDCRDI(core), ie_mask);
	else
		mtdcrx(PMUDCRDI(core), 0);

	/* Start counters */
	mtdcrx(PMUDCRAI(core), PMRN_PMUGC0);
	pmgc0 = mfdcrx(PMUDCRDI(core));
	pmgc0 &= ~PMUGC0_LFAC; /* un-freeze all counters */
	pmgc0 |= PMUGC0_FCEC; /* enable freeze all ctrs on of */

	mtdcrx(PMUDCRDI(core), pmgc0);
}

static void pmc_stop_ctrs(int core)
{
	u32 pmgc0;

	mtdcrx(PMUDCRAI(core), PMRN_PMUGC0);
	pmgc0 = mfdcrx(PMUDCRDI(core));


	pmgc0 |= (PMUGC0_LFAC|PMUGC0_PMCC);

	pmgc0 &= ~PMUGC0_FCEC;
	mtdcrx(PMUDCRDI(core), pmgc0);
	mtdcrx(PMUDCRAI(core), PMRN_PMUIE0);
	mtdcrx(PMUDCRDI(core), 0);

}

static int acp_pmu_cpu_setup(struct op_counter_config *ctr)
{
	int i;
	int core = smp_processor_id();

	/* freeze all counters */
	pmc_stop_ctrs(core);

	for (i = 0; i < num_counters; i++) {
		init_pmc_stop(core, i);
		set_pmc_event(core, i, ctr[i].event);
		set_pmc_user_kernel(core, i, ctr[i].user, ctr[i].kernel);
	}

	return 0;
}

static int acp_pmu_reg_setup(struct op_counter_config *ctr,
			     struct op_system_config *sys,
			     int num_ctrs)
{
	int i;

	num_counters = num_ctrs;

	/* Our counters count up, and "count" refers to
	 * how much before the next interrupt, and we interrupt
	 * on overflow.  So we calculate the starting value
	 * which will give us "count" until overflow.
	 * Then we set the events on the enabled counters */
	for (i = 0; i < num_counters; ++i)
		reset_value[i] = 0x80000000UL - ctr[i].count;

	return 0;
}

static int acp_pmu_start(struct op_counter_config *ctr)
{
	int i;
	u32 ie_mask = 0;
	int core = smp_processor_id();

	/* Freeze counters during update */

	mtmsr(mfmsr() | MSR_PMM);

	for (i = 0; i < num_counters; ++i) {
		if (ctr[i].enabled) {
			ie_mask |= PMUIE_IE(i);

			ctr_write(core, i, reset_value[i]);
			/* Set each enabled counter to only
			 * count when the Mark bit is *not* set */
			set_pmc_marked(core, i, 1, 0);
			pmc_start_ctr(core, i, 1);
		} else {
			ctr_write(core, i, 0);

			/* Set the ctr to be stopped */
			pmc_start_ctr(core, i, 0);
		}
	}

	/* Clear the freeze bit, and enable the interrupt.
	 * The counters won't actually start until the rfi clears
	 * the PMM bit */
	pmc_start_ctrs(core, 1, ie_mask);

	oprofile_running = 1;
#ifdef DEBUG
	mtdcrx(PMUDCRAI(core), PMRN_PMUGC0);
#endif
	pr_debug("start on cpu %d, pmgc0 %x\n", core,
		 mfdcrx(PMUDCRDI(core)));

	return 0;
}

static void acp_pmu_stop(void)
{
	int core = smp_processor_id();

	/* freeze counters */
	pmc_stop_ctrs(core);

	oprofile_running = 0;

	mtdcrx(PMUDCRAI(core), PMRN_PMUGC0);
	pr_debug("stop on cpu %d, pmgc0 %x\n", smp_processor_id(),
		 mfdcrx(PMUDCRDI(core)));

	mb();
}


static void acp_pmu_handle_interrupt(struct pt_regs *regs,
				    struct op_counter_config *ctr)
{
	int core = smp_processor_id();
	u32 ie_mask = 0;
	unsigned long pc;
	int is_kernel;
	int val;
	int i;

	pc = regs->nip;
	is_kernel = is_kernel_addr(pc);

	for (i = 0; i < num_counters; ++i) {
		val = ctr_read(core, i);
		if (val < 0) {
			if (oprofile_running && ctr[i].enabled) {
				ie_mask |= PMUIE_IE(i);
				oprofile_add_ext_sample(pc, regs, i, is_kernel);
				ctr_write(core, i, reset_value[i]);
			} else {
				ctr_write(core, i, 0);
			}
		}
	}

	/* The freeze bit was set by the interrupt. */
	/* Clear the freeze bit, and reenable the interrupt.  The
	 * counters won't actually start until the rfi clears the PMM
	 * bit.  The PMM bit should not be set until after the interrupt
	 * is cleared to avoid it getting lost in some hypervisor
	 * environments.
	 */
	mtmsr(mfmsr() | MSR_PMM);
	pmc_start_ctrs(core, 1, ie_mask);
}

struct op_powerpc_model op_model_acp_pmu = {
	.reg_setup		= acp_pmu_reg_setup,
	.cpu_setup		= acp_pmu_cpu_setup,
	.start			= acp_pmu_start,
	.stop			= acp_pmu_stop,
	.handle_interrupt	= acp_pmu_handle_interrupt,
};
