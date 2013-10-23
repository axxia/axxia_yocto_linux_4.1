/*
 * Performance event support - LSI ACP Embedded Performance Monitor
 *
 * Based on earlier code:
 *
 * perf_event_fsl_emb.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/cpumask.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/perf_event.h>
#include <linux/percpu.h>
#include <linux/hardirq.h>
#include <asm/reg_acp_pmu_fn.h>
#include <asm/pmc.h>
#include <asm/machdep.h>
#include <linux/firmware.h>
#include <linux/ptrace.h>


struct cpu_hw_events {
	int n_events;
	int disabled;
	u8  pmcs_enabled;
	struct perf_event *event[MAX_HWEVENTS];
};
static DEFINE_PER_CPU(struct cpu_hw_events, cpu_hw_events);

static struct acp_pmu *ppmu;

/* Number of perf_events counting hardware events */
static atomic_t num_events;
/* Used to avoid races in calling reserve/release_pmc_hardware */
static DEFINE_MUTEX(pmc_reserve_mutex);

/*
 * If interrupts were soft-disabled when a PMU interrupt occurs, treat
 * it as an NMI.
 */
static inline int perf_intr_is_nmi(struct pt_regs *regs)
{
#ifdef __powerpc64__
	return !regs->softe;
#else
	return 0;
#endif
}

static void perf_event_interrupt(struct pt_regs *regs);

static int cpu_to_pmu_irq(int cpu)
{
	int hwirq;

	/*
	 * NOTE: On the LSI ACP platform, the PMU interrupts are
	 * hard-wired as inputs to the MPIC. The irq numbers are
	 * fixed as follows:
	 *
	 *   Core 0 PMU: IRQ 95
	 *   Core 1 PMU: IRQ 94
	 *   Core 2 PMU: IRQ 93
	 *   Core 3 PMU: IRQ 92
	 *
	 * The IRQ assignment should probably be done in the DTB,
	 * like ARM does, but no other PowerPC platform does this.
	 * So for now, we hard-code the numbers here.
	 */
	if (cpu == 0)
		hwirq = 95;
	else if (cpu == 1)
		hwirq = 94;
	else if (cpu == 2)
		hwirq = 93;
	else if (cpu == 3)
		hwirq = 92;
	else
		hwirq = 0;

	return hwirq;
}

static cpumask_t active_irqs;

/* PMU IRQ handler */
static irqreturn_t acp_pmu_isr(int irq, void *dev_id)
{
	__get_cpu_var(irq_stat).pmu_irqs++;
	perf_irq(get_irq_regs());
	return IRQ_HANDLED;
}

static void acp_pmu_release_hardware(void)
{
	int i, irq, virq;

	for (i = 0; i < num_possible_cpus(); ++i) {
		if (!cpumask_test_and_clear_cpu(i, &active_irqs))
			continue;
		irq = cpu_to_pmu_irq(i);
		if (irq) {
			free_irq(irq, NULL);
			virq = irq_find_mapping(NULL, irq);
			if (virq)
				irq_dispose_mapping(virq);
		}
	}

	release_pmc_hardware();
}

static int acp_pmu_reserve_hardware(void)
{
	int err = 0;
	int i, irq, hwirq;

	err = reserve_pmc_hardware(perf_event_interrupt);

	if (err) {
		pr_warning("unable to reserve pmu\n");
		return err;
	}

	for (i = 0; i < num_possible_cpus(); ++i) {
		err = 0;

		hwirq = cpu_to_pmu_irq(i);
		if (!hwirq)
			continue;

		irq = irq_create_mapping(NULL, hwirq);
		if (irq == NO_IRQ) {
			pr_err("PMU irq_create_mapping() failed\n");
			continue;
		}

		irq = cpu_to_pmu_irq(i);
		if (irq < 0)
			continue;

		if (irq_set_affinity(irq, cpumask_of(i))) {
			pr_warning("PMU IRQ affinity failed (irq=%d, cpu=%d)\n",
				   irq, i);
			continue;
		}
		err = request_irq(irq, acp_pmu_isr,
				  IRQF_DISABLED | IRQF_NOBALANCING,
				  "pmu", NULL);
		if (err) {
			pr_err("PMU reqeust for IRQ%d failed\n", irq);
			acp_pmu_release_hardware();
			return err;
		}
		cpumask_set_cpu(i, &active_irqs);
	}

	return 0;
}


static void acp_pmu_read(struct perf_event *event)
{
	int core = smp_processor_id();
	s64 val, delta, prev;

	if (event->hw.state & PERF_HES_STOPPED)
		return;

	/*
	 * Performance monitor interrupts come even when interrupts
	 * are soft-disabled, as long as interrupts are hard-enabled.
	 * Therefore we treat them like NMIs.
	 */
	do {
		prev = local64_read(&event->hw.prev_count);
		barrier();
		val = ctr_read(core, event->hw.idx);
	} while (local64_cmpxchg(&event->hw.prev_count, prev, val) != prev);

	/* The counters are only 32 bits wide */
	delta = (val - prev) & 0xfffffffful;
	local64_add(delta, &event->count);
	local64_sub(delta, &event->hw.period_left);
}

/*
 * Disable all events to prevent PMU interrupts and to allow
 * events to be added or removed.
 */
static void acp_pmu_disable(struct pmu *pmu)
{
	int core = smp_processor_id();
	struct cpu_hw_events *cpuhw;
	unsigned long flags;

	local_irq_save(flags);
	cpuhw = &__get_cpu_var(cpu_hw_events);

	if (!cpuhw->disabled) {
		cpuhw->disabled = 1;

		/*
		 * Check if we ever enabled the PMU on this cpu.
		 */
		if (!cpuhw->pmcs_enabled) {
			ppc_enable_pmcs();
			cpuhw->pmcs_enabled = 1;
		}

		if (atomic_read(&num_events)) {
			u32 pmgc0;
			/*
			 * Set the 'freeze all counters' bit, and disable
			 * interrupts.  The barrier is to make sure the
			 * mtpmr has been executed and the PMU has frozen
			 * the events before we return.
			 */
			mtdcrx(PMUDCRAI(core), PMRN_PMUGC0);
			pmgc0 = mfdcrx(PMUDCRDI(core));
			pmgc0 |= PMUGC0_LFAC;
			pmgc0 &= ~PMUGC0_FCEC;
			mtdcrx(PMUDCRDI(core), pmgc0);
			mtdcrx(PMUDCRAI(core), PMRN_PMUIE0);
			mtdcrx(PMUDCRDI(core), 0);

			isync();
		}
	}
	local_irq_restore(flags);
}

/*
 * Re-enable all events if disable == 0.
 * If we were previously disabled and events were added, then
 * put the new config on the PMU.
 */
static void acp_pmu_enable(struct pmu *pmu)
{
	int core = smp_processor_id();
	struct cpu_hw_events *cpuhw;
	unsigned long flags;

	local_irq_save(flags);
	cpuhw = &__get_cpu_var(cpu_hw_events);
	if (!cpuhw->disabled)
		goto out;

	cpuhw->disabled = 0;
	ppc_set_pmu_inuse(cpuhw->n_events != 0);

	if (cpuhw->n_events > 0) {
		u32 pmgc0;
		u32 pmuie0 = 0;
		int i;
		int num_counters = ppmu->n_counter;
		for (i = 0; i < num_counters; i++) {
			if (cpuhw->event[i])
				pmuie0 |= PMUIE_IE(i);
		}
		mtdcrx(PMUDCRAI(core), PMRN_PMUIE0);
		mtdcrx(PMUDCRDI(core), pmuie0);

		mtdcrx(PMUDCRAI(core), PMRN_PMUGC0);
		pmgc0 = mfdcrx(PMUDCRDI(core));
		pmgc0 &= ~PMUGC0_LFAC; /* un-freeze all counters */
		pmgc0 |= PMUGC0_FCEC; /* enable freeze all ctrs on of */
		mtdcrx(PMUDCRDI(core), pmgc0);

		isync();
	}

out:
	local_irq_restore(flags);
}

static int collect_events(struct perf_event *group, int max_count,
			  struct perf_event *ctrs[])
{
	int n = 0;
	struct perf_event *event;

	if (!is_software_event(group)) {
		if (n >= max_count)
			return -1;
		ctrs[n] = group;
		n++;
	}
	list_for_each_entry(event, &group->sibling_list, group_entry) {
		if (!is_software_event(event) &&
		    event->state != PERF_EVENT_STATE_OFF) {
			if (n >= max_count)
				return -1;
			ctrs[n] = event;
			n++;
		}
	}
	return n;
}

/* context locked on entry */
static int acp_pmu_add(struct perf_event *event, int flags)
{
	struct cpu_hw_events *cpuhw;
	int core = smp_processor_id();
	int ret = -EAGAIN;
	int num_counters = ppmu->n_counter;
	u64 val;
	int i;
	u32 pmuie0;

	perf_pmu_disable(event->pmu);
	cpuhw = &get_cpu_var(cpu_hw_events);

	/* Allocate counters */
	for (i = 0; i < num_counters; i++) {
		if (cpuhw->event[i])
			continue;

		break;
	}

	if (i < 0)
		goto out;

	event->hw.idx = i;
	cpuhw->event[i] = event;
	++cpuhw->n_events;

	val = 0;
	if (event->hw.sample_period) {
		s64 left = local64_read(&event->hw.period_left);
		if (left < 0x80000000L)
			val = 0x80000000L - left;
	}
	local64_set(&event->hw.prev_count, val);

	if (!(flags & PERF_EF_START)) {
		event->hw.state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
		val = 0;
	}

	ctr_write(core, i, val);
	perf_event_update_userpage(event);

	set_pmlc(core, i, event->hw.config_base);

	/* Enable counter interrupt on overflow condition */
	mtdcrx(PMUDCRAI(core), PMRN_PMUIE0);
	pmuie0 = mfdcrx(PMUDCRDI(core));
	pmuie0 |= PMUIE_IE(i);
	mtdcrx(PMUDCRDI(core), pmuie0);

	ret = 0;
out:
	put_cpu_var(cpu_hw_events);
	perf_pmu_enable(event->pmu);
	return ret;
}

/* context locked on entry */
static void acp_pmu_del(struct perf_event *event, int flags)
{
	int core = smp_processor_id();
	struct cpu_hw_events *cpuhw;
	int i = event->hw.idx;

	perf_pmu_disable(event->pmu);
	if (i < 0)
		goto out;

	acp_pmu_read(event);

	cpuhw = &get_cpu_var(cpu_hw_events);

	WARN_ON(event != cpuhw->event[event->hw.idx]);

	set_pmlc(core, i, 0);
	ctr_write(core, i, 0);

	cpuhw->event[i] = NULL;
	event->hw.idx = -1;

out:
	perf_pmu_enable(event->pmu);
	put_cpu_var(cpu_hw_events);
}

static void acp_pmu_start(struct perf_event *event, int ef_flags)
{
	int core = smp_processor_id();
	unsigned long flags;
	s64 left;

	if (event->hw.idx < 0 || !event->hw.sample_period)
		return;

	if (!(event->hw.state & PERF_HES_STOPPED))
		return;

	if (ef_flags & PERF_EF_RELOAD)
		WARN_ON_ONCE(!(event->hw.state & PERF_HES_UPTODATE));

	local_irq_save(flags);
	perf_pmu_disable(event->pmu);

	event->hw.state = 0;
	left = local64_read(&event->hw.period_left);
	ctr_write(core, event->hw.idx, left);

	perf_event_update_userpage(event);
	perf_pmu_enable(event->pmu);
	local_irq_restore(flags);
}

static void acp_pmu_stop(struct perf_event *event, int ef_flags)
{
	unsigned long flags;
	int core = smp_processor_id();

	if (event->hw.idx < 0 || !event->hw.sample_period)
		return;

	if (event->hw.state & PERF_HES_STOPPED)
		return;

	local_irq_save(flags);
	perf_pmu_disable(event->pmu);

	acp_pmu_read(event);
	event->hw.state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
	ctr_write(core, event->hw.idx, 0);

	perf_event_update_userpage(event);
	perf_pmu_enable(event->pmu);
	local_irq_restore(flags);
}

/*
 * Release the PMU if this is the last perf_event.
 */
static void hw_perf_event_destroy(struct perf_event *event)
{
	if (!atomic_add_unless(&num_events, -1, 1)) {
		mutex_lock(&pmc_reserve_mutex);
		if (atomic_dec_return(&num_events) == 0)
			acp_pmu_release_hardware();
		mutex_unlock(&pmc_reserve_mutex);
	}
}

/*
 * Translate a generic cache event_id config to a raw event_id code.
 */
static int hw_perf_cache_event(u64 config, u64 *eventp)
{
	unsigned long type, op, result;
	int ev;

	if (!ppmu->cache_events)
		return -EINVAL;

	/* unpack config */
	type = config & 0xff;
	op = (config >> 8) & 0xff;
	result = (config >> 16) & 0xff;

	if (type >= PERF_COUNT_HW_CACHE_MAX ||
	    op >= PERF_COUNT_HW_CACHE_OP_MAX ||
	    result >= PERF_COUNT_HW_CACHE_RESULT_MAX)
		return -EINVAL;

	ev = (*ppmu->cache_events)[type][op][result];
	if (ev == 0)
		return -EOPNOTSUPP;
	if (ev == -1)
		return -EINVAL;
	*eventp = ev;
	return 0;
}

static int acp_pmu_event_init(struct perf_event *event)
{
	u64 ev;
	struct perf_event *events[MAX_HWEVENTS];
	int n;
	int err;

	switch (event->attr.type) {
	case PERF_TYPE_HARDWARE:
		ev = event->attr.config;
		if (ev >= ppmu->n_generic || ppmu->generic_events[ev] == 0)
			return -EOPNOTSUPP;
		ev = ppmu->generic_events[ev];
		break;

	case PERF_TYPE_HW_CACHE:
		err = hw_perf_cache_event(event->attr.config, &ev);
		if (err)
			return err;
		break;

	case PERF_TYPE_RAW:
		ev = event->attr.config;
		break;

	default:
		return -ENOENT;
	}

	event->hw.config = ppmu->xlate_event(ev);
	if (!(event->hw.config & ACP_EVENT_VALID))
		return -EINVAL;

	/*
	 * If this is in a group, check if it can go on with all the
	 * other hardware events in the group.  We assume the event
	 * hasn't been linked into its leader's sibling list at this point.
	 */
	n = 0;
	if (event->group_leader != event) {
		n = collect_events(event->group_leader,
				   ppmu->n_counter - 1, events);
		if (n < 0)
			return -EINVAL;
	}
	event->hw.idx = -1;

	event->hw.config_base = PMLCA_CE | PMLCA_FCM1 |
				(u32)((ev) & PMLCA_EVENT_MASK);

	if (event->attr.exclude_user)
		event->hw.config_base |= PMLCA_FCU;
	if (event->attr.exclude_kernel)
		event->hw.config_base |= PMLCA_FCS;
	if (event->attr.exclude_idle)
		return -ENOTSUPP;

	event->hw.last_period = event->hw.sample_period;
	local64_set(&event->hw.period_left, event->hw.last_period);

	/*
	 * See if we need to reserve the PMU.
	 * If no events are currently in use, then we have to take a
	 * mutex to ensure that we don't race with another task doing
	 * reserve_pmc_hardware or release_pmc_hardware.
	 */
	err = 0;
	if (!atomic_inc_not_zero(&num_events)) {

		mutex_lock(&pmc_reserve_mutex);
		if (atomic_read(&num_events) == 0 &&
		    acp_pmu_reserve_hardware())
			err = -EBUSY;
		else
			atomic_inc(&num_events);
		mutex_unlock(&pmc_reserve_mutex);
		isync();
	}
	event->destroy = hw_perf_event_destroy;

	return err;
}

static struct pmu acp_pmu = {
	.pmu_enable	= acp_pmu_enable,
	.pmu_disable	= acp_pmu_disable,
	.event_init	= acp_pmu_event_init,
	.add		= acp_pmu_add,
	.del		= acp_pmu_del,
	.start		= acp_pmu_start,
	.stop		= acp_pmu_stop,
	.read		= acp_pmu_read,
};

/*
 * A counter has overflowed; update its count and record
 * things if requested.  Note that interrupts are hard-disabled
 * here so there is no possibility of being interrupted.
 */
static void record_and_restart(int core, struct perf_event *event,
			       unsigned long val, struct pt_regs *regs)
{
	u64 period = event->hw.sample_period;
	s64 prev, delta, left;
	int record = 0;

	if (event->hw.state & PERF_HES_STOPPED) {
		ctr_write(core, event->hw.idx, 0);
		return;
	}

	/* we don't have to worry about interrupts here */
	prev = local64_read(&event->hw.prev_count);
	delta = (val - prev) & 0xfffffffful;
	local64_add(delta, &event->count);

	/*
	 * See if the total period for this event has expired,
	 * and update for the next period.
	 */
	val = 0;
	left = local64_read(&event->hw.period_left) - delta;
	if (period) {
		if (left <= 0) {
			left += period;
			if (left <= 0)
				left = period;
			record = 1;
			event->hw.last_period = event->hw.sample_period;
		}
		if (left < 0x80000000LL)
			val = 0x80000000LL - left;
	}

	ctr_write(core, event->hw.idx, val);
	local64_set(&event->hw.prev_count, val);
	local64_set(&event->hw.period_left, left);
	perf_event_update_userpage(event);

	/*
	 * Finally record data if requested.
	 */
	if (record) {
		struct perf_sample_data data;

		perf_sample_data_init(&data, 0, event->hw.last_period);

		if (perf_event_overflow(event, &data, regs))
			acp_pmu_stop(event, 0);
	}
}

static void perf_event_interrupt(struct pt_regs *regs)
{
	int i;
	int core = smp_processor_id();
	struct cpu_hw_events *cpuhw = &__get_cpu_var(cpu_hw_events);
	struct perf_event *event;
	unsigned long val;
	int found = 0;
	int nmi;
	u32 pmgc0;

	nmi = perf_intr_is_nmi(regs);
	if (nmi)
		nmi_enter();
	else
		irq_enter();

	for (i = 0; i < ppmu->n_counter; ++i) {
		event = cpuhw->event[i];
		val = ctr_read(core, i);
		if ((int)val < 0) {
			if (event) {
				/* event has overflowed */
				found = 1;
				record_and_restart(core, event, val, regs);
			} else {
				/*
				 * Disabled counter is negative,
				 * reset it just in case.
				 */
				ctr_write(core, i, 0);
			}
		}
	}

	/* PMM will keep counters frozen until we return from the interrupt. */
	mtmsr(mfmsr() | MSR_PMM);

	mtdcrx(PMUDCRAI(core), PMRN_PMUGC0);
	pmgc0 = mfdcrx(PMUDCRDI(core));
	pmgc0 &= ~PMUGC0_LFAC; /* un-freeze all counters */
	pmgc0 |= PMUGC0_FCEC; /* enable freeze all ctrs on of */
	mtdcrx(PMUDCRDI(core), pmgc0);

	isync();

	if (nmi)
		nmi_exit();
	else
		irq_exit();
}

int register_acp_pmu(struct acp_pmu *pmu)
{
	int core;

	if (ppmu)
		return -EBUSY;		/* something's already registered */

	/*
	 * ACP PMU is enabled and may fire after reset
	 * disable until someone is listening
	 */

	for_each_possible_cpu(core) {
		u32 pmgc0;

		mtdcrx(PMUDCRAI(core), PMRN_PMUGC0);
		pmgc0 = mfdcrx(PMUDCRDI(core));
		pmgc0 |= (PMUGC0_LFAC|PMUGC0_PMCC);
		pmgc0 &= ~PMUGC0_FCEC;
		mtdcrx(PMUDCRDI(core), pmgc0);
	}

	ppmu = pmu;
	pr_info("%s performance monitor hardware support registered\n",
		pmu->name);

	perf_pmu_register(&acp_pmu, "cpu", PERF_TYPE_RAW);

	return 0;
}
