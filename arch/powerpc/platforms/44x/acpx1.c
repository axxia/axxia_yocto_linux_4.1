/*
 * PPC476 board specific routines
 *
 * - add udbg and clock support
 *
 * Copyright 2009 Torez Smith, IBM Corporation.
 *
 * Based on earlier code:
 *    Matt Porter <mporter@kernel.crashing.org>
 *    Copyright 2002-2005 MontaVista Software Inc.
 *
 *    Eugene Surovegin <eugene.surovegin@zultys.com> or <ebs@ebshome.net>
 *    Copyright (c) 2003-2005 Zultys Technologies
 *
 *    Rewritten and ported to the merged powerpc tree:
 *    Copyright 2007 David Gibson <dwg@au1.ibm.com>, IBM Corporation.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * These patches add ACP3400 support signed-off-by: john.jacques@lsi.com
 */

#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/rtc.h>
#include <linux/kexec.h>
#include <linux/highmem.h>

#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/time.h>
#include <asm/uic.h>
#include <asm/ppc4xx.h>
#include <asm/mpic.h>
#include <asm/mmu.h>

#include <sysdev/mpic.h>
#include "acpclock.h"

static __initdata struct of_device_id acpx14xx_of_bus[] = {
	{ .compatible = "ibm,plb4", },
	{ .compatible = "ibm,plb6", },
	{ .compatible = "ibm,opb", },
	{ .compatible = "ibm,ebc", },
	{ .compatible = "acp,rapidio-delta", },
	{},
};

static int __init acpx14xx_device_probe(void)
{
	acp_clk_init();

	of_platform_bus_probe(NULL, acpx14xx_of_bus, NULL);

	return 0;
}
machine_device_initcall(acpx14xx, acpx14xx_device_probe);

/* We can have either UICs or MPICs */
static void __init acpx14xx_init_irq(void)
{
	struct device_node *np;

	/* Find top level interrupt controller */
	for_each_node_with_property(np, "interrupt-controller") {
		if (of_get_property(np, "interrupts", NULL) == NULL)
			break;
	}
	if (np == NULL)
		panic("Can't find top level interrupt controller");

	/* Check type and do appropriate initialization */
	if (of_device_is_compatible(np, "chrp,open-pic")) {
		/* The MPIC driver will get everything it needs from the
		 * device-tree, just pass 0 to all arguments
		 */
		struct mpic *mpic =
			mpic_alloc(np, 0, 0, 0, 0, " MPIC     ");
		BUG_ON(mpic == NULL);
		mpic_init(mpic);
		ppc_md.get_irq = mpic_get_irq;
	} else
		panic("Unrecognized top level interrupt controller");
}

#ifdef CONFIG_SMP
#ifdef CONFIG_KEXEC
atomic_t kexec_down_cpus = ATOMIC_INIT(0);
extern void kexec_smp_wait(void);
void smp_acpx14xx_kexec_cpu_down(int crash_shutdown, int secondary)
{
	local_irq_disable();

	if (secondary) {
		atomic_inc(&kexec_down_cpus);
		kexec_smp_wait();
	}
}

static void smp_acpx14xx_kexec_down(void *arg)
{
	if (ppc_md.kexec_cpu_down)
		ppc_md.kexec_cpu_down(0, 1);
}

static void map_and_flush(unsigned long paddr)
{
	struct page *page = pfn_to_page(paddr >> PAGE_SHIFT);
	unsigned long kaddr  = (unsigned long)kmap(page);

	flush_dcache_range(kaddr, kaddr + PAGE_SIZE);
	kunmap(page);
}

static void smp_acpx14xx_flush_dcache_kexec(struct kimage *image)
{
	kimage_entry_t *ptr, entry;
	unsigned long paddr;
	int i;

	if (image->type == KEXEC_TYPE_DEFAULT) {
		/* normal kexec images are stored in temporary pages */
		for (ptr = &image->head; (entry = *ptr) && !(entry & IND_DONE);
		     ptr = (entry & IND_INDIRECTION) ?
				phys_to_virt(entry & PAGE_MASK) : ptr + 1) {
			if (!(entry & IND_DESTINATION))
				map_and_flush(entry);
		}
		/* flush out last IND_DONE page */
		map_and_flush(entry);
	} else {
		/* crash type kexec images are copied to the crash region */
		for (i = 0; i < image->nr_segments; i++) {
			struct kexec_segment *seg = &image->segment[i];
			for (paddr = seg->mem; paddr < seg->mem + seg->memsz;
			     paddr += PAGE_SIZE) {
				map_and_flush(paddr);
			}
		}
	}

	/* also flush the kimage struct to be passed in as well */
	flush_dcache_range((unsigned long)image,
			   (unsigned long)image + sizeof(*image));
}

static void smp_acpx14xx_machine_kexec(struct kimage *image)
{
	int timeout = INT_MAX;
	int i, num_cpus = num_present_cpus();

	smp_acpx14xx_flush_dcache_kexec(image);

	if (image->type == KEXEC_TYPE_DEFAULT)
		smp_call_function(smp_acpx14xx_kexec_down, NULL, 0);

	while ((atomic_read(&kexec_down_cpus) != (num_cpus - 1)) &&
		(timeout > 0)) {

		timeout--;
	}

	if (!timeout)
		pr_err("Unable to bring down secondary cpu(s)");

	for_each_online_cpu(i) {
		if (i == smp_processor_id())
			continue;
		mpic_reset_core(i);
	}

	default_machine_kexec(image);
}
#endif /* CONFIG_KEXEC */

static void __cpuinit smp_acpx14xx_setup_cpu(int cpu)
{
	mpic_setup_this_cpu();
}

static int __cpuinit smp_acpx14xx_kick_cpu(int cpu)
{
	struct device_node *cpunode = of_get_cpu_node(cpu, NULL);
	const u64 *spin_table_addr_prop;
	u32 *spin_table;
	extern void __cpuinit start_secondary_47x(void);

	BUG_ON(cpunode == NULL);

	/* Assume spin table. We could test for the enable-method in
	 * the device-tree but currently there's little point as it's
	 * our only supported method
	 */
	spin_table_addr_prop =
		of_get_property(cpunode, "cpu-release-addr", NULL);

	if (spin_table_addr_prop == NULL) {
		pr_err("CPU%d: Can't start, macpx1ing cpu-release-addr !\n",
		       cpu);
		return -1;
	}

	/* Assume it's mapped as part of the linear mapping. This is a bit
	 * fishy but will work fine for now
	 */
	spin_table = (u32 *)__va(*spin_table_addr_prop);
	pr_debug("CPU%d: Spin table mapped at %p\n", cpu, spin_table);

	spin_table[3] = cpu;
	smp_wmb();
	spin_table[1] = __pa(start_secondary_47x);
	mb();

	return 0;
}

static struct smp_ops_t acpx1_smp_ops = {
	.probe		= smp_mpic_probe,
	.message_pass	= smp_mpic_message_pass,
	.setup_cpu	= smp_acpx14xx_setup_cpu,
	.kick_cpu	= smp_acpx14xx_kick_cpu,
	.give_timebase	= smp_generic_give_timebase,
	.take_timebase	= smp_generic_take_timebase,
};

static void __init acpx14xx_smp_init(void)
{
	if (mmu_has_feature(MMU_FTR_TYPE_47x))
		smp_ops = &acpx1_smp_ops;
#ifdef CONFIG_KEXEC
	ppc_md.kexec_cpu_down = smp_acpx14xx_kexec_cpu_down;
	ppc_md.machine_kexec = smp_acpx14xx_machine_kexec;
#endif
}

#else /* CONFIG_SMP */
static void __init acpx14xx_smp_init(void) { }
#endif /* CONFIG_SMP */

static void __init acpx14xx_setup_arch(void)
{
	acpx14xx_smp_init();
}

/*
 * Called very early, MMU is off, device-tree isn't unflattened
 */
static int __init acpx14xx_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (!of_flat_dt_is_compatible(root, "lsi,acp") &&
	    !of_flat_dt_is_compatible(root, "ibm,acpx1-4xx"))
		return 0;

	return 1;
}

/*
 * Issue a "core" reset.
 */

void
acp_jump_to_boot_loader(void *input)
{
	mpic_teardown_this_cpu(0);
	/* This is only valid in the "core" reset case, so 0x10000000. */
	mtspr(SPRN_DBCR0, mfspr(SPRN_DBCR0) | 0x10000000);

	while (1)
		;		/* Just in case the jump fails. */
}

/*
 * Get all other cores to run "acp_jump_to_boot_loader()" then go
 * there as well.
 */

void
acp_reset_cores(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		if (cpu != smp_processor_id())
			smp_call_function_single(cpu, acp_jump_to_boot_loader,
						 NULL, 0);
	}

	acp_jump_to_boot_loader(NULL);
}

/*
 * Apply a system reset. Alternatively a board specific value may be
 * provided via the "reset-type" property in the cpu node.
 */
void acpx14xx_reset_system(char *cmd)
{
	struct device_node *np;
	u32 reset_type = DBCR0_RST_SYSTEM;
	const u32 *prop;

	np = of_find_node_by_type(NULL, "cpu");
	if (np) {
		prop = of_get_property(np, "reset-type", NULL);

		/*
		 * Check if property exists and if it is in range:
		 * 1 - PPC4xx core reset
		 * 2 - PPC4xx chip reset
		 * 3 - PPC4xx system reset (default)
		 */
		if ((prop) && ((prop[0] >= 1) && (prop[0] <= 3)))
			reset_type = prop[0] << 28;
	}

	if (DBCR0_RST_CORE == reset_type) {
		acp_reset_cores();
	} else {
		/*
		  In this case, reset_type is either chip or system.

		  On the AXM3500 (PVR=0x7ff520c1), writing to DBCR0
		  will occasionally stall the system.  As a
		  work-around, write to the system control register.
		*/

		u32 pvr_value;

		asm volatile ("mfpvr    %0" : "=r"(pvr_value));

		if (0x7ff520c1 == pvr_value) {
			u32 value;

			/* Enable privileged accesses */
			value = mfdcrx(0xd0a);
			value |= 0xab;
			mtdcrx(0xd0a, value);

			/* Switch to the reference clock */
			printk(KERN_WARNING
			       "Switching PPCs to Reference Clock\n");
			value = mfdcrx(0xd00);
			value &= ~0xc0000000;
			mtdcrx(0xd00, value);

			/* Reset */
			printk(KERN_WARNING
			       "Resetting Using SYSCON\n");
			mtdcrx(0xe00, (reset_type >> 28));
		} else {
			mtspr(SPRN_DBCR0, mfspr(SPRN_DBCR0) | reset_type);
		}
	}

	while (1)
		;	/* Just in case the reset doesn't work */
}

define_machine(acpx14xx) {
	.name			= "ACPX1",
	.probe			= acpx14xx_probe,
	.progress		= udbg_progress,
	.init_IRQ		= acpx14xx_init_irq,
	.setup_arch		= acpx14xx_setup_arch,
	.restart		= acpx14xx_reset_system,
	.calibrate_decr		= generic_calibrate_decr,
};
