/*
 * Register definitions for the LSI Axxia3400 Embedded Performance
 * Monitor.
 *
 * Portions derived from CPP platform legacy perf driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#ifndef __ASM_POWERPC_REG_ACP_PMU_H__
#define __ASM_POWERPC_REG_ACP_PMU_H__

#define L2_LWARX_COMPLETE            (0x00)
#define L2_STWCX_SUCCESS             (0x01)
#define L2_MISS_EVICTION             (0x02)
#define L2_MISS_D_FETCH              (0x03)
#define L2_MISS_I_FETCH              (0x04)
#define L2_MISS_STORE                (0x05)
#define L2_HIT_D_FETCH               (0x06)
#define L2_HIT_I_FETCH               (0x07)
#define L2_HIT_STORE                 (0x08)
#define L2_READ_AFTER_WRITE          (0x09)
#define L2_WRITE_AFTER_WRITE         (0x0a)
#define PLB_MASTER_COMMAND           (0x0b)
#define PLB_MASTER_READ              (0x0c)
#define PLB_MASTER_RWITM             (0x0d)
#define PLB_MASTER_DCLAIM            (0x0e)
#define PLB_MASTER_WRITE             (0x0f)
#define PLB_READ_OCCUPANCY           (0x10)
#define PLB_MASTER_INTVN_M           (0x11)
#define PLB_MASTER_INTVN_S           (0x12)
#define PLB_MASTER_MEM_DATA          (0x13)
#define PLB_SNOOP_CMD                (0x14)
#define PLB_SNOOP_L2_CMD             (0x15)
#define PLB_SNOOP_HIT_INTVN          (0x16)
#define PLB_SNOOP_HIT                (0x17)
#define PLB_SNOOP_RETRY              (0x18)
#define CPU_COMMITTED_INST           (0x19)
#define CPU_DCACHE_HIT               (0x1a)
#define CPU_DTLB_RELOAD              (0x1b)
#define CPU_ICACHE_HIT               (0x1c)
#define CPU_ITLB_RELOAD              (0x1d)
#define L2_CYCLE_COUNT               (0x1e)
#define CPU_CYCLE_COUNT              (0x1f)

#ifdef __KERNEL__

#include <asm/dcr-native.h>

/* LSI ACP ppc476  Performance Monitor Registers */

/* Address and Data Indirect Register */
#define PMUDCRAI(core)               (0x80 + 0x300 + (core) * 0x100)
#define PMUDCRDI(core)               (0x84 + 0x300 + (core) * 0x100)

/* Global Control Registers */

#define PMRN_PMUGS0     0x000 /* PMU Global Status Register */

#define PMUGS_PMC_STATE(nr) (1<<(31-(nr))) /* Stop/Start state of PMUC(nr), */
					   /* 1== start */
#define PMUGS_CPUFAC    (1<<(31-29)) /* PMU_C476FAC signal input */
#define PMUGS_CPUR      (1<<(31-30)) /* PMU_C476PR signal input */
#define PMUGS_CPUMM     (1<<(31-31)) /* PMU_C476MM signal input */

#define PMRN_PMUGC0     0x004 /* PMU Global Control Register */

#define PMUGC0_PMCC      (1<<(31-15)) /* Returns all counters to zero */
#define PMUGC0_LFAC      (1<<(31-30)) /* Freeze all counters */
#define PMUGC0_FCEC      (1<<(31-31)) /* Freeze counters on enabled Condition*/

#define PMRN_PMUIS0     0x010 /* PMU Global Interrupt Status Register */

#define PMUIS_ISTAT(nr) (1<<(31-(nr))) /* Interrupt status of PMUC(nr), */
				       /* 1== counter has an enable condition */

#define PMRN_PMUIE0     0x014 /* PMU Global Interrupt Control Register */

#define PMUIE_IE(nr) (1<<(31-(nr))) /* Interrupt enable of PMUC(nr), */
				    /*  1== Interrupt enable for PMUC(nr) */

#define PMRN_REVID      0xC00 /* PMU Revision ID register */

/* Local Counter registers */

#define PMRN_PMCA0	0x808	/* Performance Monitor Counter 0 */
#define PMRN_PMCA1	0x818	/* Performance Monitor Counter 1 */
#define PMRN_PMCA2	0x828	/* Performance Monitor Counter 2 */
#define PMRN_PMCA3	0x838	/* Performance Monitor Counter 3 */
#define PMRN_PMCA4	0x908	/* Performance Monitor Counter 4 */
#define PMRN_PMCA5	0x918	/* Performance Monitor Counter 5 */
#define PMRN_PMCA6	0x928	/* Performance Monitor Counter 6 */
#define PMRN_PMCA7	0x938	/* Performance Monitor Counter 7 */
#define PMRN_PMLCA0	0x800	/* PM Local Counter Control Register 0 */
#define PMRN_PMLCA1	0x810	/* PM Local Counter Control Register 1 */
#define PMRN_PMLCA2	0x820	/* PM Local Counter Control Register 2 */
#define PMRN_PMLCA3	0x830	/* PM Local Counter Control Register 3 */
#define PMRN_PMLCA4	0x900	/* PM Local Counter Control Register 4 */
#define PMRN_PMLCA5	0x910	/* PM Local Counter Control Register 5 */
#define PMRN_PMLCA6	0x920	/* PM Local Counter Control Register 6 */
#define PMRN_PMLCA7	0x930	/* PM Local Counter Control Register 7 */


#define PMLCA_FC	(1<<(31-10))	/* Freeze Counter */
#define PMLCA_FCS	(1<<(31-11))	/* Freeze in Supervisor */
#define PMLCA_FCU	(1<<(31-12))	/* Freeze in User */
#define PMLCA_FCM1	(1<<(31-13))	/* Freeze Counter while Mark is Set */
#define PMLCA_FCM0	(1<<(31-14))	/* Freeze Cntr while Mark is Cleared */
#define PMLCA_CE	(1<<(31-15))	/* Condition Enable */

#define PMLCA_EVENT_MASK 0x0000003f	/* Event field */
#define PMLCA_EVENT_SHIFT 0

#endif /* __KERNEL__ */

#endif /* __ASM_POWERPC_REG_ACP_PMU_H__ */
