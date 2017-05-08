#include <linux/linkage.h>


/* Current ATF support up to 4 return argument only. */

.macro SMCCC instr
        .cfi_startproc
	\instr #0
	/* if x4/ptr is zero, skip copying x1-x3 */
	cmp x4, #0
	b.eq 1f
	stp x0, x1, [x4]
	stp x2, x3, [x4, #16]
1:
	ret
        .cfi_endproc
.endm


/*asmlinkage u64 __arm_smccc_hvc(u64 a0, u64 a1,
				u64 a2, u64 a3,
				struct arm_smccc_res *ptr); */
ENTRY(__arm_smccc_hvc)
	SMCCC hvc
ENDPROC(__arm_smccc_hvc)


/* asmlinkage u64 __arm_smccc_smc(u64 a0, u64 a1,
				u64 a2, u64 a3,
				struct arm_smccc_res *ptr); */
ENTRY(__arm_smccc_smc)
	SMCCC smc
ENDPROC(__arm_smccc_smc)
