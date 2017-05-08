#ifndef __LINUX_ARM_SMCCC_H
#define __LINUX_ARM_SMCCC_H

#include <linux/linkage.h>
#include <linux/types.h>

/*
 * This file provides common defines for ARM SMC Calling Convention as
 * specified in
 * http://infocenter.arm.com/help/topic/com.arm.doc.den0028a/index.html
 */

#define ARM_SMCCC_STD_CALL              0
#define ARM_SMCCC_FAST_CALL             1
#define ARM_SMCCC_TYPE_SHIFT            31

#define ARM_SMCCC_SMC_32                0
#define ARM_SMCCC_SMC_64                1
#define ARM_SMCCC_CALL_CONV_SHIFT       30

#define ARM_SMCCC_OWNER_MASK            0x3F
#define ARM_SMCCC_OWNER_SHIFT           24

#define ARM_SMCCC_FUNC_MASK             0xFFFF

#define ARM_SMCCC_IS_FAST_CALL(smc_val) \
	((smc_val) & (ARM_SMCCC_FAST_CALL << ARM_SMCCC_TYPE_SHIFT))
#define ARM_SMCCC_IS_64(smc_val) \
	((smc_val) & (ARM_SMCCC_SMC_64 << ARM_SMCCC_CALL_CONV_SHIFT))
#define ARM_SMCCC_FUNC_NUM(smc_val)     ((smc_val) & ARM_SMCCC_FUNC_MASK)
#define ARM_SMCCC_OWNER_NUM(smc_val) \
	(((smc_val) >> ARM_SMCCC_OWNER_SHIFT) & ARM_SMCCC_OWNER_MASK)

#define ARM_SMCCC_CALL_VAL(type, calling_convention, owner, func_num) \
	(((type) << ARM_SMCCC_TYPE_SHIFT) | \
	((calling_convention) << ARM_SMCCC_CALL_CONV_SHIFT) | \
	(((owner) & ARM_SMCCC_OWNER_MASK) << ARM_SMCCC_OWNER_SHIFT) | \
	((func_num) & ARM_SMCCC_FUNC_MASK))

#define ARM_SMCCC_OWNER_ARCH            0
#define ARM_SMCCC_OWNER_CPU             1
#define ARM_SMCCC_OWNER_SIP             2
#define ARM_SMCCC_OWNER_OEM             3
#define ARM_SMCCC_OWNER_STANDARD        4
#define ARM_SMCCC_OWNER_TRUSTED_APP     48
#define ARM_SMCCC_OWNER_TRUSTED_APP_END 49
#define ARM_SMCCC_OWNER_TRUSTED_OS      50
#define ARM_SMCCC_OWNER_TRUSTED_OS_END  63

struct arm_smccc_res {
	u64 a0;
	u64 a1;
	u64 a2;
	u64 a3;
};


asmlinkage u64 __arm_smccc_hvc(u64 a0, u64 a1,
				u64 a2, u64 a3,
				struct arm_smccc_res *ptr);

asmlinkage u64 __arm_smccc_smc(u64 a0, u64 a1,
				u64 a2, u64 a3,
				struct arm_smccc_res *ptr);

#endif /*__LINUX_ARM_SMCCC_H*/
