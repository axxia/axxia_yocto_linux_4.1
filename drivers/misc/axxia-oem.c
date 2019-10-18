/*
 *  Copyright (C) 2016 Intel Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 */

/*
  ==============================================================================
  ==============================================================================
  Private
  ==============================================================================
  ==============================================================================
*/

#include <linux/module.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/linkage.h>
#include <linux/axxia-oem.h>
#include <linux/uaccess.h>
#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/cacheflush.h>

/*
  DSP Cluster Control
*/

static ssize_t
axxia_dspc_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (0 != finished) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_dspc_get_state());

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_dspc_write(struct file *file, const char __user *buffer,
		 size_t count, loff_t *ppos)
{
	char *input;
	unsigned long mask;

	input = kmalloc(count + 1, __GFP_WAIT);

	if (NULL == input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count)) {
		kfree(input);

		return -EFAULT;
	}

	input[count] = 0;

	mask = simple_strtoul(input, NULL, 0);
	axxia_dspc_set_state((unsigned int)mask);

	kfree(input);

	return count;
}

static const struct file_operations axxia_dspc_proc_ops = {
	.read       = axxia_dspc_read,
	.write      = axxia_dspc_write,
	.llseek     = noop_llseek,
};

/*
  ACTLR_EL3 Control
*/

static ssize_t
axxia_actlr_el3_read(struct file *filp, char *buffer, size_t length,
		     loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (0 != finished) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_actlr_el3_get());

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_actlr_el3_write(struct file *file, const char __user *buffer,
		      size_t count, loff_t *ppos)
{
	char *input;

	input = kmalloc(count + 1, __GFP_WAIT);

	if (NULL == input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count)) {
		kfree(input);

		return -EFAULT;
	}

	input[count] = 0;

	axxia_actlr_el3_set(simple_strtoul(input, NULL, 0));

	kfree(input);

	return count;
}

static const struct file_operations axxia_actlr_el3_proc_ops = {
	.read       = axxia_actlr_el3_read,
	.write      = axxia_actlr_el3_write,
	.llseek     = noop_llseek,
};

/*
  ACTLR_EL2 Control
*/

static ssize_t
axxia_actlr_el2_read(struct file *filp, char *buffer, size_t length,
		     loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (0 != finished) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_actlr_el2_get());

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_actlr_el2_write(struct file *file, const char __user *buffer,
		      size_t count, loff_t *ppos)
{
	char *input;

	input = kmalloc(count + 1, __GFP_WAIT);

	if (NULL == input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count)) {
		kfree(input);

		return -EFAULT;
	}

	input[count] = 0;

	axxia_actlr_el2_set(simple_strtoul(input, NULL, 0));

	kfree(input);

	return count;
}

static const struct file_operations axxia_actlr_el2_proc_ops = {
	.read       = axxia_actlr_el2_read,
	.write      = axxia_actlr_el2_write,
	.llseek     = noop_llseek,
};

/*
  CCN Access
*/

#define DMA_X_SRC_COUNT                                0x00
#define DMA_Y_SRC_COUNT                                0x04
#define DMA_X_MODIF_SRC                                0x08
#define DMA_Y_MODIF_SRC                                0x0c
#define DMA_SRC_CUR_ADDR                       0x10
#define DMA_SRC_ACCESS                         0x14
#define    DMA_SRC_ACCESS_TAIL_LENGTH(x)       (((x) & 0xf) << 11)
#define    DMA_SRC_ACCESS_SRC_MASK_LENGTH(x)   (((x) & 0x1f) << 6)
#define    DMA_SRC_ACCESS_SRC_SIZE(x)          (((x) & 7) << 3)
#define    DMA_SRC_ACCESS_SRC_BURST(x)         (((x) & 7) << 0)
#define DMA_SRC_MASK                           0x18
#define DMA_X_DST_COUNT                                0x1c
#define DMA_Y_DST_COUNT                                0x20
#define DMA_X_MODIF_DST                                0x24
#define DMA_DST_CUR_ADDR                       0x2c
#define DMA_DST_ACCESS                         0x30
#define    DMA_DST_ACCESS_DST_SIZE(x)          (((x) & 7) << 3)
#define    DMA_DST_ACCESS_DST_BURST(x)         (((x) & 7) << 0)
#define DMA_CHANNEL_CONFIG                     0x38
#define    DMA_CONFIG_DST_SPACE(x)             (((x) & 7) << 26)
#define    DMA_CONFIG_SRC_SPACE(x)             (((x) & 7) << 23)
#define    DMA_CONFIG_PRIORITY_ROW             (1<<21)
#define    DMA_CONFIG_PRIORITY                 (1<<20)
#define    DMA_CONFIG_CH_FULL_PRIORITY          (1<<19)
#define    DMA_CONFIG_LAST_BLOCK               (1<<15)
#define    DMA_CONFIG_CLEAR_FIFO               (1<<14)
#define    DMA_CONFIG_START_MEM_LOAD           (1<<13)
#define    DMA_CONFIG_STOP_DST_EOB             (1<<11)
#define    DMA_CONFIG_FULL_DESCR_ADDR          (1<<8)
#define    DMA_CONFIG_INT_DST_EOT              (1<<7)
#define    DMA_CONFIG_INT_DST_EOB              (1<<6)
#define    DMA_CONFIG_WAIT_FOR_TASK_CNT2       (1<<5)
#define    DMA_CONFIG_TASK_CNT2_RESET          (1<<4)
#define    DMA_CONFIG_WAIT_FOR_TASK_CNT1       (1<<3)
#define    DMA_CONFIG_TASK_CNT1_RESET          (1<<2)
#define    DMA_CONFIG_TX_EN                    (1<<1)
#define    DMA_CONFIG_CHAN_EN                  (1<<0)
#define DMA_STATUS                             0x3c
#define    DMA_STATUS_WAIT_TASK_CNT2           (1<<20)
#define    DMA_STATUS_TASK_CNT2_OVERFLOW       (1<<19)
#define    DMA_STATUS_WAIT_TASK_CNT1           (1<<18)
#define    DMA_STATUS_TASK_CNT1_OVERFLOW       (1<<17)
#define    DMA_STATUS_CH_PAUS_WR_EN            (1<<16)
#define    DMA_STATUS_ERR_ACC_DESCR            (1<<14)
#define    DMA_STATUS_ERR_ACC_DST              (1<<13)
#define    DMA_STATUS_ERR_ACC_SRC              (1<<12)
#define    DMA_STATUS_ERR_OVERFLOW             (1<<9)
#define    DMA_STATUS_ERR_UNDERFLOW            (1<<8)
#define    DMA_STATUS_CH_PAUSE                 (1<<7)
#define    DMA_STATUS_CH_WAITING               (1<<5)
#define    DMA_STATUS_CH_ACTIVE                        (1<<4)
#define    DMA_STATUS_TR_COMPLETE              (1<<3)
#define    DMA_STATUS_BLK_COMPLETE             (1<<2)
#define    DMA_STATUS_UNALIGNED_READ           (1<<1)
#define    DMA_STATUS_UNALIGNED_WRITE          (1<<0)
#define    DMA_STATUS_UNALIGNED_ERR            (DMA_STATUS_UNALIGNED_READ | \
                                                DMA_STATUS_UNALIGNED_WRITE)
#define DMA_SRC_ADDR_SEG                       0x54
#define DMA_DST_ADDR_SEG                       0x58

static int ccn_oem_available = 1;
static unsigned long long ccn_phys_base;

static int
enable_ccn_access(void)
{
	unsigned long *value;
	void __iomem *gpdma0;
	void __iomem *mmap_scb;
	unsigned int gpdma0_axprot_override;
	unsigned int gpdma0_status;
	int retries = 1000;

	/*
	  Update CCN 0, bit 0
	*/

	/*
	  Map Addresses
	*/

	if (of_find_compatible_node(NULL, NULL, "lsi,axc6732")) {
		gpdma0 = ioremap(0x8005020000, SZ_64K);
	} else if (of_find_compatible_node(NULL, NULL, "lsi,axm5616")) {
		gpdma0 = ioremap(0x8004120000, SZ_64K);
	} else {
		pr_err("Only Valid for Axxia 6700 and 5600!\n");

		return -EFAULT;
	}

	mmap_scb = ioremap(0x8032000000, SZ_512K);

	/*
	  Input for the DMA Transfer
	*/

	value = kmalloc(sizeof(unsigned long), GFP_ATOMIC);
	*value = 1;
	__flush_dcache_area(value, sizeof(unsigned long));
	mb();

	/* Make sure no other transactions are in process. */
	gpdma0_status = readl(gpdma0 + DMA_STATUS);

	if (0 != (gpdma0_status & DMA_STATUS_CH_ACTIVE)) {
		iounmap(mmap_scb);
		iounmap(gpdma0);
		kfree(value);
		pr_err("Error Reading DMA_STATUS!\n");

		return -1;
	}

	/* Clear status bits. */
	writel((DMA_STATUS_TR_COMPLETE | DMA_STATUS_BLK_COMPLETE),
               gpdma0 + DMA_STATUS);

	/* Set gpdma0_axprot_override to secure or non-secure. */
	gpdma0_axprot_override = readl(mmap_scb + 0x48800);
	writel(2, mmap_scb + 0x48800);

	/* Destination is 0x80_0000_0000 */
	writel(0x80, (gpdma0 + DMA_DST_ADDR_SEG));
	writel(0, (gpdma0 + DMA_DST_CUR_ADDR));

	/* value is the source */
	writel((unsigned long)((virt_to_phys(value) & 0xff00000000) >> 32),
	       gpdma0 + DMA_SRC_ADDR_SEG);
	writel((unsigned long)((virt_to_phys(value) & 0xffffffff)),
	       gpdma0 + DMA_SRC_CUR_ADDR);

	/* Remaing setup. */
	writel(DMA_SRC_ACCESS_SRC_MASK_LENGTH(8) |
	       DMA_SRC_ACCESS_SRC_SIZE(3) |
	       DMA_SRC_ACCESS_SRC_BURST(5),
	       gpdma0 + DMA_SRC_ACCESS);
	writel(DMA_DST_ACCESS_DST_SIZE(3) |
	       DMA_DST_ACCESS_DST_BURST(5),
	       gpdma0 + DMA_DST_ACCESS);
	writel(0xffffffff, gpdma0 + DMA_SRC_MASK);
	writel(sizeof(unsigned long), gpdma0 + DMA_X_MODIF_SRC);
	writel(sizeof(unsigned long), gpdma0 + DMA_X_MODIF_DST);
	writel(1, gpdma0 + DMA_X_SRC_COUNT);
	writel(0, gpdma0 + DMA_Y_SRC_COUNT);
	writel(1, gpdma0 + DMA_X_DST_COUNT);
	writel(0, gpdma0 + DMA_Y_DST_COUNT);

	/* Start the transfer. */
	writel(DMA_CONFIG_DST_SPACE(1) |
	       DMA_CONFIG_SRC_SPACE(1) |
	       DMA_CONFIG_CH_FULL_PRIORITY |
	       DMA_CONFIG_LAST_BLOCK |
	       DMA_CONFIG_FULL_DESCR_ADDR |
	       DMA_CONFIG_WAIT_FOR_TASK_CNT2 |
	       DMA_CONFIG_WAIT_FOR_TASK_CNT1 |
	       DMA_CONFIG_TX_EN |
	       DMA_CONFIG_CHAN_EN, gpdma0 + DMA_CHANNEL_CONFIG);

	/* Wait for completion. */
	while (0 < retries--) {
		if (0 != (readl(gpdma0 + DMA_STATUS) & 0x8) &&
		    0 == readl(gpdma0 + DMA_X_DST_COUNT))
			break;

		udelay(1);
	}

	/* Restore gpdma0_axprot_override. */
	writel(gpdma0_axprot_override, (mmap_scb + 0x48800));

	if (0 >= retries) {
		iounmap(mmap_scb);
		iounmap(gpdma0);
		kfree(value);
		pr_err("gpdma_xfer timed out\n");

		return -EFAULT;
	}

	iounmap(mmap_scb);
	iounmap(gpdma0);
	kfree(value);

	return 0;
}

static unsigned long ccn_offset;

static ssize_t
axxia_ccn_offset_read(struct file *filp, char *buffer, size_t length,
		      loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (0 != finished) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", ccn_offset);

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_ccn_offset_write(struct file *file, const char __user *buffer,
		       size_t count, loff_t *ppos)
{
	char *input;
	unsigned int new_ccn_offset;

	input = kmalloc(count + 1, __GFP_WAIT);

	if (NULL == input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count)) {
		kfree(input);

		return -EFAULT;
	}

	input[count] = 0;

	new_ccn_offset = (unsigned int)simple_strtoul(input, NULL, 0);

	if (of_find_compatible_node(NULL, NULL, "lsi,axc6732")) {
		if (0x9cff00 < new_ccn_offset)
			pr_err("Invalid CCN Offset!\n");
	} else if (of_find_compatible_node(NULL, NULL, "lsi,axm5616")) {
		if (0x94ff00 < new_ccn_offset)
			pr_err("Invalid CCN Offset!\n");
	} else {
		pr_err("Internal Error!\n");
	}

	ccn_offset = (unsigned int)new_ccn_offset;

	kfree(input);

	return count;
}

static const struct file_operations axxia_ccn_offset_proc_ops = {
	.read       = axxia_ccn_offset_read,
	.write      = axxia_ccn_offset_write,
	.llseek     = noop_llseek,
};

static ssize_t
axxia_ccn_value_read(struct file *filp, char *buffer, size_t length,
		     loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (0 != finished) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_ccn_get(ccn_offset));

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_ccn_value_write(struct file *file, const char __user *buffer,
		       size_t count, loff_t *ppos)
{
	char *input;

	input = kmalloc(count + 1, __GFP_WAIT);

	if (NULL == input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count)) {
		kfree(input);

		return -EFAULT;
	}

	input[count] = 0;

	axxia_ccn_set(ccn_offset, simple_strtoul(input, NULL, 0));

	kfree(input);

	return count;
}

static const struct file_operations axxia_ccn_value_proc_ops = {
	.read       = axxia_ccn_value_read,
	.write      = axxia_ccn_value_write,
	.llseek     = noop_llseek,
};

/*
  ------------------------------------------------------------------------------
  setup_ccn_proc
*/

static int
setup_ccn_proc(void)
{
	int rc = 0;

	if (of_find_compatible_node(NULL, NULL, "lsi,axc6732")) {
		ccn_phys_base = 0x4000000000ULL;
	} else if (of_find_compatible_node(NULL, NULL, "lsi,axm5616")) {
		ccn_phys_base = 0x8000000000ULL;
	} else {
		pr_err("Only Valid for Axxia 6700 and 5600!\n");

		return -EFAULT;
	}

	ccn_offset = 0;

	if (proc_create("driver/axxia_ccn_offset", 0200, NULL,
			&axxia_ccn_offset_proc_ops) == NULL) {
		pr_err("Could not create /proc/driver/axxia_ccn_offset!\n");
		rc = -EFAULT;
	} else if (proc_create("driver/axxia_ccn_value", 0200, NULL,
			       &axxia_ccn_value_proc_ops) == NULL) {
		pr_err("Could not create /proc/driver/axxia_ccn_value!\n");
		rc = -EFAULT;
	}

	if (rc) {
		pr_err("Axxia CCN Access is Not Available!\n");
	} else {
		if (ccn_oem_available)
			pr_info("Axxia CCN (scm) Initialized\n");
		else
			pr_info("Axxia CCN (direct) Initialized\n");
	}

	return rc;
}

/*
  ==============================================================================
  ==============================================================================
  Public
  ==============================================================================
  ==============================================================================
*/

/*
  ------------------------------------------------------------------------------
  axxia_dspc_get_state
*/

unsigned long
axxia_dspc_get_state(void)
{
	struct arm_smccc_res res;
	u64 function_id = 0xc3000000;
	u64 arg0 = 0;
	u64 arg1 = 0;
	u64 arg2 = 0;

	__arm_smccc_smc(function_id, arg0, arg1, arg2, &res);

	if (0 != res.a0)
		pr_warn("Getting the DSP State Failed!\n");

	return res.a1;
}
EXPORT_SYMBOL(axxia_dspc_get_state);

/*
  ------------------------------------------------------------------------------
  axxia_dspc_set_state
*/

void
axxia_dspc_set_state(unsigned long state)
{
	struct arm_smccc_res res;
	u64 function_id = 0xc3000001;
	u64 arg0 = state;
	u64 arg1 = 0;
	u64 arg2 = 0;

	__arm_smccc_smc(function_id, arg0, arg1, arg2, &res);

	if (0 != res.a0)
		pr_warn("Getting the DSP State Failed!\n");

	return;
}
EXPORT_SYMBOL(axxia_dspc_set_state);

/*
  ------------------------------------------------------------------------------
  axxia_actlr_el3_get
*/

unsigned long
axxia_actlr_el3_get(void)
{
	struct arm_smccc_res res;
	u64 function_id = 0xc3000002;
	u64 arg0 = 0;
	u64 arg1 = 0;
	u64 arg2 = 0;

	__arm_smccc_smc(function_id, arg0, arg1, arg2, &res);

	if (0 != res.a0)
		pr_warn("Getting ACTLR_EL3 Failed!\n");

	return res.a1;
}
EXPORT_SYMBOL(axxia_actlr_el3_get);

/*
  ------------------------------------------------------------------------------
  axxia_actlr_el3_set
*/

void
axxia_actlr_el3_set(unsigned long input)
{
	struct arm_smccc_res res;
	u64 function_id = 0xc3000003;
	u64 arg0 = input;
	u64 arg1 = 0;
	u64 arg2 = 0;

	__arm_smccc_smc(function_id, arg0, arg1, arg2, &res);

	if (0 != res.a0)
		pr_warn("Setting ACTLR_EL3 Failed!\n");

	return;
}
EXPORT_SYMBOL(axxia_actlr_el3_set);

/*
  ------------------------------------------------------------------------------
  axxia_actlr_el2_get
*/

unsigned long
axxia_actlr_el2_get(void)
{
	struct arm_smccc_res res;
	u64 function_id = 0xc3000004;
	u64 arg0 = 0;
	u64 arg1 = 0;
	u64 arg2 = 0;

	__arm_smccc_smc(function_id, arg0, arg1, arg2, &res);

	if (0 != res.a0)
		pr_warn("Getting ACTLR_EL2 Failed!\n");

	return res.a1;
}
EXPORT_SYMBOL(axxia_actlr_el2_get);

/*
  ------------------------------------------------------------------------------
  axxia_actlr_el2_set
*/

void
axxia_actlr_el2_set(unsigned long input)
{
	struct arm_smccc_res res;
	u64 function_id = 0xc3000005;
	u64 arg0 = input;
	u64 arg1 = 0;
	u64 arg2 = 0;

	__arm_smccc_smc(function_id, arg0, arg1, arg2, &res);

	if (0 != res.a0)
		pr_warn("Setting ACTLR_EL2 Failed!\n");

	return;
}
EXPORT_SYMBOL(axxia_actlr_el2_set);

/*
  ------------------------------------------------------------------------------
  axxia_ccn_get
*/

unsigned long
axxia_ccn_get(unsigned int offset)
{
	struct arm_smccc_res res;

	if (0 == ccn_oem_available) {
		void __iomem *ccn;

		ccn = ioremap(ccn_phys_base +
			      ((offset / SZ_1K) * SZ_1K), SZ_1K);
		res.a1 = readq(ccn + (offset % SZ_1K));
		iounmap(ccn);
	} else {
		u64 function_id = 0xc3000006;
		u64 arg0 = offset;
		u64 arg1 = 0;
		u64 arg2 = 0;

		__arm_smccc_smc(function_id, arg0, arg1, arg2, &res);

		if (0 != res.a0)
			pr_warn("Getting CCN Register Failed!\n");
	}

	return res.a1;
}
EXPORT_SYMBOL(axxia_ccn_get);

/*
  ------------------------------------------------------------------------------
  axxia_ccn_set
*/

void
axxia_ccn_set(unsigned int offset, unsigned long value)
{
	struct arm_smccc_res res;

	if (0 == ccn_oem_available) {
		void __iomem *ccn;

		ccn = ioremap(ccn_phys_base +
			      ((offset / SZ_1K) * SZ_1K), SZ_1K);
		writeq(value, (ccn + (offset % SZ_1K)));
		iounmap(ccn);
	} else {
		u64 function_id = 0xc3000007;
		u64 arg0 = offset;
		u64 arg1 = value;
		u64 arg2 = 0;

		__arm_smccc_smc(function_id, arg0, arg1, arg2, &res);

		if (0 != res.a0)
			pr_warn("Getting CCN Register Failed!\n");
	}

	return;
}
EXPORT_SYMBOL(axxia_ccn_set);

/*
  ------------------------------------------------------------------------------
  axxia_dspc_init
*/

static int
axxia_oem_init(void)
{
	struct arm_smccc_res res;
	int rc = 0;

	if (of_find_compatible_node(NULL, NULL, "lsi,axc6732")) {
		/* Only applicable to the 6700. */
		if (NULL == proc_create("driver/axxia_dspc", S_IWUSR, NULL,
					&axxia_dspc_proc_ops))
			pr_err("Could not create /proc/driver/axxia_dspc!\n");
		else
			pr_info("Axxia DSP Control Initialized\n");
	}

	if (NULL == proc_create("driver/axxia_actlr_el3", S_IWUSR, NULL,
				&axxia_actlr_el3_proc_ops))
		pr_err("Could not create /proc/driver/axxia_actlr_el3!\n");
	else
		pr_info("Axxia ACTLR_EL3 Control Initialized\n");

	if (NULL == proc_create("driver/axxia_actlr_el2", S_IWUSR, NULL,
				&axxia_actlr_el2_proc_ops))
		pr_err("Could not create /proc/driver/axxia_actlr_el2!\n");
	else
		pr_info("Axxia ACTLR_EL3 Control Initialized\n");

	/*
	  In some cases, CCN access is required (SBB work around for
	  example) but OEM access is not available (version of ATF
	  before 1.38).  As it happens, in U-Boot versions prior to
	  1.79, the hni_axprot_override is set.  This allows the
	  cores, while non-secure, to access the SCB blocks.  The
	  following provides CCN acces to non-secure masters by doing
	  the following:

	  -1-
	  Check for OEM access.  If available, skip the reset.

	  -2-
	  If OEM access is not avialable, try to enable non-secure
	  access by setting bit 0 of CCN offset 0 (the 'secure_acces'
	  bit).  See enable_ccn_access() above for details.
	*/

	__arm_smccc_smc(0xc3000006, 0xfe0, 0, 0, &res);

	if (0 != res.a0) {
		ccn_oem_available = 0;
		rc = enable_ccn_access();

		if (rc)
			pr_err("Unable to Enable CCN Access via GPDMA!\n");
	}

	if (!rc)
		rc = setup_ccn_proc();

	return rc;
}

device_initcall(axxia_oem_init);

MODULE_AUTHOR("John Jacques <john.jacques@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Axxia OEM Control");
