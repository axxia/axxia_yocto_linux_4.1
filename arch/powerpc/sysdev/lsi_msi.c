/*
* LSI specific PCI-Express MSI support;
*/

#undef DEBUG

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/bootmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include <linux/io.h>
#include <asm/pci-bridge.h>
#include <asm/machdep.h>
#include <asm/dcr.h>
#include <asm/dcr-regs.h>
#include <mm/mmu_decl.h>

#include "../../drivers/misc/lsi-ncr.h"
#include "lsi_msi.h"

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/bitmap.h>
#include <linux/msi.h>
#include <linux/of_platform.h>
#include <asm/prom.h>
#include <asm/hw_irq.h>
#include <asm/ppc-pci.h>

#define MAX_NUMBER_PCI_SLOTS 3
static struct acp_pci_msi *acp_pci_msi[MAX_NUMBER_PCI_SLOTS];

static void acp_pci_msi_end_irq(struct irq_data *d)
{
}

static struct irq_chip acp_pci_msi_chip = {
	.irq_mask       = mask_msi_irq,
	.irq_unmask     = unmask_msi_irq,
	.irq_ack        = acp_pci_msi_end_irq,
	.name   = " ACP-MSI  ",
};

static int acp_pci_msi_host_map(struct irq_domain *h, unsigned int virq,
	irq_hw_number_t hw)
{
	struct irq_chip *chip = &acp_pci_msi_chip;
	irq_set_status_flags(virq, IRQ_TYPE_EDGE_FALLING);
	irq_set_chip_and_handler(virq, chip, handle_edge_irq);
	return 0;
}

static struct irq_domain_ops acp_pci_msi_host_ops = {
	.map = acp_pci_msi_host_map,
};

static void acp_pci_msi_free_hwirqs(struct acp_pci_msi *msi, int offset,
	int num)
{
	unsigned long flags;
	int order = get_count_order(num);

	pr_debug("%s: freeing 0x%x (2^%d) at offset 0x%x\n",
		__func__, num, order, offset);

	spin_lock_irqsave(&msi->bitmap_lock, flags);
	bitmap_release_region(msi->acp_pci_msi_bitmap, offset, order);
	spin_unlock_irqrestore(&msi->bitmap_lock, flags);
}

static int acp_pci_msi_free_dt_hwirqs(struct acp_pci_msi *msi_data)
{
	bitmap_allocate_region(msi_data->acp_pci_msi_bitmap, 0,
		get_count_order(NR_MSI_IRQS));

	acp_pci_msi_free_hwirqs(msi_data, 0, 0x100);
	return 0;
}

static int acp_pci_msi_init_allocator(struct acp_pci_msi *msi_data)
{
	int rc;
	int size = BITS_TO_LONGS(NR_MSI_IRQS) * sizeof(u32);

	msi_data->acp_pci_msi_bitmap = kzalloc(size, GFP_KERNEL);
	if (msi_data->acp_pci_msi_bitmap == NULL) {
		pr_debug("%s: out of memory at line %d\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	rc = acp_pci_msi_free_dt_hwirqs(msi_data);
	if (rc)
		goto out_free;

	return 0;
out_free:
	kfree(msi_data->acp_pci_msi_bitmap);
	msi_data->acp_pci_msi_bitmap = NULL;
	return rc;
}

static int acp_pci_msi_hw_init(struct acp_pci_msi *msi,
	struct platform_device *pdev)
{
	u32 int_enable;
	u32 msg_phys_adrs;
	int rc, i;

	if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(64)) != 0) {
		if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(32)) != 0) {
			dev_err(&pdev->dev, "No memory for MSI table\n");
			rc = -ENOMEM;
			return rc;
		} else {
			dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
		}
	} else {
		dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64));
	}

	msi->msg_buf = dma_alloc_coherent(&pdev->dev, PAGE_SIZE,
		&msi->msg_buf_dma, GFP_KERNEL);
	if (!msi->msg_buf) {
		pr_debug("%s: out of memory at line %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		return rc;
	}

	msi->msi_addr_hi = 0;
	/* Align address to 1K */
	if (msi->msg_buf_dma & 0x3f)
		msi->msg_buf_dma = (msi->msg_buf_dma + 0x400) & 0xfffffc00;
	msi->msi_addr_lo = msi->msg_buf_dma;

	/* Set MSI buffer address. */
	msg_phys_adrs = msi->msg_buf_dma & 0x0fffffff;

	pr_debug("%s: MSI buffer address = x%x\n", __func__, msg_phys_adrs);
	printk(KERN_INFO "%s: MSI buffer address = x%x\n", __func__,
		msg_phys_adrs);
	out_le32(msi->utl_base + 0x1190, (msg_phys_adrs) >> 10);

	/* Enable MSI interrupts. */
	int_enable = in_le32(msi->utl_base + 0x10C4);
	int_enable |= (1 << 31);
	out_le32(msi->utl_base + 0x10C4, int_enable);
	out_le32(msi->utl_base + 0x1234, 0xffff);
	for (i = 0; i < 16; i++)
		out_le32(msi->utl_base + 0x1240 + (i*0xc), 0xffff);

	return 0;
}

static int acp_pci_msi_setup_irqs(struct pci_dev *pdev, int nvec, int type);
static void acp_pci_msi_teardown_irqs(struct pci_dev *pdev);
static int acp_pci_msi_check_device(struct pci_dev *pdev, int nvec, int type);

static int acp_pci_msi_init(struct platform_device *pdev, int port)
{
	int rc;
	struct acp_pci_msi *msi;

	msi = acp_pci_msi[port];

	msi->irqhost = irq_domain_add_linear(msi->node,
		NR_MSI_IRQS, &acp_pci_msi_host_ops, 0);

	if (msi->irqhost == NULL) {
		pr_debug("%s: out of memory at line %d\n",
			__func__, __LINE__);
		rc = -ENOMEM;
		goto error_out;
	}

	rc = acp_pci_msi_init_allocator(msi);
	if (rc)
		goto error_out;

	rc = acp_pci_msi_hw_init(msi, pdev);
	if (rc != 0)
		goto error_out;

	spin_lock_init(&msi->bitmap_lock);
	acp_pci_msi[port] = msi;
	ppc_md.setup_msi_irqs = acp_pci_msi_setup_irqs;
	ppc_md.teardown_msi_irqs = acp_pci_msi_teardown_irqs;
	ppc_md.msi_check_device = acp_pci_msi_check_device;

	return 0;

error_out:
	kfree(msi);
	return rc;
}

static int lsi_msi_probe(struct platform_device *pdev)
{

	const u32 *pval;
	int rc = 0;
	struct device_node *np = pdev->dev.of_node;
	struct acp_pci_msi *pci_msi;
	u32 int_enable;

	pci_msi = kzalloc(sizeof(struct acp_pci_msi), GFP_KERNEL);
	if (!pci_msi) {
		rc = -ENOMEM;
		return -1;
	}

	pci_msi->node = of_node_get(np);

	/* Get the port number from the device-tree */
	pval = of_get_property(np, "port", NULL);
	if (pval == NULL) {
		printk(KERN_ERR "PCIE: Can't find port number for %s\n",
			np->full_name);
		return -1;
	}
	pci_msi->port = *pval;

	pci_msi->utl_base = of_iomap(np, 0);
	if (!pci_msi->utl_base) {
		printk(KERN_ERR "No memory for utl_base\n");
		return -1;
	}
	int_enable = in_le32(pci_msi->utl_base + 0x1004);

	acp_pci_msi[pci_msi->port] = pci_msi;

	acp_pci_msi_init(pdev, pci_msi->port);

	return 0;
}

static int lsi_msi_remove(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
	return 0;
}

int acp_pci_msi_irq_handle(int port)
{
	struct acp_pci_msi *msi = acp_pci_msi[port];
	u32 *msg = (u32 *) msi->msg_buf;
	int rc = -1;
	int i;

	/*
	 * Check each message slot.  If the slot is not zero,
	 * then handle the associated virq.
	 */
	for (i = 0; i < 256; i++) {
		if (msg[i] != 0) {
			pr_debug("%s msg[%d] = x%x\n", __func__, i, msg[i]);
			msg[i] = 0;
			generic_handle_irq(i);
			rc = 0;
		}
	}
	/* Clear MSI interrupts */
	out_le32(msi->utl_base + 0x1230, 0xffff);
	for (i = 0; i < 16; i++)
		out_le32(msi->utl_base + 0x123c + (i*0xc), 0xffff);
	return rc;
}
EXPORT_SYMBOL_GPL(acp_pci_msi_irq_handle);

static irq_hw_number_t acp_pci_msi_alloc_hwirqs(struct acp_pci_msi *msi,
	int num)
{
	unsigned long flags;
	int order = get_count_order(num);
	int offset;

	spin_lock_irqsave(&msi->bitmap_lock, flags);
	offset = bitmap_find_free_region(msi->acp_pci_msi_bitmap,
		NR_MSI_IRQS, order);

	spin_unlock_irqrestore(&msi->bitmap_lock, flags);

	pr_debug("%s: allocated 0x%x (2^%d) at offset 0x%x\n",
		__func__, num, order, offset);

	return offset;
}

static int acp_pci_msi_check_device(struct pci_dev *pdev, int nvec, int type)
{
	int rc = 0;
	return rc;
}

static void acp_pci_msi_teardown_irqs(struct pci_dev *pdev)
{
	struct msi_desc *entry;
	struct pci_bus *bus = pdev->bus;
	struct pci_controller *hose = (struct pci_controller *) bus->sysdata;
	struct acp_pci_msi *msi_data = acp_pci_msi[hose->indirect_type];

	list_for_each_entry(entry, &pdev->msi_list, list) {
		if (entry->irq == NO_IRQ)
			continue;
		irq_set_msi_desc(entry->irq, NULL);
		acp_pci_msi_free_hwirqs(msi_data, virq_to_hw(entry->irq), 1);
		irq_dispose_mapping(entry->irq);
	}
	return;
}

static int acp_pci_msi_setup_irqs(struct pci_dev *pdev, int nvec, int type)
{
	irq_hw_number_t hwirq;
	int rc;
	unsigned int virq;
	struct msi_desc *entry;
	struct msi_msg msg;
	struct pci_bus *bus = pdev->bus;
	struct pci_controller *hose = (struct pci_controller *) bus->sysdata;
	struct acp_pci_msi *msi = acp_pci_msi[hose->indirect_type];

	list_for_each_entry(entry, &pdev->msi_list, list) {
		hwirq = acp_pci_msi_alloc_hwirqs(msi, 1);
		if (hwirq < 0) {
			rc = hwirq;
			pr_debug("%s: fail allocating msi interrupt\n",
				__func__);
			goto out_free;
		}

		virq = irq_create_mapping(msi->irqhost, hwirq);
		if ((virq == NO_IRQ) || (virq >= NR_MSI_IRQS)) {
			pr_debug("%s: fail mapping hwirq 0x%lx\n",
				__func__, hwirq);
			acp_pci_msi_free_hwirqs(msi, hwirq, 1);
			rc = -ENOSPC;
			goto out_free;
		}

		pr_debug("%s: hwirq = %d, virq = %d entry = %p\n", __func__,
			(int) hwirq, virq, entry);
		printk(KERN_INFO "%s: hwirq = %d, virq = %d entry = %p\n",
			__func__,
			(int) hwirq, virq, entry);
		irq_set_msi_desc(virq, entry);

		msg.address_hi = msi->msi_addr_hi;
		msg.address_lo = msi->msi_addr_lo + (virq * sizeof(u32));
		msg.data = MSI_MSG_DATA;
		pr_debug("%s: msi_msg: hi = x%x, lo = x%x, data = x%x\n",
			__func__, msg.address_hi, msg.address_lo, msg.data);
		printk(KERN_INFO "%s: msi_msg: hi = x%x, lo = x%x, data = x%x\n",
			__func__, msg.address_hi, msg.address_lo, msg.data);
		write_msi_msg(virq, &msg);
	}
	return 0;

out_free:
	return rc;
}

static const struct of_device_id lsi_msi_match[] = {
	{.compatible = "lsi,msi"},
	{}
};

MODULE_DEVICE_TABLE(of, lsi_msi_match);

static struct platform_driver lsi_msi_driver = {
	.driver = {
		.name = "lsi_msi",
		.owner = THIS_MODULE,
		.of_match_table = lsi_msi_match,
		},
	.probe = lsi_msi_probe,
	.remove = lsi_msi_remove,
};

module_platform_driver(lsi_msi_driver);
MODULE_AUTHOR("Sangeetha Rao <sangeetha.rao@avagotech.com>");
MODULE_DESCRIPTION("LSI/Avago PCIe MSI Driver");
