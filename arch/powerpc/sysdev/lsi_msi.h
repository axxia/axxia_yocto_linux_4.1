/*
* LSI specific PCI-Express support;
*/

#define NR_MSI_IRQS (256)
#define MSI_MSG_DATA (0xcece)

struct acp_pci_msi {
	struct irq_domain *irqhost;
	u32 msi_addr_lo;
	u32 msi_addr_hi;
	unsigned long *acp_pci_msi_bitmap;
	spinlock_t bitmap_lock;
	void *msg_buf;
	void __iomem		*utl_base;
	struct device_node *node;
	dma_addr_t msg_buf_dma;
	int port;
};
int acp_pci_msi_irq_handle(int port);
