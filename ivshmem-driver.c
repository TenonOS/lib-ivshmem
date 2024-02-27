#include <uk/config.h>
#include <uk/print.h>
#include <uk/arch/types.h>
#include <uk/alloc.h>
#include <uk/intctlr.h>
#include <uk/bus/pci.h>

static struct uk_alloc *a;

#define IVSHMEM_PCI_VENDOR_ID (0x1Af4)
#define IVSHMEM_PCI_DEVICE_ID (0x1110)
#define IVSHMEM_REVISION_NUM  1
#define DRV_NAME	          "ivshmem_driver"
#define IVSHMEM_REGISTER_BAR  0
#define IVSHMEM_MEMORY_BAR    2

struct ivshmem_pci_dev {
    
    __u64 pci_base_addr;

    struct pci_device *pdev;
};

/**
 * Static function declaration.
 */
static int ivshmem_pci_add_dev(struct pci_device *pci_dev);
static int ivshmem_pci_drv_init(struct uk_alloc *drv_allocator);


static int ivshmem_pci_add_dev(struct pci_device *pci_dev){
    struct ivshmem_pci_dev *ipci_dev = NULL;
    int rc = 0;

    UK_ASSERT(pci_dev != NULL);
    uk_pr_info("Allocate ivshmem device\n");
    ipci_dev = uk_malloc(a, sizeof(*ipci_dev));
    if (!ipci_dev) {
        uk_pr_err("Failed to allocate ivshmem pci device\n");
        return -ENOMEM;
    }

    ipci_dev->pdev = pci_dev;
    ipci_dev->pci_base_addr = pci_dev->base;
}

static int ivshmem_pci_drv_init(struct uk_alloc *drv_allocator){
    if (!drv_allocator)
        return -EINVAL;
    uk_pr_info("Initialize ivshmem driver\n");
    a = drv_allocator;
    return 0;
}

static const struct pci_device_id ivshmem_pci_ids[] = {
    {PCI_DEVICE_ID(IVSHMEM_PCI_VENDOR_ID, IVSHMEM_PCI_DEVICE_ID)},
    {PCI_ANY_DEVICE_ID}
};

static struct pci_driver ivshmem_pci_drv = {
    .device_ids = ivshmem_pci_ids,
    .init = ivshmem_pci_drv_init,
    .add_dev = ivshmem_pci_add_dev
};
PCI_REGISTER_DRIVER(&ivshmem_pci_drv);
