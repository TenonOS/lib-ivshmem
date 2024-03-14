#include <uk/bus/pci.h>
#include <uk/alloc.h>

static struct uk_alloc *a;

#define IVSHMEM_PCI_VENDOR_ID (0x1Af4)
#define IVSHMEM_PCI_DEVICE_ID (0x1110)
#define IVSHMEM_REVISION_NUM  1
#define DRV_NAME	          "ivshmem_driver"
#define IVSHMEM_REGISTER_BAR  0
#define IVSHMEM_MEMORY_BAR    2

struct ivshmem_pci_dev {
	/* BAR2 region */
    __u64 *ivshmem_addr_start;
    __u64 ivshmem_size;
	/* Pci device information */
    struct pci_device *pdev;
};
