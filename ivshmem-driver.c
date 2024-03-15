#include <uk/config.h>
#include <uk/print.h>
#include <uk/plat/common/cpu.h>
#include <uk/arch/types.h>
#include <uk/intctlr.h>
#include "./include/ivshmem-driver.h"

struct ivshmem_pci_dev *ivshmem_dev;

/**
 * Static function declaration.
 */

static inline uint32_t get_config_addr(uint32_t bus, uint32_t device, uint32_t function, uint32_t offset)
{
    return (PCI_ENABLE_BIT)
			| (bus << PCI_BUS_SHIFT)
			| (device << PCI_DEVICE_SHIFT)
			| (function << PCI_FUNCTION_SHIFT)
            | (offset);
}
/**
 * Read pci config data from addr
*/
static inline uint32_t pci_conf_read(uint32_t addr)
{
    outl(PCI_CONFIG_ADDR, addr);
    return inl(PCI_CONFIG_DATA);
}
/**
 * Write data to addr
*/
static inline void pci_conf_write(uint32_t addr, uint32_t data)
{
    outl(PCI_CONFIG_ADDR, addr);
	outl(PCI_CONFIG_DATA, data);
    return;
}

static int ivshmem_pci_add_dev(struct pci_device *pci_dev)
{
    UK_ASSERT(pci_dev != NULL);
    int rc = 0;
    uint32_t bus, device, function, command_addr, command_data;
    uint32_t bar0_addr, bar1_addr, bar2_addr, bar3_addr;
    uint32_t bar0_data, bar1_data, bar2_data, bar3_data;

    pci_dev->state = PCI_DEVICE_STATE_RUNNING;

    uk_pr_info("Allocate ivshmem device\n");
    ivshmem_dev = uk_malloc(a, sizeof(*ivshmem_dev));
    if (!ivshmem_dev) {
        uk_pr_err("Failed to allocate ivshmem pci device\n");
        return -ENOMEM;
    }

    bus = pci_dev->addr.bus;
    device = pci_dev->addr.devid;
    function = pci_dev->addr.function;

    /* Enable memory and IO */
    command_addr = get_config_addr(bus, device, function, PCI_COMMAND);
    pci_conf_write(command_addr, PCI_COMMAND_DECODE_ENABLE | PCI_COMMAND_MASTER);
    command_data = pci_conf_read(command_addr);
    uk_pr_info("command_data:0x%lx\n", command_data);

    /* Probe BAR data */
    bar0_addr = get_config_addr(bus, device, function, PCI_BASE_ADDRESS_0);
    pci_conf_write(bar0_addr, 0xffffffff);
    bar0_data = pci_conf_read(bar0_addr);

    bar2_addr = get_config_addr(bus, device, function, PCI_BASE_ADDRESS_2);
    //
	bar2_data = pci_conf_read(bar2_addr);
    uk_pr_info("bar2_data:0x%lx\n", bar2_data);

    bar3_addr = get_config_addr(bus, device, function, PCI_BASE_ADDRESS_3);
    //
	bar3_data = pci_conf_read(bar3_addr);
    uk_pr_info("bar3_data:0x%lx\n", bar3_data);

    uint64_t ivshmem_addr = (((uint64_t)bar3_data << 32) | (bar2_data & 0xfffffff0));
    /* Calculate share memory region size */
    pci_conf_write(bar2_addr, 0xffffffff);
    bar2_data = pci_conf_read(bar2_addr);
    pci_conf_write(bar3_addr, 0xffffffff);
    bar3_data = pci_conf_read(bar3_addr);
    uint64_t ivshmem_size = ~((((uint64_t)bar3_data << 32) | bar2_data) & 0xfffffffffffffff0) + 1;
    uk_pr_info("ivshmem_size:0x%lx\n", ivshmem_size);

    ivshmem_dev->pdev = pci_dev;
    ivshmem_dev->ivshmem_size = ivshmem_size;
    ivshmem_dev->ivshmem_addr_start = ivshmem_addr;   
    
    return 0;
}

static int ivshmem_pci_drv_init(struct uk_alloc *drv_allocator)
{
    if (!drv_allocator)
        return -EINVAL;
    uk_pr_info("Initialize ivshmem driver\n");
    a = drv_allocator;
    return 0;
}

static const struct pci_device_id ivshmem_pci_ids[] = {
    {PCI_DEVICE_ID(IVSHMEM_PCI_VENDOR_ID, IVSHMEM_PCI_DEVICE_ID)},
    {PCI_ANY_DEVICE_ID},
};

static struct pci_driver ivshmem_pci_drv = {
    .device_ids = ivshmem_pci_ids,
    .init = ivshmem_pci_drv_init,
    .add_dev = ivshmem_pci_add_dev
};
PCI_REGISTER_DRIVER(&ivshmem_pci_drv);
