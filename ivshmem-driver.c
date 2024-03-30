#include <uk/config.h>
#include <uk/print.h>
#include <uk/plat/common/cpu.h>
#include <uk/arch/types.h>
#include <uk/intctlr.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <uk/essentials.h>
#include <vfscore/uio.h>
#include <devfs/device.h>
#include "./include/ivshmem-driver.h"

#define DEV_IVSHMEM_NAME "ivshmem"

struct ivshmem_pci_dev *ivshmem_dev;



/**
 * Static function declaration.
 */

static inline __u32 get_config_addr(__u32 bus, __u32 device, __u32 function, __u32 offset)
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
static inline __u32 pci_conf_read(__u32 addr)
{
    outl(PCI_CONFIG_ADDR, addr);
    return inl(PCI_CONFIG_DATA);
}
/**
 * Write data to addr
*/
static inline void pci_conf_write(__u32 addr, __u32 data)
{
    outl(PCI_CONFIG_ADDR, addr);
	outl(PCI_CONFIG_DATA, data);
    return;
}

static int ivshmem_pci_add_dev(struct pci_device *pci_dev)
{
    UK_ASSERT(pci_dev != NULL);
    int rc = 0;
    __u32 bus, device, function, command_addr, command_data;
    __u32 devreg_addr, bar1_data, bar2_data, bar3_data;
    __u32 int_mask, int_stat, IVPosition, doorbell;

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
    ivshmem_dev->bar_addr[0] = get_config_addr(bus, device, function, PCI_BASE_ADDRESS_0);
    devreg_addr = pci_conf_read(ivshmem_dev->bar_addr[0]);
    uk_pr_info("bar0_data:0x%lx\n", devreg_addr);
    
    /* Probe interrupt info */
    int_mask = readl(devreg_addr);
    uk_pr_info("interrupt mask:0x%lx\n", int_mask);
    int_stat = readl(devreg_addr | 0x4);
    uk_pr_info("interrupt status:0x%lx\n", int_stat);
    IVPosition = readl(devreg_addr | 0x8);
    uk_pr_info("IVPosition:0x%lx\n", IVPosition);

    ivshmem_dev->bar_addr[1] = get_config_addr(bus, device, function, PCI_BASE_ADDRESS_1);
    bar1_data = pci_conf_read(ivshmem_dev->bar_addr[1]);
    uk_pr_info("bar1_data:0x%lx\n", bar1_data);

    ivshmem_dev->bar_addr[2] = get_config_addr(bus, device, function, PCI_BASE_ADDRESS_2);
	bar2_data = pci_conf_read(ivshmem_dev->bar_addr[2]);
    uk_pr_info("bar2_data:0x%lx\n", bar2_data);

    ivshmem_dev->bar_addr[3] = get_config_addr(bus, device, function, PCI_BASE_ADDRESS_3);
	bar3_data = pci_conf_read(ivshmem_dev->bar_addr[3]);
    uk_pr_info("bar3_data:0x%lx\n", bar3_data);

    __u64 ivshmem_addr = (((__u64)bar3_data << 32) | (bar2_data & 0xfffffff0));
    __u64 ivshmem_size = 4194304;
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



static int dev_ivshmem_read(struct device *dev __unused, struct uio *uio,
			   int flags __unused)
{
	size_t count;
	char *buf;

	buf = uio->uio_iov->iov_base;
	count = uio->uio_iov->iov_len;

	uk_swrand_fill_buffer(buf, count);

	uio->uio_resid = 0;
	return 0;
}

static int dev_ivshmem_write(struct device *dev __unused,
			    struct uio *uio __unused,
			    int flags __unused)
{
	return EPERM;
}

static struct devops ivshmem_devops = {
	.open = dev_noop_open,
	.close = dev_noop_close,
	.read = dev_ivshmem_read,
	.write = dev_ivshmem_write,
	.ioctl = dev_noop_ioctl,
};

static struct driver drv_ivshmem = {
	.devops = &ivshmem_devops,
	.devsz = 0,
	.name = DEV_IVSHMEM_NAME
};

static int devfs_register(struct uk_init_ctx *ictx __unused)
{
	int rc;

	uk_pr_info("Register '%s' to devfs\n", DEV_IVSHMEM_NAME);

	/* register /dev/ivshmem */
	rc = device_create(&drv_ivshmem, DEV_IVSHMEM_NAME, D_CHR, NULL);
	if (unlikely(rc)) {
		uk_pr_err("Failed to register '%s' to devfs: %d\n",
			  DEV_IVSHMEM_NAME, rc);
		return -rc;
	}

	return 0;
}

devfs_initcall(devfs_register);