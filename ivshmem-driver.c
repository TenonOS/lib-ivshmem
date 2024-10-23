#include <uk/config.h>
#include <uk/print.h>
#include <uk/plat/common/cpu.h>
#include <uk/arch/types.h>
#include <uk/intctlr.h>
#include <uk/essentials.h>
#include <vfscore/uio.h>
#include <devfs/device.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <ivshmem-driver.h>

struct ivshmem_pci_dev *ivshmem_pdev;
int noraml_ivshmem_init = 0;
struct ivshmem_pci_dev *ivshmem_pdev_cxl;

#define DEV_IVSHMEM_NAME "ivshmem"

/**
 * Static function declaration.
 */

static inline __u32 get_config_addr(struct pci_address addr, __u32 offset)
{
    return (PCI_ENABLE_BIT)
			| (addr.bus << PCI_BUS_SHIFT)
			| (addr.devid << PCI_DEVICE_SHIFT)
			| (addr.function << PCI_FUNCTION_SHIFT)
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
    UK_ASSERT(pci_dev);
    if (!noraml_ivshmem_init) {
        noraml_ivshmem_init = 1;
        __u32 command_addr, command_data;
        __u32 devreg_addr, bar1_data, bar2_data, bar3_data;
        pci_dev->state = PCI_DEVICE_STATE_RUNNING;
        uk_pr_info("Allocate ivshmem device\n");
        ivshmem_pdev = uk_malloc(a, sizeof(*ivshmem_pdev));
        if (!ivshmem_pdev) {
            uk_pr_err("Failed to allocate ivshmem pci device\n");
            return -ENOMEM;
        }
        /* Enable memory and IO */
        command_addr = get_config_addr(pci_dev->addr, PCI_COMMAND);
        pci_conf_write(command_addr, PCI_COMMAND_DECODE_ENABLE | PCI_COMMAND_MASTER);
        command_data = pci_conf_read(command_addr);
        uk_pr_info("command_data:0x%x\n", command_data);
        /* Probe revision id */
        __u32 class_addr = get_config_addr(pci_dev->addr, PCI_CONF_CLASS_ID);
        ivshmem_pdev->revision = pci_conf_read(class_addr) & PCI_CLASS_REVISION;
        /* Probe BAR data */
        ivshmem_pdev->bar_addr[0] = get_config_addr(pci_dev->addr, PCI_BASE_ADDRESS_0);
        devreg_addr = pci_conf_read(ivshmem_pdev->bar_addr[0]);
        uk_pr_info("bar0_data:0x%x\n", devreg_addr);

        /* Probe interrupt info */
        __u32 int_mask, int_stat, IVPosition;
        int_mask = readl((__u32 *)devreg_addr);
        uk_pr_info("interrupt mask:0x%x\n", int_mask);
        int_stat = readl((__u32 *)(devreg_addr | 0x4));
        uk_pr_info("interrupt status:0x%x\n", int_stat);
        IVPosition = readl((__u32 *)(devreg_addr | 0x8));
        uk_pr_info("IVPosition:0x%x\n", IVPosition);

        ivshmem_pdev->bar_addr[1] = get_config_addr(pci_dev->addr, PCI_BASE_ADDRESS_1);
        bar1_data = pci_conf_read(ivshmem_pdev->bar_addr[1]);
        uk_pr_info("bar1_data:0x%x\n", bar1_data);

        ivshmem_pdev->bar_addr[2] = get_config_addr(pci_dev->addr, PCI_BASE_ADDRESS_2);
	    bar2_data = pci_conf_read(ivshmem_pdev->bar_addr[2]);
        uk_pr_info("bar2_data:0x%x\n", bar2_data);

        ivshmem_pdev->bar_addr[3] = get_config_addr(pci_dev->addr, PCI_BASE_ADDRESS_3);
	    bar3_data = pci_conf_read(ivshmem_pdev->bar_addr[3]);
        uk_pr_info("bar3_data:0x%x\n", bar3_data);

        __u64 ivshmem_addr = (((__u64)bar3_data << 32) | (bar2_data & 0xfffffff0));
        __u64 ivshmem_size = 0x40000000;
        uk_pr_info("ivshmem_size:0x%x\n", ivshmem_size);
    // int rc = 0;
    // rc = uk_intctlr_irq_register(ivshmem_pdev->pdev->irq, ivshmem_irq_handle,
    //          ivshmem_pdev);
    // if (rc != 0) {
	// 	uk_pr_err("Failed to register the interrupt\n");
	// 	return rc;
	// }
        ivshmem_pdev->pdev = pci_dev;
        ivshmem_pdev->ivshmem_size = ivshmem_size;
        ivshmem_pdev->ivshmem_addr_start = ivshmem_addr;
        ivshmem_pdev->ivposition = IVPosition;  
    } else {
        __u32 command_addr, command_data;
        __u32 devreg_addr, bar1_data, bar2_data, bar3_data;
        pci_dev->state = PCI_DEVICE_STATE_RUNNING;
        uk_pr_info("Allocate ivshmem device\n");
        ivshmem_pdev_cxl = uk_malloc(a, sizeof(*ivshmem_pdev_cxl));
        if (!ivshmem_pdev_cxl) {
            uk_pr_err("Failed to allocate ivshmem pci device\n");
            return -ENOMEM;
        }
        /* Enable memory and IO */
        command_addr = get_config_addr(pci_dev->addr, PCI_COMMAND);
        pci_conf_write(command_addr, PCI_COMMAND_DECODE_ENABLE | PCI_COMMAND_MASTER);
        command_data = pci_conf_read(command_addr);
        uk_pr_info("command_data:0x%x\n", command_data);
        /* Probe revision id */
        __u32 class_addr = get_config_addr(pci_dev->addr, PCI_CONF_CLASS_ID);
        ivshmem_pdev_cxl->revision = pci_conf_read(class_addr) & PCI_CLASS_REVISION;
        /* Probe BAR data */
        ivshmem_pdev_cxl->bar_addr[0] = get_config_addr(pci_dev->addr, PCI_BASE_ADDRESS_0);
        devreg_addr = pci_conf_read(ivshmem_pdev_cxl->bar_addr[0]);
        uk_pr_info("bar0_data:0x%x\n", devreg_addr);

        /* Probe interrupt info */
        __u32 int_mask, int_stat, IVPosition;
        int_mask = readl((__u32 *)devreg_addr);
        uk_pr_info("interrupt mask:0x%x\n", int_mask);
        int_stat = readl((__u32 *)(devreg_addr | 0x4));
        uk_pr_info("interrupt status:0x%x\n", int_stat);
        IVPosition = readl((__u32 *)(devreg_addr | 0x8));
        uk_pr_info("IVPosition:0x%x\n", IVPosition);

        ivshmem_pdev_cxl->bar_addr[1] = get_config_addr(pci_dev->addr, PCI_BASE_ADDRESS_1);
        bar1_data = pci_conf_read(ivshmem_pdev_cxl->bar_addr[1]);
        uk_pr_info("bar1_data:0x%x\n", bar1_data);

        ivshmem_pdev_cxl->bar_addr[2] = get_config_addr(pci_dev->addr, PCI_BASE_ADDRESS_2);
	    bar2_data = pci_conf_read(ivshmem_pdev_cxl->bar_addr[2]);
        uk_pr_info("bar2_data:0x%x\n", bar2_data);

        ivshmem_pdev_cxl->bar_addr[3] = get_config_addr(pci_dev->addr, PCI_BASE_ADDRESS_3);
	    bar3_data = pci_conf_read(ivshmem_pdev_cxl->bar_addr[3]);
        uk_pr_info("bar3_data:0x%x\n", bar3_data);

        __u64 ivshmem_addr = (((__u64)bar3_data << 32) | (bar2_data & 0xfffffff0));
        __u64 ivshmem_size = 0x20000000;
        uk_pr_info("ivshmem_size:0x%x\n", ivshmem_size);
    // int rc = 0;
    // rc = uk_intctlr_irq_register(ivshmem_pdev_cxl->pdev->irq, ivshmem_irq_handle,
    //          ivshmem_pdev_cxl);
    // if (rc != 0) {
	// 	uk_pr_err("Failed to register the interrupt\n");
	// 	return rc;
	// }
        ivshmem_pdev_cxl->pdev = pci_dev;
        ivshmem_pdev_cxl->ivshmem_size = ivshmem_size;
        ivshmem_pdev_cxl->ivshmem_addr_start = ivshmem_addr;
        ivshmem_pdev_cxl->ivposition = IVPosition;
    } 
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

static int dev_ivshmem_open(struct device *dev, int flags __unused)
{
    UK_ASSERT(dev);
    uk_pr_info("open ivshmem dev\n");
    return 0;
}

static int dev_ivshmem_close(struct device *dev)
{
    UK_ASSERT(dev);
    uk_pr_info("close ivshmem dev\n");
    return 0;
}

static int dev_ivshmem_read(struct device *dev, struct uio *uio,
			   int flags __unused)
{
    UK_ASSERT(dev);
    UK_ASSERT(uio);
    UK_ASSERT(dev->private_data);
	size_t count;
	char *buf;
    struct ivshmem_pci_dev *ivshmem_dev = dev->private_data;
	buf = uio->uio_iov->iov_base;
	count = uio->uio_iov->iov_len;
	memcpy(buf, ivshmem_dev->ivshmem_addr_start, strlen(ivshmem_dev->ivshmem_addr_start));
    uk_pr_info("Reading msglen:%d, msg:%s\n", uio->uio_iov->iov_len, uio->uio_iov->iov_base);
	uio->uio_resid = 0;
	return 0;
}

static int dev_ivshmem_write(struct device *dev, struct uio *uio,
			    int flags __unused)
{
    UK_ASSERT(dev);
    UK_ASSERT(uio);
    UK_ASSERT(dev->private_data);
    uk_pr_info("Device name: %s\n", dev->name);
    uk_pr_info("Writing msglen:%d, msg:%s\n", uio->uio_iov->iov_len, uio->uio_iov->iov_base);
    struct ivshmem_pci_dev *ivshmem_dev = dev->private_data;
    uk_pr_info("dest addr:0x%lx\n", ivshmem_dev->ivshmem_addr_start);
    memcpy(ivshmem_dev->ivshmem_addr_start, uio->uio_iov->iov_base, uio->uio_iov->iov_len);
	return 0;
}

static dev_ivshmem_ioctl(struct device *dev, unsigned long cmd, void *args)
{
    UK_ASSERT(dev);
    struct ivshmem_pci_dev *pdev;

    pdev = (struct ivshmem_pci_dev *)dev->private_data;
    UK_ASSERT(pdev);
    (void)(cmd);
    return 0;
}

static struct devops ivshmem_devops = {
	.open = dev_ivshmem_open,
	.close = dev_ivshmem_close,
	.read = dev_ivshmem_read,
	.write = dev_ivshmem_write,
	.ioctl = dev_ivshmem_ioctl,
};

static struct driver drv_ivshmem = {
	.devops = &ivshmem_devops,
	.devsz = sizeof(struct ivshmem_pci_dev),
	.name = DEV_IVSHMEM_NAME
};

static int devfs_register(struct uk_init_ctx *ictx __unused)
{
    struct device *dev;
	uk_pr_info("Register '%s' to devfs\n", DEV_IVSHMEM_NAME);
    //struct device **ivshmem_devp;
	/* register /dev/ivshmem */
	int rc = device_create(&drv_ivshmem, DEV_IVSHMEM_NAME, D_CHR, &dev);
	if (unlikely(rc)) {
		uk_pr_err("Failed to register '%s' to devfs: %d\n",
			  DEV_IVSHMEM_NAME, rc);
		return -rc;
	}
    UK_ASSERT(dev);
    memcpy(dev->private_data, ivshmem_pdev, sizeof(struct ivshmem_pci_dev));
	return 0;
}

devfs_initcall(devfs_register);