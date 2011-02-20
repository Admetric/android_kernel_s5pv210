/**
 *
 *  \file drmc110.c
 *
 *  Supplies a minimal ioctl interface for DRM, pretending to be the
 *  real kernel DRM by exporting standard kernel DRM functions. This
 *  is simply a temporary workaround for our target system as it does
 *  not have a PCI bus, but PCI support is required to use the full
 *  Linux DRM implementation.
 *
 *  We assume that the ioctl functions required by libdrm (listed in
 *  drm_ioctls[] below) do not use any significant functionality from
 *  the kernel DRM code, nor do the ioctls implemented in the PVR SGX
 *  driver (pvrsrvkm.ko) module. Therefore we are able to use a set of
 *  stub functions to allow the PVR driver (but not any other DRM
 *  clients) to operate. We are simply forwarding the driver-specific
 *  ioctls to the PVR code, rather than actually implementing on any
 *  DRM functionality. 
 *
 *  Note that all the stub functions are adapted from the
 *  corresponding linux kernel DRM code in drivers/gpu/drm.
 */

/*
 * Copyright 1999, 2000 Precision Insight, Inc., Cedar Park, Texas.
 * Copyright 2000 VA Linux Systems, Inc., Sunnyvale, California.
 *
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including
 * the next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT.  IN NO EVENT SHALL VA LINUX SYSTEMS AND/OR ITS
 * SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/*
 * Copyright 2009, 2010, Samsung Electronics.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/platform_device.h>

#include <drm/drmP.h>
#include <drm/drm.h>

#undef PCI_SETUP
#undef IOCTL_DEBUG

//#define EXTRA_DEBUG
//#define MINIMUM_DEBUG

MODULE_LICENSE("Dual BSD/GPL");

/* ---------------------------------------------------------------------- */

struct drm_device *c110_dev = 0;
struct drm_minor *c110_minor = 0;

/*
 *  We just have one drm_device structure, and one drm_minor:
 *  
 *   drm_device
 *   +--------------+ 
 *   |              |     
 *   |   c110_dev   | .driver => PVR driver (pvrsrvkm.ko)
 *   |              |     
 *   +--------------+     
 * 
 *   drm_minor
 *   +--------------+			 
 *   |              |			  
 *   |  c110_minor  | .dev => c110_dev
 *   |              |			 
 *   +--------------+                  
 * 
 * Each open device file (libdrm uses /dev/dri/card0) *filp, has
 * a drm_file private data area, *file_priv = filp->private_data,
 * all of which point to our single drm_minor and drm_device.
 * 
 *   drm_file
 *   +--------------+
 *   |      0       | .minor => c110_minor
 *   +--------------+
 * 
 *   drm_file
 *   +--------------+
 *   |      1       | .minor => c110_minor
 *   +--------------+
 *         ...
 *
 *   drm_file
 *   +--------------+
 *   |      n       | .minor => c110_minor
 *   +--------------+
 * 
 */

/* ---------------------------------------------------------------------- */
/* tmp, just for debug... */

typedef struct PVRSRV_BRIDGE_PACKAGE_TAG
{
	unsigned int ui32BridgeID;			/*!< ioctl/drvesc index */
	unsigned int ui32Size;				/*!< size of structure */
	void *pvParamIn;					/*!< input data buffer */
	unsigned int ui32InBufferSize;		/*!< size of input data buffer */
	void *pvParamOut;					/*!< output data buffer */
	unsigned int ui32OutBufferSize;		/*!< size of output data buffer */
	unsigned int hKernelServices;		/*!< kernel servcies handle */
} PVRSRV_BRIDGE_PACKAGE;

/* ---------------------------------------------------------------------- */

/* Platform ------------------------------------------------------------- */
static int drmc110_platform_suspend(struct platform_device *pdev, pm_message_t state);
static int drmc110_platform_resume(struct platform_device *pdev);
static void drmc110_platform_dev_release(struct device *dev);

static int drmc110_platform_register(void);
static void drmc110_platform_unregister(void);

static struct platform_device drmc110_platform_device = {
	.name = "drmc110plat",
	.id = -1,
	.num_resources = 0,
	.resource = NULL,
	.id_entry = NULL,
	.dev = {
		/* required or kernel complains */
		.release = drmc110_platform_dev_release,
	},
};

static struct platform_driver drmc110_platform_driver = {
	.suspend = drmc110_platform_suspend,
	.resume = drmc110_platform_resume,
	.driver = {
		.name = "drmc110plat",
		.owner = THIS_MODULE,
	},
};

static int drmc110_platform_suspend(struct platform_device *pdev, pm_message_t state)
{
	if(!c110_dev)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "C110 DRM not initialised\n");
		#endif
	}
	else if(c110_dev->driver->suspend)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "Calling DRM suspend function\n");
		#endif
		c110_dev->driver->suspend(c110_dev, state);
	}
	else
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "No DRM suspend function supplied\n");
		#endif
	}

	return 0;
}

static int drmc110_platform_resume(struct platform_device *pdev)
{
	if(!c110_dev)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "C110 DRM not initialised\n");
		#endif
	}
	else if(c110_dev->driver->resume)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "Calling DRM resume function\n");
		#endif
		c110_dev->driver->resume(c110_dev);
	}
	else
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "No DRM resume function supplied\n");
		#endif
	}

	return 0;
}

static void drmc110_platform_dev_release(struct device *dev)
{

}

static int drmc110_platform_register(void)
{
	int ret;

	ret = platform_driver_register(&drmc110_platform_driver);

	if(ret == 0) {
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "platform_driver_register ok\n");
		#endif
	} else {
		printk(KERN_ERR "platform_driver_register failed\n");
		return ret;
	}
	
	ret = platform_device_register(&drmc110_platform_device);
	
	if(ret == 0) {
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "platform_device_register ok\n");
		#endif
	} else {
		printk(KERN_ERR "platform_device_register failed\n");
		platform_driver_unregister(&drmc110_platform_driver);
		return ret;
	}

	return ret;
}

static void drmc110_platform_unregister(void)
{
	platform_device_unregister(&drmc110_platform_device);
	platform_driver_unregister(&drmc110_platform_driver);
}

/* ---------------------------------------------------------------------- */

#if (((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32))&&(defined(CONFIG_MACH_VOGUEV210)))\
	||(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33)))
static int drm_copy_field(char *buf, size_t *buf_len, const char *value)
{
	int len;

	/* don't overflow userbuf */
	len = strlen(value);
	if (len > *buf_len)
		len = *buf_len;

	/* let userspace know exact length of driver value (which could be
	* larger than the userspace-supplied buffer) */
	*buf_len = strlen(value);

	/* finally, try filling in the userbuf */
	if (len && buf)
		if (copy_to_user(buf, value, len))
			return -EFAULT;
	return 0;
}

static int drm_version(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	struct drm_version *version = data;
	int err;

	version->version_major = dev->driver->major;
	version->version_minor = dev->driver->minor;
	version->version_patchlevel = dev->driver->patchlevel;

	err = drm_copy_field(version->name, &version->name_len, dev->driver->name);
	if (!err)
		err = drm_copy_field(version->date, &version->date_len, dev->driver->date);

        if (!err)
	        err = drm_copy_field(version->desc, &version->desc_len, dev->driver->desc);

	return err;
}
#else
static int drm_version(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	struct drm_version *version = data;
	int len;

	version->version_major = dev->driver->major;
	version->version_minor = dev->driver->minor;
	version->version_patchlevel = dev->driver->patchlevel;
	DRM_COPY(version->name, dev->driver->name);
	DRM_COPY(version->date, dev->driver->date);
	DRM_COPY(version->desc, dev->driver->desc);

	return 0;
}
#endif

/*
 *  Libdrm checks for "the first minor number that matches the device
 *  name and isn't already in use" and, "if it's in use it will have a
 *  busid assigned already". It'll call drm_getunique() twice, first
 *  to get the size, second to get the string.
 */

int drm_getunique(struct drm_device *dev, void *data,
		  struct drm_file *file_priv)
{
	struct drm_unique *u = data;

	const char* unique = "";
	int unique_len = 0; 

	if (u->unique_len >= unique_len)
	{
		if (copy_to_user(u->unique, unique, unique_len))
			return -EFAULT;
	}
	u->unique_len = unique_len;

	return 0;
}

int drm_getmagic(struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	static drm_magic_t sequence = 0;
	static DEFINE_SPINLOCK(lock);
	struct drm_auth *auth = data;

	/* Find unique magic */
	if (file_priv->magic) 
	{
		auth->magic = file_priv->magic;
	} 
	else 
	{
		spin_lock(&lock);
		if (!sequence)
			++sequence;	/* reserve 0 */
		auth->magic = sequence++;
		spin_unlock(&lock);
		file_priv->magic = auth->magic;

		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "drmc110: assigned magic %d\n", auth->magic);
		#endif
	}

	return 0;
}

int drm_authmagic(struct drm_device *dev, void *data,
		  struct drm_file *file_priv)
{
	struct drm_auth *auth = data;
	return (auth->magic != 0) ? 0 : -EINVAL;
}

int drm_dropmaster_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file_priv)
{
	return 0;
}

int drm_setversion(struct drm_device *dev, void *data, 
				   struct drm_file *file_priv)
{
	if (dev->driver->set_version)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "drmc110: warning, driver implements set_version()");
		/* dev->driver->set_version(dev, sv); */
		#endif
	}
	return 0;
}

/* ---------------------------------------------------------------------- */

static struct drm_ioctl_desc drm_ioctls[] = 
{
	DRM_IOCTL_DEF(DRM_IOCTL_VERSION, drm_version, 0),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_UNIQUE, drm_getunique, 0),
	DRM_IOCTL_DEF(DRM_IOCTL_GET_MAGIC, drm_getmagic, 0),
	DRM_IOCTL_DEF(DRM_IOCTL_AUTH_MAGIC, drm_authmagic, 0),
	DRM_IOCTL_DEF(DRM_IOCTL_SET_VERSION, drm_setversion, 0),
	DRM_IOCTL_DEF(DRM_IOCTL_DROP_MASTER, drm_dropmaster_ioctl, 0)
};

#define C110DRM_IOCTL_COUNT	ARRAY_SIZE( drm_ioctls )

/* ---------------------------------------------------------------------- */

int drmc110_setup(struct drm_device * dev)
{
	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110_setup (not used at present)\n");
	#endif
	return 0;
}

static int drmc110_open_helper(struct inode *inode, struct file *filp,
			   struct drm_device * dev)
{
	struct drm_file *priv = 0;
	int ret;

	if (filp->f_flags & O_EXCL)
		return -EBUSY;

	priv = kmalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	memset(priv, 0, sizeof(*priv));
	filp->private_data = priv;
	priv->filp = filp;
	priv->uid = current_euid();
	priv->pid = task_pid_nr(current);
	priv->minor = c110_minor;
	priv->ioctl_count = 0;
	priv->authenticated = capable(CAP_SYS_ADMIN);
	priv->lock_count = 0;

	if (c110_dev->driver->open)
	{
		ret = c110_dev->driver->open(c110_dev, priv);
		if (ret < 0)
		{
			#if defined(MINIMUM_DEBUG)
			printk(KERN_ALERT "drmc110: warning: driver->open() returned %d\n", ret);
			#endif
			goto out_free;
		}
	}

	priv->is_master = 0;
	priv->authenticated = 1;

	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110: open file=0x%08x, priv=0x%08x\n", 
		   (unsigned int) filp, (unsigned int) priv);
	#endif

	return 0;

out_free:
	kfree(priv);
	filp->private_data = NULL;
	return ret;
}

int drmc110_lastclose(struct drm_device * dev)
{
	if (dev->driver->lastclose)
		dev->driver->lastclose(dev);

	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "driver lastclose completed\n");
	#endif

	return 0;
}

/* ---------------------------------------------------------------------- */
/* 
 *  Export a subset of the standard DRM API, as used by pvrsrvkm.ko code:
 */

int drm_init(struct drm_driver *driver)
{
	int minor_id = 0;
	int ret = 0;

#ifdef PCI_SETUP
	struct pci_dev *pdev = NULL;
	struct pci_dev *idev = NULL;
	const struct pci_device_id *pid;
	int i;

	printk(KERN_ALERT "drmc110: drm_init() PCI\n");

	if (driver->driver_features & DRIVER_MODESET)
		printk(KERN_ALERT "drmc110: ERROR, DRIVER_MODESET\n"); /* to do: assert */

	pid = &driver->pci_driver.id_table[0];

	if (!pid)
	{
		printk(KERN_ALERT "drmc110: null PCI device ID\n");
		return -EINVAL;
	}
		
	i = 0;
	while ((idev = pci_get_subsys(pid->vendor, pid->device, 
         pid->subvendor, pid->subdevice, idev)) != NULL) 
	{
		if ((idev->class & pid->class_mask) != pid->class)
			continue;

		printk(KERN_ALERT "drmc110: found pci device %d, 0x%08x\n", 
			   i, (unsigned int) idev);
		
		if (i==0)
			pdev = idev;
		++i;
	}
	
	if (!pdev)
	{
		printk(KERN_ALERT "drmc110: null PCI device\n");
		return -EINVAL;
	}
#endif

	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110: drm_init()\n");
	#endif

	/*  The real code creates a list of drm_device structs,
	 *  in "driver->device_list", and populates the list "using
	 *  consecutive minors". We're only going to have the one
	 *  DRM device.
	 */ 

	c110_dev = kzalloc(sizeof(*c110_dev), GFP_KERNEL);
	if (!c110_dev)
		return -ENOMEM;

#ifdef PCI_SETUP
	pci_dev_get(pdev);
	ret = pci_enable_device(pdev);
	if (ret) /* todo: error handling is broken (cleanup) */
		return ret;
	pci_set_master(pdev);
	c110_dev->pdev = pdev;
	c110_dev->pci_device = pdev->device;
	c110_dev->pci_vendor = pdev->vendor;
#else
	c110_dev->pdev = 0;
	c110_dev->pci_device = 0;
	c110_dev->pci_vendor = 0;
#endif

	c110_dev->driver = driver;

	if (c110_dev->driver->firstopen) /* not needed? */
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "drmc110: calling firstopen()\n");
		#endif

		ret = c110_dev->driver->firstopen(c110_dev);
		if (ret != 0)
			return ret;
	}

	spin_lock_init(&c110_dev->count_lock);
	
	atomic_set(&c110_dev->ioctl_count, 0);

	/*  Real code calls drm_get_minor() here, which would fill in a
	 *  struct with the device number and add debugfs, sysfs, /proc,
	 *  etc.  We're just assuming one device at minor 0.
	 */

	/* to do: mem leak if previous ok and this fails */
	c110_minor = kzalloc(sizeof(*c110_minor), GFP_KERNEL);
	if (!c110_minor)
		return -ENOMEM;

	c110_minor->type = DRM_MINOR_UNASSIGNED;
	c110_minor->device = MKDEV(DRM_MAJOR, minor_id);
	c110_minor->dev = c110_dev;
	c110_minor->index = minor_id;

	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110: new minor assigned %d\n", minor_id);
	#endif

	/* Run driver load hook */
	if (driver->load)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "drmc110: call driver load hook\n");
		#endif

		ret = driver->load(c110_dev, 0);

		#if defined(MINIMUM_DEBUG)
		if (ret)
		{
			printk(KERN_ALERT "drmc110 driver load failed %d\n", ret);
		}
		else
		{
			printk(KERN_ALERT "drmc110: initialized %s %d.%d.%d %s\n", 
				   driver->name, driver->major, 
				   driver->minor, driver->patchlevel, driver->date);
		}
		#endif
	}

	return ret;
}

EXPORT_SYMBOL(drm_init);

void drm_exit(struct drm_driver *driver)
{
	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110: drm_exit()\n");
	#endif

	if (!driver)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "drmc110: drm_exit() driver=0\n");
		#endif

		return;
	}
	
	if (!c110_dev)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "drmc110: drm_exit() dev=0\n");
		#endif

		return;
	}

	if (c110_dev->driver != driver)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "drmc110: drm_exit() driver != dev->driver\n");
		#endif

		return;
	}
	
	/*  Real drm_exit will trigger driver->lastclose. Don't think we
	 *  define one in the PVR code, but just in case: 
	 */
	if (driver->lastclose)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "drmc110: call driver lastclose hook\n");			
		#endif

		driver->lastclose(c110_dev);
	}

	if (driver->unload)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "drmc110: call driver unload hook\n");
		#endif

		driver->unload(c110_dev);
	}

	if (c110_dev->devname) 
	{
		kfree(c110_dev->devname);
		c110_dev->devname = NULL;
	}

	kfree(c110_dev);
	kfree(c110_minor);
}

EXPORT_SYMBOL(drm_exit);

/* ---------------------------------------------------------------------- */

int drm_open(struct inode *inode, struct file *filp)
{
	int minor_id = iminor(inode);
	int retcode;

#ifdef EXTRA_DEBUG
	printk(KERN_ALERT "drmc110: open inode=0x%x(%d), minor=%d, filp=0x%x, pid=%d\n", 
		   (unsigned int) inode, 
		   (int) inode->i_ino,
		   (unsigned int) minor_id, 
		   (unsigned int) filp,
		   task_pid_nr(current));
#endif

	/* Refuse all but minor 0 */
	if (minor_id != 0)
		return -ENODEV;

	retcode = drmc110_open_helper(inode, filp, c110_dev);

	if (!retcode)
	{
		spin_lock(&c110_dev->count_lock);
		if (!c110_dev->open_count++) 
		{
			spin_unlock(&c110_dev->count_lock);
			retcode = drmc110_setup(c110_dev);
			goto out;
		}
		spin_unlock(&c110_dev->count_lock);
	}

out:
	return retcode;
}

EXPORT_SYMBOL(drm_open);

int drm_release(struct inode *inode, struct file *filp)
{
	struct drm_file *file_priv = filp->private_data;
	struct drm_device *dev = file_priv->minor->dev;
	int retcode = 0;

	lock_kernel();

#ifdef EXTRA_DEBUG
	printk(KERN_ALERT "drmc110_release inode=0x%x(%d), filp=0x%x, pid=%d, dev=0x%lx, open=%d\n", 
		   (unsigned int) inode, 
		   (int) inode->i_ino,
		   (unsigned int) filp,
		   task_pid_nr(current),
		   (long)old_encode_dev(file_priv->minor->device),
		   dev->open_count);
#endif

	if (dev->driver->preclose)
		dev->driver->preclose(dev, file_priv);

	if (dev->driver->postclose)
		dev->driver->postclose(dev, file_priv);

#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110: close file=0x%08x, priv=0x%08x\n", 
		   (unsigned int) filp, (unsigned int) file_priv);
#endif

	kfree(file_priv);

	spin_lock(&dev->count_lock);
	if (!--dev->open_count) 
	{
		if (atomic_read(&dev->ioctl_count)) 
		{
			#if defined(MINIMUM_DEBUG)
			printk("Device busy: %d\n", atomic_read(&dev->ioctl_count));
			#endif

			spin_unlock(&dev->count_lock);
			unlock_kernel();
			return -EBUSY;
		}
		spin_unlock(&dev->count_lock);
		unlock_kernel();
		return drmc110_lastclose(dev);
	}
	spin_unlock(&dev->count_lock);

	unlock_kernel();

	return retcode;
}

EXPORT_SYMBOL(drm_release);

/* ---------------------------------------------------------------------- */
#if (((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32))&&(defined(CONFIG_MACH_VOGUEV210)))\
	||(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33)))
long drm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
int drm_ioctl(struct inode *inode, struct file *filp, 
				  unsigned int cmd, unsigned long arg)
#endif
{
	struct drm_file *file_priv = 0;
	struct drm_device *dev = 0;
	struct drm_ioctl_desc *ioctl = 0;
	drm_ioctl_t *func = 0;
	int retcode = -EINVAL;
	unsigned int nr = DRM_IOCTL_NR(cmd);
	char *kdata = NULL;

	int iocnt = 0;

	PVRSRV_BRIDGE_PACKAGE* psBridgePackageKM; /* tmp debug only */

#if 0
//#ifdef IOCTL_DEBUG
	printk(KERN_ALERT "drmc110: ioctl nr=%d arg=0x%lx, pid=%d, inode=0x%x(%d)\n", 
		   nr, arg, task_pid_nr(current), inode, inode->i_ino);
#endif

	if (!filp)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "stopping as null filp parameter\n");
		#endif

		goto err_i1;
	}

	file_priv = filp->private_data;

	if (!file_priv)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "stopping as null file_priv parameter\n");
		#endif

		goto err_i1;
	}

	if (!file_priv->minor)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "stopping as null file_priv->minor parameter\n");
		#endif

		goto err_i1;
	}

	dev = file_priv->minor->dev;

	#if defined(MINIMUM_DEBUG)
	if (dev != c110_dev)
		printk(KERN_ALERT " dev=0x%08x vs c110_dev=0x%08x\n",
			   (unsigned int) dev, (unsigned int) c110_dev);
	#endif

	if (dev != c110_dev)
	{
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "stopping as dev parameter doesn't match\n");
		#endif

		goto err_i1;
	}

	atomic_inc(&dev->ioctl_count);
	iocnt = atomic_read(&dev->ioctl_count);
	#if defined(MINIMUM_DEBUG)
	if (iocnt > 1)
		printk(KERN_ALERT "ioctl_count %d\n", iocnt);
	#endif
	++file_priv->ioctl_count;

	/* Driver-defined ioctls: */
	if ((nr >= DRM_COMMAND_BASE) && (nr < DRM_COMMAND_END) &&
	    (nr < DRM_COMMAND_BASE + dev->driver->num_ioctls))
	{
		ioctl = &dev->driver->ioctls[nr - DRM_COMMAND_BASE];
#ifdef IOCTL_DEBUG
		printk(KERN_ALERT "drmc110: device-specific DRM ioctl %d\n", 
			   nr - DRM_COMMAND_BASE);
#endif

		if ((nr - DRM_COMMAND_BASE) == 0)
		{
#ifdef IOCTL_DEBUG
			printk(KERN_ALERT " BridgeDispatch ioctl\n");
#endif
			psBridgePackageKM = (PVRSRV_BRIDGE_PACKAGE *)arg;

			if (!psBridgePackageKM)
			{
				printk(KERN_ALERT " Null psBridgePackageKM\n"); 
			}
			else
			{
#ifdef EXTRA_DEBUG
				printk(KERN_ALERT "df:io=%d,hl=0x%x,ic=%d,oc=%d\n",
					   DRM_IOCTL_NR(psBridgePackageKM->ui32BridgeID), 
					   psBridgePackageKM->hKernelServices,
					   iocnt, dev->open_count);
#endif

#ifdef IOCTL_DEBUG
				printk(KERN_ALERT " id = 0x%x, %d, size = %d, insize = %d, outsize = %d, handle = 0x%x\n",
					   psBridgePackageKM->ui32BridgeID,
					   DRM_IOCTL_NR(psBridgePackageKM->ui32BridgeID), 
					   psBridgePackageKM->ui32Size,
					   psBridgePackageKM->ui32InBufferSize,
					   psBridgePackageKM->ui32OutBufferSize,
					   psBridgePackageKM->hKernelServices);
#endif
			}
		}
	}
	else
	{

		if (((nr >= DRM_COMMAND_END) || (nr < DRM_COMMAND_BASE)) && 
			(nr < C110DRM_IOCTL_COUNT))
		{
			ioctl = &drm_ioctls[nr];
			cmd = ioctl->cmd;
		}
		else
		{
			#if defined(MINIMUM_DEBUG)
			printk(KERN_ALERT "drmc110: warning: unhandled ioctl 0x%x (%d)\n", nr, nr);
			#endif

			retcode = -EINVAL;
			goto err_i1;
		}
	}

	func = ioctl->func;

	if (!func) 
	{
		retcode = -EINVAL;
		#if defined(MINIMUM_DEBUG)
		printk(KERN_ALERT "drmc110: warning: Unhandled ioctl 0x%x (%d)\n", nr, nr);
		#endif
		goto err_i1;
	} 

	if (cmd & (IOC_IN | IOC_OUT)) 
	{
		{
			kdata = kmalloc(_IOC_SIZE(cmd), GFP_KERNEL);
			if (!kdata) 
			{
				retcode = -ENOMEM;
				goto err_i1;
			}
		}
	}
	
	if (cmd & IOC_IN)
	{
		if (copy_from_user(kdata, (void __user *)arg, _IOC_SIZE(cmd)) != 0) 
		{
			retcode = -EFAULT;
			goto err_i1;
		}
	}
	retcode = func(dev, kdata, file_priv);
	
	if ((retcode == 0) && (cmd & IOC_OUT))
	{
		if (copy_to_user((void __user *)arg, kdata, _IOC_SIZE(cmd)) != 0)
			retcode = -EFAULT;
	}
	
err_i1:
	if (kdata)
		kfree(kdata);
	atomic_dec(&dev->ioctl_count);

	#if defined(MINIMUM_DEBUG)
	if (retcode)
		printk("drmc110 ioctl ret=%d\n", retcode);
	#endif

	return retcode;
}

EXPORT_SYMBOL(drm_ioctl);

/* ---------------------------------------------------------------------- */
/* 
 *  These are referred to during DRM setup, but aren't ever called:
 */

int drm_mmap(struct file *filp, struct vm_area_struct *vma)
{
	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110: warning: drm_mmap() called\n");
	#endif

	return -EINVAL;
}

EXPORT_SYMBOL(drm_mmap);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
resource_size_t drm_core_get_map_ofs(struct drm_local_map *map)
#else
unsigned long drm_core_get_map_ofs(struct drm_map *map)
#endif
{
	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110: warning: drm_core_get_map_ofs() called\n");
	#endif

	return 0;
}

EXPORT_SYMBOL(drm_core_get_map_ofs);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
resource_size_t drm_core_get_reg_ofs(struct drm_device *dev)
#else
unsigned long drm_core_get_reg_ofs(struct drm_device *dev)
#endif
{
	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110: warning: drm_core_get_reg_ofs() called\n");
	#endif

	return 0;
}

EXPORT_SYMBOL(drm_core_get_reg_ofs);

unsigned int drm_poll(struct file *filp, struct poll_table_struct *wait)
{
	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110: warning: drm_poll() called\n");
	#endif

	return 0;
}

EXPORT_SYMBOL(drm_poll);

int drm_fasync(int fd, struct file *filp, int on)
{
	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110: warning: drm_fasync() called\n");
	#endif

	return 0;
}

EXPORT_SYMBOL(drm_fasync);

/* ---------------------------------------------------------------------- */
/*
 *  Module initialisation 
 */

int drm_stub_open(struct inode *inode, struct file *filp)
{
	struct drm_device *dev = NULL;
	int minor_id = iminor(inode);
	int err = -ENODEV;
	const struct file_operations *old_fops;

#ifdef EXTRA_DEBUG_XXX
	printk(KERN_ALERT "drm_stub_open inode=0x%x(%d), minor=%d, filp=0x%x, pid=%d\n", 
		   (unsigned int) inode, 
		   inode->i_ino,
		   (unsigned int) minor_id, 
		   (unsigned int) filp,
		   task_pid_nr(current));
#endif

	/* Refuse all but minor 0 */
	if (minor_id != 0)
		return -ENODEV;

	lock_kernel();

	dev = c110_dev;

	old_fops = filp->f_op;
	filp->f_op = fops_get(&dev->driver->fops);
	if (filp->f_op == NULL) 
	{
		filp->f_op = old_fops;
		goto out;
	}
	if (filp->f_op->open && (err = filp->f_op->open(inode, filp)))
	{
		fops_put(filp->f_op);
		filp->f_op = fops_get(old_fops);
	}
	fops_put(old_fops);

out:
	unlock_kernel();
	return err;
}

static const struct file_operations drmc110_fops = 
{
	.owner = THIS_MODULE,
	.open = drm_stub_open
};

static int drmc110_module_init(void)
{
	int err = 0;

	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110_module_init()\n");
	#endif

	err = register_chrdev(DRM_MAJOR, DRM_NAME, &drmc110_fops);

	#if defined(MINIMUM_DEBUG)
	if (err < 0)
		printk(KERN_ALERT "register_chrdev failed\n");
	else
		printk(KERN_ALERT "register_chrdev ok\n");
	#endif

	if(!err)
	{
		if((err = drmc110_platform_register()))
		{
			printk(KERN_ERR "drmc110_platform_register failed\n");	
		}
	}

	return err;
}

static void drmc110_module_exit(void)
{
	drmc110_platform_unregister();
	
	unregister_chrdev(DRM_MAJOR, DRM_NAME);

	#if defined(MINIMUM_DEBUG)
	printk(KERN_ALERT "drmc110_module_exit()\n");
	#endif
}

module_init(drmc110_module_init);
module_exit(drmc110_module_exit);

/* ---------------------------------------------------------------------- */
