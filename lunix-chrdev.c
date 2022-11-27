/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Alexandros Ionitsa < alexandros.ionitsa@gmail.com >
 * Panagiotis Giadikiaroglou < panosgiadi@gmail.com >
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
/*
 * Declare a prototype so we can define the "unused" attribute and keep
 * the compiler happy. This function is not yet used, because this helpcode
 * is a stub.
 */
// static int __attribute__((unused)) lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *);
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;

	WARN_ON(!(sensor = state->sensor));
	/* ? */
	/* We added sensor check 22/11
	 * needs checking!
	 */
	if (sensor->msr_data[state->type]->last_update == state->buf_timestamp)
	{
		debug("Refresh not required!\n");
		return 0;
	}
	/* The following return is bogus, just for the stub to compile */
	debug("Refresh required!\n");
	return 1; /* ? */
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	// struct lunix_sensor_struct __attribute__((unused)) * sensor;
	long int form_value;
	uint32_t raw_value, cur_timestamp;
	struct lunix_sensor_struct *sensor;

	WARN_ON(!(sensor = state->sensor));
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	/* ? */
	debug("Initiating state update\n");
	spin_lock(&sensor->lock);
	raw_value = sensor->msr_data[state->type]->values[0];
	cur_timestamp = sensor->msr_data[state->type]->last_update;
	spin_unlock(&sensor->lock);

	/* Why use spinlocks? See LDD3, p. 119 */

	/*
	 * Any new data available?
	 */
	/* ? */
	if (!lunix_chrdev_state_needs_refresh(state))
		return -EAGAIN;

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
	/* ? */

	if (state->type == BATT)
	{
		form_value = lookup_voltage[raw_value];
	}
	else if (state->type == TEMP)
	{
		form_value = lookup_temperature[raw_value];
	}
	else
	{
		form_value = lookup_light[raw_value];
	}

	state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%ld.%ld ", form_value / 1000, (form_value % 1000));

	debug("Formtatting buffer\n");
	debug("Updating state. Previous timestamp %d to current timestamp %d. Raw value is %d. Formatted value is %s.\n", state->buf_timestamp, cur_timestamp, raw_value, state->buf_data);

	state->buf_timestamp = cur_timestamp;
	debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	struct lunix_chrdev_state_struct *lunix_chrdev_state;
	/* ? */
	int ret;
	unsigned int minor, sensor_num;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	lunix_chrdev_state = kzalloc(sizeof(*lunix_chrdev_state), GFP_KERNEL);
	if (!lunix_chrdev_state)
	{
		ret = -ENOMEM;
		printk(KERN_ERR "Failed to allocate memory for Lunix sensors\n");
		goto out;
	}

	minor = iminor(inode);
	lunix_chrdev_state->type = minor % 8;
	sensor_num = minor / 8;
	lunix_chrdev_state->sensor = &(lunix_sensors[sensor_num]);
	lunix_chrdev_state->buf_lim = 0;
	lunix_chrdev_state->buf_timestamp = 0;
	sema_init(&(lunix_chrdev_state->lock), 1);

	filp->private_data = lunix_chrdev_state;

	/* Allocate a new Lunix character device private state structure */
	/* ? */
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	kfree(filp->private_data);
	printk(KERN_INFO "State data unloaded successfully\n");
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;
	debug("Read initiated.\n");

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* Lock? */

	if (down_interruptible(&state->lock))
	{
		debug("Process interrupted.\n");
		return -ERESTARTSYS;
	}
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0)
	{
		/* Check this implementation:
		 * if (filp->f_flags & O_NONBLOCK)
		 * 		return -EAGAIN;
		 */
		while (lunix_chrdev_state_update(state) == -EAGAIN)
		{
			/* ? */
			up(&state->lock);
			debug("Go to sleep bitch. Die motherfucker die.\n");
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
				return -ERESTARTSYS;
			if (down_interruptible(&state->lock))
				return -ERESTARTSYS;
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
		}
	}

	/* End of file */
	/* ? */

	/* Determine the number of cached bytes to copy to userspace */
	/* ? */
	cnt = min(cnt, (size_t)(state->buf_lim - *f_pos));

	if (copy_to_user(usrbuf, state->buf_data + *f_pos, cnt))
	{
		debug("Copy to user failed.");
		up(&state->lock);
		return -EFAULT;
	}

	/* Auto-rewind on EOF mode? */
	/* ? */
	*f_pos += cnt;
	if (*f_pos >= state->buf_lim)
		*f_pos = 0;

	/*
	 * The next two lines  are just meant to suppress a compiler warning
	 * for the "unused" out: label, and for the uninitialized "ret" value.
	 * It's true, this helpcode is a stub, and doesn't use them properly.
	 * Remove them when you've started working on this code.
	 */
	// ret = -ENODEV;
	ret = cnt;
	up(&state->lock);
	return cnt;
	// goto out;
	// out:
	/* Unlock? */
	// return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops =
	{
		.owner = THIS_MODULE,
		.open = lunix_chrdev_open,
		.release = lunix_chrdev_release,
		.read = lunix_chrdev_read,
		.unlocked_ioctl = lunix_chrdev_ioctl,
		.mmap = lunix_chrdev_mmap};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */

	/*
	 * register_chrdev_region(_ , _ , char *name);
	 * possible wrong name, check later
	 */

	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix");
	/* register_chrdev_region? */

	/* Since this code is a stub, exit early */
	// return 0;

	if (ret < 0)
	{
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	/* ? */
	/* cdev_add? */
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);

	if (ret < 0)
	{
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
