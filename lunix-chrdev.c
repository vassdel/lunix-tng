/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Doufexi Maria
 * Delis Vassilis
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
//static int __attribute__((unused)) lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *);
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));
	/* ? */

	/* The following return is bogus, just for the stub to compile */
	return (state->buf_timestamp != sensor->msr_data[0]->last_update); // alternative with <= ?
	/* ? 
		* if these too timestamps are not the same we need update!
		* msr_data[{0,1,2}] are the same from get_seconds()
	*/
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint32_t sensor_timestamp, sensor_value;
	long value;
	int ret=0;

	debug("beggining update method\n");
	
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	WARN_ON( !(sensor = state->sensor));


	/* ? */
	/* Why use spinlocks? See LDD3, p. 119 */

	/*
	 * Any new data available?
	 */

	/* spinlocks vs mutex(or semaphores)*/
	spin_lock_irq(&sensor->lock); 		
	sensor_timestamp = sensor->msr_data[BATT]->last_update;
	sensor_value = sensor->msr_data[state->type]->values[0];
	spin_unlock_irq(&sensor->lock); 

	/* ? */

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */


	/* ? */
	if (lunix_chrdev_state_needs_refresh(state)) {
		state->buf_timestamp = sensor_timestamp;
		switch (state->type) {
			case BATT:
				value = lookup_voltage[sensor_value];
				break;
			case TEMP:
				value = lookup_temperature[sensor_value];
				break;
			case LIGHT:
				value = lookup_light[sensor_value];
				break;
			default:
				debug("Not a a valid type! (eg temperature, battery, light)!\n");
				ret = -EAGAIN;
				goto out;
			}
		/* I have got the right data and I am going to convert them*/
		debug("Refreshed Data(Formatted): %ld\n", value);
		state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%ld.%ld\n", value/1000, abs(value%1000)); // Output Formatted
		debug("Refreshed Data received -> %d Bytes\n", state->buf_lim);
	}
	else { // we don't need refresh
		debug("No new data. Already upt to date!\n");
		ret = -EAGAIN;
		goto out;
	}

out:
	debug("Leaving update method\n");
	return ret;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/


static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
	struct lunix_chrdev_state_struct *state;

	int ret;
	int minor, sensor_num;

	ret = -ENODEV; // default return value: Error NO DEVice

	// character devices should be non-seekable. If not-> error
	if ((ret = nonseekable_open(inode, filp)) < 0) { 
		goto out;
	}

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */


	/* Allocate a new Lunix character device private state structure */

	state = kmalloc(sizeof(*state), GFP_KERNEL);
	// unsuccesful memory allocation case
	if (!state) { 
		ret = -ENOMEM;
		debug("Failed to allocate memory for Lunix driver state");
		goto out;
	}
	minor = iminor(inode);

	sensor_num = minor / 8; 						// minor = sensor * 8 + {0,1,2}
	state->sensor = &(lunix_sensors[sensor_num]); 	// array of sensors (pointer)
  	sema_init(&state->lock, 1); 					// init the semaphore to unlocked
	state->type = minor % 8;						// BATT, TEMP, LIGHT
	state->buf_lim = 0; 							// initially zero-> gets bigger when we write to buffer
	state->buf_timestamp = 0;						// random timestamp
	filp->private_data = state;						
	debug("successfully allocated state\n");
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	struct lunix_chrdev_state_struct *state;
	state = filp->private_data;
	WARN_ON(!state); /*according to read method*/
	kfree(state);
	debug("Released memory!\n");
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

	state = filp->private_data; // struct to hold private data between processes
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* Lock? */
	if (down_interruptible(&state->lock)){ // down->lock, interruptible->all interrupts allowed
		return -ERESTARTSYS;
	}
	/* --------------------------------------------------------> Update Hint! 
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	debug("Just Before Update");
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) { /* doesn't have something to read*/
			/* ? */
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
			
			// release the lock. if not, no "writer" would wake it up->keep sleeping indefinately
			up(&state->lock); 
			// quick check to see if user wants non blocking I/O (sensor interrupts)
			if (filp->f_flags & O_NONBLOCK){ 
				return -EAGAIN;
			}
			// checks if we got woken up by signal from the waiting queue 
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state))){
				return -ERESTARTSYS; // either restarts the system call or returns -EINTR to user space.
			}
			// acquire the lock again to avoid race conditions
			if (down_interruptible(&state->lock)){
				return -ERESTARTSYS; // either restarts the system call or returns -EINTR to user space.
			}
		}
	}	

	/* End of file */
	/* ? */
	if (*f_pos >= state->buf_lim){ // in case of bad pointer (overflow?)
		goto out;
	}
	/*if read method wants to copy more than available, cnt gets smaller 
	(eg reads 4 every time and in the last "loop" it only has 2 available)*/
	if (cnt > state->buf_lim - *f_pos){  
		cnt = state->buf_lim - *f_pos;
	}
	/* Determine the number of cached bytes to copy to userspace */
	/* ? */
	if (copy_to_user(usrbuf, state->buf_data + *f_pos, cnt)) {
		ret = -EFAULT;
		goto out;
	}
	debug("Bytes read: %d", (int)cnt);
	*f_pos += cnt;
	ret = cnt;

	/* Auto-rewind on EOF mode? */
	/* ? */

	/* When you reach the end of the file, reset the pointer to the beggining */
	if (*f_pos == state->buf_lim) {
		*f_pos = 0;
	}

	/*
	 * The next two lines  are just meant to suppress a compiler warning
	 * for the "unused" out: label, and for the uninitialized "ret" value.
	 * It's true, this helpcode is a stub, and doesn't use them properly.
	 * Remove them when you've started working on this code.
	 */
	//ret = -ENODEV;
	//goto out;
	
out:
	/* Unlock? */
	up(&state->lock); // ensure accessibility by others when you finish you job
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
    .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
    /*
     
	Register the character device with the kernel, asking for
	a range of minor numbers (number of sensors * 8 measurements / sensor)
 	beginning with LINUX_CHRDEV_MAJOR:0*/
	
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt * 8;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0); //  Constructs a device number from the major number and a starting minor number (0).
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix"); 	// major/minor, number of devs, name
    if (ret < 0) {
        debug("failed to register region, ret = %d\n", ret);
        goto out;
    }
    /* ? /
    / cdev_add? */
    ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt); 		// pointer to structure, first device, number of devs
    if (ret < 0) {
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