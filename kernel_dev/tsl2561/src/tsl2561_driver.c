/*
*	TSL2561 Raspberry Pi Kernel Module Driver
*
*	github.com/thaletterb
*/

#include <linux/slab.h>			// kzalloc
#include <linux/module.h>		// needed by all modules
#include <linux/kernel.h>		// kern info
#include <linux/timer.h>		// timer list
#include <linux/workqueue.h>	// schedule work

#include <linux/input.h>		
#include <linux/i2c.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brian Vuong");
MODULE_DESCRIPTION("Controls a TSL2561 Lux Sensor");
MODULE_VERSION("0.1");

/*
*	I2C Setup
*/

/*
*	Device Setup
*/

/*	
*	Driver Lifecycle
*/
int tsl2561_init(void)
{
	s32 value;
	int result = 0;

	printk(KERN_INFO "loading tsl2561 driver v 0.1\n");

finish:
	return result;
}

void tsl2561_exit(void)
{

}

module_init(tsl2561_init);
module_exit(tsl2561_exit);
