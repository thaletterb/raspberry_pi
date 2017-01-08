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

typedef struct sensor_data_t
{
	struct input_dev *device;
}sensor_data_t;

sensor_data_t *sensor_data;

/*
*	I2C Setup
*/

/*
*	Device Setup
*/

static int setup_device(void)
{
	int result = 0;

	// allocate memory to store our pad data
	sensor_data = kzalloc(sizeof(sensor_data_t), GFP_KERNEL);

	// create the device
	sensor_data->device = input_allocate_device();
	if(!sensor_data->device)
	{
		printk(KERN_ERR "Not enough memory\n");
		return -1;
	}

	// setup device description
	sensor_data->device->name = "TSL2561";
	sensor_data->device->phys = "TSL2561_phys";
	sensor_data->device->uniq = "TSL2561_uniq";

	sensor_data->device->evbit[0] = BIT_MASK(EV_ABS);

	// setup the sensor
	// setup_device_axis(pad_data->device, ABS_X);
	// setup_device_axis(pad_data->device, ABS_Y);

	result = input_register_device(sensor_data->device);
	if(result)
	{
		printk(KERN_ERR "Failed to register tsl2561 device. Result: %d\n", result);
	}

	return result;
}

/*	
*	Driver Lifecycle
*/
int tsl2561_init(void)
{
	s32 value;
	int result = 0;

	result = setup_device();
	if(result)
	{
		printk(KERN_INFO "FAIL: Could not setup tsl2561 device. Result: %d\n", result);
		// input_free_device(pad_data->device)
		goto finish;
	}

	printk(KERN_INFO "loading tsl2561 driver v 0.1\n");

finish:
	return result;
}

void tsl2561_exit(void)
{
	printk(KERN_INFO "unloading tsl2561 driver v 0.1\n");
}

module_init(tsl2561_init);
module_exit(tsl2561_exit);
