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

#define I2C_PERIPHERAL_NUM		0	// use for rev 1
#define REFRESH_RATE_MSECS		3000	// sample every 3 secs

typedef struct sensor_data_t
{
	struct input_dev *device;
}sensor_data_t;

sensor_data_t *sensor_data;

/*
*	I2C Setup
*/

struct i2c_adapter* i2c_dev;
struct i2c_client* i2c_client;

static struct timer_list i2c_timer;

static struct i2c_board_info __initdata board_info[] = {
	{
		I2C_BOARD_INFO("TSL2561", 0x39),
	}
};

static void i2c_work_handler(struct work_struct* work)
{
	printk(KERN_INFO "Doing Work\n");
}

DECLARE_WORK(i2c_work, i2c_work_handler);

static void i2c_timer_callback(unsigned long data)
{
	schedule_work(&i2c_work);
	mod_timer(&i2c_timer, jiffies + msecs_to_jiffies(REFRESH_RATE_MSECS));
}

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
int __init tsl2561_init(void)
{
	s32 value;
	int result = 0;

	printk(KERN_INFO "loading tsl2561 driver v 0.1\n");

	result = setup_device();
	if(result)
	{
		printk(KERN_INFO "FAIL: Could not setup tsl2561 device. Result: %d\n", result);
		input_free_device(sensor_data->device);
		goto finish;
	}

	// setup device
	i2c_dev = i2c_get_adapter(I2C_PERIPHERAL_NUM);	// TODO - VERIFY 
	i2c_client = i2c_new_device(i2c_dev, board_info);

	// setup timer
	setup_timer(&i2c_timer, i2c_timer_callback, 0);
	result = mod_timer(&i2c_timer, jiffies + msecs_to_jiffies(REFRESH_RATE_MSECS));
	
	if(result)
	{
		printk(KERN_INFO "FAIL: Timer not setup. Result %d\n", result);
		goto finish;
	}

	// initialize with control call to set mode as output
	// value = i2c_smbus_write_byte_data(i2c_client, 0x40, 0)
	// printk(KERN_INFO "Write Control, result: %d\n", value);
	// value = i2c_smbus_read_byte(i2c_client)
	// printk(KERN_INFO "Flush value: %d\n", value);
finish:
	return result;
}

void __exit tsl2561_exit(void)
{
	printk(KERN_INFO "unloading tsl2561 driver v 0.1\n");

	input_unregister_device(sensor_data->device);
	input_free_device(sensor_data->device);

	i2c_unregister_device(i2c_client);

	del_timer(&i2c_timer);

	kzfree(sensor_data);	// free the mem
}

module_init(tsl2561_init);
module_exit(tsl2561_exit);
