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
		//I2C_BOARD_INFO("TSL2561", 0x40),	// test wrong addy - ERROR helper_write8()
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

#define TSL2561_I2C_ADDR_LOW 0x29
#define TSL2561_I2C_ADDR_DEFAULT 0x39
#define TSL2561_I2C_ADDR_HIGH 0x49

#define TSL2561_INTEGRATION_TIME_13MS 0x00
#define TSL2561_INTEGRATION_TIME_101MS 0x01
#define TSL2561_INTEGRATION_TIME_402MS 0x02

#define TSL2561_GAIN_0X 0x00
#define TSL2561_GAIN_16X 0x10

/*
 * TSL2561 constants.
 */
#define TSL2561_REG_CTRL 0x00
#define TSL2561_REG_TIMING 0x01

#define TSL2561_REG_CH0_LOW 0x0C
#define TSL2561_REG_CH0_HIGH 0x0D
#define TSL2561_REG_CH1_LOW 0x0E
#define TSL2561_REG_CH1_HIGH 0x0F

#define TSL2561_CMD_BIT (0x80)
#define TSL2561_WORD_BIT (0x20)

#define TSL2561_CTRL_PWR_ON 0x03
#define TSL2561_CTRL_PWR_OFF 0x00


int tsl2561_enable(void);
int tsl2561_disable(void *_tsl);

//void* tsl2561_init(int address, const char *i2c_device_filepath);
//void tsl2561_close(void *_tsl);

void tsl2561_set_timing(void *_tsl, int integration_time, int gain);
void tsl2561_set_gain(void *_tsl, int gain);
void tsl2561_set_integration_time(void *_tsl, int ingeration_time);
void tsl2561_set_type(void *_tsl, int type);

void tsl2561_read(void *_tsl, int *visible, int *ir);
long tsl2561_lux(void *_tsl);
void tsl2561_luminosity(void *_tsl, int *visible, int *ir);
	
void tsl2561_enable_autogain(void *_tsl);
void tsl2561_disable_autogain(void *_tsl);	

/*
*	 Helper functions
*/

// Prototypes for helper functions
uint8_t tsl2561_write_byte_data(void *_tsl, uint8_t reg, uint8_t value);
uint16_t tsl2561_write_word_data(void *_tsl, uint8_t reg, uint8_t value);
int16_t tsl2561_read_word_data(void *_tsl, uint8_t cmd);


// writes a byte on the i2c bus
uint8_t tsl2561_write_byte_data(void *_tsl, uint8_t reg, uint8_t value)
{
	// tsl2561_t *tsl = TO_TSL(_tsl);

	uint8_t data = i2c_smbus_write_byte_data(i2c_client, reg, value);
	
	if(data<0)
	{
		printk("Error: helper_write8()\n");
	}
	return data;
}

// writes a word on the i2c bus
uint16_t tsl2561_write_word_data(void *_tsl, uint8_t reg, uint8_t value)
{
	// tsl2561_t *tsl = TO_TSL(_tsl);	// use i2c_client instead 
	uint16_t data = i2c_smbus_write_word_data(i2c_client, reg, value);

	if(data<0)
	{
		printk("Error: helper_write16()\n");
	}
	return data;
}

// reads a word from the i2c bus
int16_t tsl2561_read_word_data(void *_tsl, uint8_t reg)
{
	// tsl2561_t *tsl = TO_TSL(_tsl);
	// int error = tsl2561_set_addr(_tsl);
	// if(error<0) 
	//	return -1

	int16_t data = i2c_smbus_read_word_data(i2c_client, reg);
	// DEBUG
	return data;
}

// enables TSL2561 sensor
int tsl2561_enable(void)
{
	return tsl2561_write_byte_data(i2c_client, TSL2561_CMD_BIT | TSL2561_REG_CTRL, TSL2561_CTRL_PWR_ON);
}

// disables TSL2561 sensor
int tsl2561_disable(void *_tsl)
{
	return tsl2561_write_byte_data(i2c_client, TSL2561_CMD_BIT | TSL2561_REG_CTRL, TSL2561_CTRL_PWR_OFF);
}

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
	if(result<0)
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
	if(result<0)
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
	
	if(result<0)
	{
		printk(KERN_INFO "FAIL: Timer not setup. Result %d\n", result);
		goto finish;
	}

	// initialize with control call to set mode as output
	// value = i2c_smbus_write_byte_data(i2c_client, 0x40, 0)
	// printk(KERN_INFO "Write Control, result: %d\n", value);
	// value = i2c_smbus_read_byte(i2c_client)
	// printk(KERN_INFO "Flush value: %d\n", value);
	tsl2561_enable();
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
