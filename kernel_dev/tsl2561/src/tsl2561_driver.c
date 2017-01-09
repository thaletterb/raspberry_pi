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

#include <linux/delay.h>		// allows for msleep

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brian Vuong");
MODULE_DESCRIPTION("Controls a TSL2561 Lux Sensor");
MODULE_VERSION("0.1");

#define I2C_PERIPHERAL_NUM		0	// use for rev 1
#define REFRESH_RATE_MSECS		3000	// sample every 3 secs

typedef struct sensor_data_t
{
	uint8_t gain;
	uint8_t integration_time;
	bool autogain;
	uint8_t type;
	struct input_dev *device;
}sensor_data_t;

sensor_data_t *sensor_data;

#define TO_TSL(x)	(tsl2561_t*) x

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
	long lux = tsl2561_lux((void*)sensor_data);
	printk(KERN_INFO "Doing Work\n");
	printk(KERN_INFO "%lu\n", lux);
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

/* 
 * Autogain thresholds
 */
#define TSL2561_AGC_THI_13MS 4850	// Max value at Ti 13ms = 5047
#define TSL2561_AGC_TLO_13MS 100	
#define TSL2561_AGC_THI_101MS 36000 // Max value at Ti 101ms = 37177
#define TSL2561_AGC_TLO_101MS 200
#define TSL2561_AGC_THI_402MS 63000	// Max value at Ti 402ms = 65535
#define TSL2561_AGC_TLO_402MS 500	

/*
 * Clipping thresholds
 */
#define TSL2561_CLIPPING_13MS 4900
#define TSL2561_CLIPPING_101MS 37000
#define TSL2561_CLIPPING_402MS 65000

/*
 * Constats for simplified lux calculation
 * according TAOS Inc.
 */
#define LUX_SCALE 14
#define RATIO_SCALE 9

#define CH_SCALE 10
#define CH_SCALE_TINT0 0x7517
#define CH_SCALE_TINT1 0x0FE7

/* 
 * T, FN, and CL Package coefficients
 */
#define TSL2561_K1T 0x0040
#define TSL2561_B1T 0x01F2
#define TSL2561_M1T 0x01BE
#define TSL2561_K2T 0x0080
#define TSL2561_B2T 0x0214
#define TSL2561_M2T 0x02D1
#define TSL2561_K3T 0x00C0
#define TSL2561_B3T 0x023F
#define TSL2561_M3T 0x037B
#define TSL2561_K4T 0x0100
#define TSL2561_B4T 0x0270
#define TSL2561_M4T 0x03FE
#define TSL2561_K5T 0x0138
#define TSL2561_B5T 0x016F
#define TSL2561_M5T 0x01fC
#define TSL2561_K6T 0x019A
#define TSL2561_B6T 0x00D2
#define TSL2561_M6T 0x00FB
#define TSL2561_K7T 0x029A
#define TSL2561_B7T 0x0018
#define TSL2561_M7T 0x0012
#define TSL2561_K8T 0x029A
#define TSL2561_B8T 0x0000
#define TSL2561_M8T 0x0000



/* 
 * CS package coefficients
 */
#define TSL2561_K1C 0x0043
#define TSL2561_B1C 0x0204
#define TSL2561_M1C 0x01AD
#define TSL2561_K2C 0x0085
#define TSL2561_B2C 0x0228
#define TSL2561_M2C 0x02C1
#define TSL2561_K3C 0x00C8
#define TSL2561_B3C 0x0253
#define TSL2561_M3C 0x0363
#define TSL2561_K4C 0x010A
#define TSL2561_B4C 0x0282
#define TSL2561_M4C 0x03DF
#define TSL2561_K5C 0x014D
#define TSL2561_B5C 0x0177
#define TSL2561_M5C 0x01DD
#define TSL2561_K6C 0x019A
#define TSL2561_B6C 0x0101
#define TSL2561_M6C 0x0127
#define TSL2561_K7C 0x029A
#define TSL2561_B7C 0x0037
#define TSL2561_M7C 0x002B
#define TSL2561_K8C 0x029A
#define TSL2561_B8C 0x0000
#define TSL2561_M8C 0x0000

/*
*	 Helper functions
*/

// Prototypes for helper functions
uint8_t tsl2561_write_byte_data(void *_tsl, uint8_t reg, uint8_t value);
uint16_t tsl2561_write_word_data(void *_tsl, uint8_t reg, uint8_t value);
int16_t tsl2561_read_word_data(void *_tsl, uint8_t cmd);
unsigned long tsl2561_compute_lux(void *_tsl, int visible, int channel1);


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

// sets the time integration and gain value 
void tsl2561_set_timing(void *_tsl, int integration_time, int gain)
{
	// tsl2561_t *tsl = TO_TSL(_tsl);

	// update values
	 sensor_data->integration_time = integration_time;
	 sensor_data->gain = gain;

	tsl2561_write_byte_data(i2c_client, TSL2561_CMD_BIT | TSL2561_REG_TIMING, integration_time | gain);
}

// sets the gain value
void tsl2561_set_gain(void *_tsl, int gain)
{
	// tsl2561_t *tsl = TO_TSL(_tsl);
	tsl2561_set_timing(i2c_client, sensor_data->integration_time, gain);
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

// reads value from tsl2561 sensor
void tsl2561_read(void *_tsl, int *broadband, int *ir)
{
	tsl2561_enable();
	// tsl2561_t *tsl = TO_TSL(_tsl);

	// wait until ADC is complete
	switch(sensor_data->integration_time)
	{
		case TSL2561_INTEGRATION_TIME_402MS:
			msleep_interruptible(402);
			break;
		case TSL2561_INTEGRATION_TIME_101MS:
			msleep_interruptible(102);
			break;
		case TSL2561_INTEGRATION_TIME_13MS:
			msleep_interruptible(14);
			break;
		default:
			msleep_interruptible(402);
			break;
	}

	*broadband = tsl2561_read_word_data(_tsl, TSL2561_CMD_BIT | TSL2561_WORD_BIT | TSL2561_REG_CH0_LOW);
	*ir = tsl2561_read_word_data(_tsl, TSL2561_CMD_BIT | TSL2561_WORD_BIT | TSL2561_REG_CH1_LOW);
	
	if( *broadband < 0 || *ir < 0){
		printk("error: i2c_smbus_read_word_data() failed\n");
	} else {
		printk("bb=%i, ir=%i\n", *broadband, *ir);
	}
	tsl2561_disable(_tsl);
}

// Computes a lux value for this sensor
long tsl2561_lux(void *_tsl)
{
	// tsl2561_t *tsl = TO_TSL(_tsl);
	int visible, channel1, threshold;
	tsl2561_luminosity(i2c_client, &visible, &channel1);

	switch(sensor_data->integration_time)
	{
		case TSL2561_INTEGRATION_TIME_13MS:
			threshold = TSL2561_CLIPPING_13MS;
			break;
		case TSL2561_INTEGRATION_TIME_101MS:
			threshold = TSL2561_CLIPPING_101MS;
			break;
		default:
			threshold = TSL2561_CLIPPING_402MS;
			break;
	}

	if((visible > threshold) || (channel1 > threshold))
	{
		printk("Here\n");
		return 0;
	}

	return tsl2561_compute_lux(sensor_data, visible, channel1);
}

void tsl2561_luminosity(void *_tsl, int *channel0, int *channel1)
{
	// tsl2561_t *tsl = TO_TSL(_tsl);
	uint16_t hi, lo;
	bool agc_check = false, valid = false;

	if(!sensor_data->autogain)
	{
		tsl2561_read(_tsl, channel0, channel1);
		return;
	}

	while(!valid)
	{
		switch(sensor_data->integration_time)
		{
			case TSL2561_INTEGRATION_TIME_13MS:
				hi = TSL2561_AGC_THI_13MS;
				lo = TSL2561_AGC_TLO_13MS;
				break;

			case TSL2561_INTEGRATION_TIME_101MS:
				hi = TSL2561_AGC_THI_101MS;
				lo = TSL2561_AGC_TLO_101MS;
				break;

			default:
				hi = TSL2561_AGC_THI_402MS;
				lo = TSL2561_AGC_TLO_402MS;
				break;
		}
	}

	tsl2561_read(_tsl, channel0, channel1);
	if(!agc_check)
	{
		if((*channel0 < lo) && (sensor_data->gain == TSL2561_GAIN_0X)) {
			tsl2561_set_gain(_tsl, TSL2561_GAIN_16X);
			tsl2561_read(_tsl, channel0, channel1);
			agc_check = true;

		} else if((*channel0 > hi) && (sensor_data->gain == TSL2561_GAIN_16X)) {
			tsl2561_set_gain(_tsl, TSL2561_GAIN_0X);
			tsl2561_read(_tsl, channel0, channel1);
			agc_check = true;
		} else {
			valid = true;
		}
	} 
	else 
	{
		valid = true;
	}
	
}

// helper function for computing lux values
unsigned long tsl2561_compute_lux(void *_tsl, int ch0, int ch1)
{
	//tsl2561_t *tsl = TO_TSL(_tsl);
	unsigned long ch_scale, channel0, channel1;

	unsigned long tmp;
	unsigned long lux;
	unsigned long ratio = 0, ratio1 = 0;
	int b, m;

	// first scale the channel values depending on gain and integration time
	switch(sensor_data->integration_time)
	{
		case TSL2561_INTEGRATION_TIME_13MS:
			ch_scale = CH_SCALE_TINT0;
			break;
		case TSL2561_INTEGRATION_TIME_101MS:
			ch_scale = CH_SCALE_TINT1;
			break;
		default:
			ch_scale = (1 << CH_SCALE);
			break;
	}

	// scale if gain is not 16x
	if(!sensor_data->gain)
	{	
		ch_scale = (ch_scale << 4);	// scale 1x to 16x
	}

	//scale the channel values

	channel0 = (ch0 * ch_scale) >> CH_SCALE;
	channel1 = (ch1 * ch_scale) >> CH_SCALE;

	
	// find the ratio of the channel values and protect against div by 0
	if(channel0 != 0)
		ratio1 = (channel1 << (RATIO_SCALE+1)) / channel1;

	// round the ratio value
	ratio = (ratio1 + 1) >> 1;


	switch(sensor_data->type)
	{
		case 1:
			if((ratio >= 0) && (ratio <= TSL2561_K1C)){ 
				b = TSL2561_B1C; m = TSL2561_M1C; 
			} else if(ratio <= TSL2561_K2C) {
				b = TSL2561_B2C; m = TSL2561_M2C;
			} else if(ratio <= TSL2561_K3C) {
				b = TSL2561_B3C; m = TSL2561_M3C;
			} else if(ratio <= TSL2561_K4C) {
				b = TSL2561_B4C; m = TSL2561_M4C;
			} else if (ratio <= TSL2561_K5T) {
				b = TSL2561_B5C; m = TSL2561_M5C;
			} else if(ratio <= TSL2561_K6T) {
				b = TSL2561_B6C; m = TSL2561_M6C;
			} else if(ratio <= TSL2561_K7T) {
				b = TSL2561_B7C; m = TSL2561_M7C;
			} else if(ratio > TSL2561_K8C) {
				b = TSL2561_B8C; m = TSL2561_M8C;
			}
			break;
		case 0:
		default:
			if((ratio >= 0) && (ratio <= TSL2561_K1T)){ 
				b = TSL2561_B1T; m = TSL2561_M1T;
			} else if(ratio <= TSL2561_K2T) {
				b = TSL2561_B2T; m = TSL2561_M2T;
			} else if(ratio <= TSL2561_K3T) {
				b = TSL2561_B3T; m = TSL2561_M3T;
			} else if(ratio <= TSL2561_K4T) {
				b = TSL2561_B4T; m = TSL2561_M4T;
			} else if (ratio <= TSL2561_K5T) {
				b = TSL2561_B5T; m = TSL2561_M5T;
			} else if(ratio <= TSL2561_K6T) {
				b = TSL2561_B6T; m = TSL2561_M6T;
			} else if(ratio <= TSL2561_K7T) {
				b = TSL2561_B7T; m = TSL2561_M7T;
			} else if(ratio > TSL2561_K8T) {
				b = TSL2561_B8T; m = TSL2561_M8T;
			}
			break;
	}

	tmp = (channel0 * b) - (channel1 * m);

	if(tmp < 0) 
		tmp = 0;

	tmp += (1 << (LUX_SCALE-1));
	lux = (tmp >> LUX_SCALE);

	return lux;
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

	// set init sensor values
	sensor_data->gain = TSL2561_GAIN_0X;
	//sensor_data->integration_time = TSL2561_INTEGRATION_TIME_402MS; 
	sensor_data->integration_time = TSL2561_INTEGRATION_TIME_13MS; 
	//sensor_data->autogain = true;
	sensor_data->autogain = false;
	sensor_data->type = 0;

	tsl2561_enable();
	tsl2561_set_timing((void*)sensor_data, sensor_data->integration_time, sensor_data->gain);
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
