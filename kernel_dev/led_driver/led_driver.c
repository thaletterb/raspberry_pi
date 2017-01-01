/* 
 * "Hello World - LED" kernel module
 *
 *	github.com/thaletterb
 * 
 *	follows: http://sysprogs.com/VisualKernel/tutorials/raspberry/leddriver/
 */

/*
 * The below are header files provided by the kernel which are
 * required for all modules.  They include things like the definition
 * of the module_init() macro.
 */
#include <linux/init.h>
#include <linux/module.h>

#include <linux/kernel.h>   // Needed for KERN_INFO
#include <asm/io.h>
#include <mach/platform.h>	// Provides the GPIO base address

#include <linux/timer.h>	// Provides timers
#include <linux/err.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brian Vuong");
MODULE_DESCRIPTION("Toggles a GPIO through direct register values");
MODULE_VERSION("0.1");

static struct timer_list s_blink_timer;	// use timers instead of sleep()
static int s_blink_period = 1000;	
static const int led_gpio_pin = 18;			// GPIO 18


// See section 6.1 of BCM2835-ARM_Peripherals.pdf
// Registers start at addres 0x7E20 0000
typedef struct gpio_registers_t
{
	uint32_t GPFSEL[6];	// GPIO Function Select - Configures GPIOs as input/outpu
	uint32_t Reserved1;
	uint32_t GPSET[2];	// GPIO Pin Output Set
	uint32_t Reserved2;	
	uint32_t GPCLR[2];	// GPIO Pin Output Clear
} gpio_registers_t; 

gpio_registers_t *s_p_gpio_registers;

/** @brief: There are 54 GPIOs, spread over 6 GPFSELn registers
*			Determine which GPFSELn register and bit controls the given GPIO
*			and set the corresponding 3 bit function_code (i.e input/output)
*/
static void set_gpio_function(int GPIO, int function_code)
{
	int register_index = GPIO / 10;	// GPIO 1 -> register 0
	int bit = (GPIO % 10) * 3;		// GPIO 1 -> bit 3-5

	unsigned old_value = s_p_gpio_registers->GPFSEL[register_index];
	unsigned mask = 0b111 << bit;

	printk("Changing Function of GPIO: %d from %x to %x\n", GPIO, (old_value>>bit) & 0b111, function_code);

	s_p_gpio_registers->GPFSEL[register_index] = (old_value & ~mask) | ((function_code << bit) & mask);
}

static void set_gpio_output_value(int GPIO, bool output_value)
{
	if(output_value)
	{
		s_p_gpio_registers->GPSET[GPIO / 32] = (1<<(GPIO % 32));
	}
	else
	{
		s_p_gpio_registers->GPCLR[GPIO / 32] = (1<<(GPIO % 32));
	}
}

static void blink_timer_handler(unsigned long unused)
{
	static bool on = false;
	on = !on;
	set_gpio_output_value(led_gpio_pin, on);
	mod_timer(&s_blink_timer, jiffies + msecs_to_jiffies(s_blink_period));
}

/*
 * This is the init function, which is run when the module is first
 * loaded.  The __init keyword tells the kernel that this code will
 * only be run once, when the module is loaded.
 */
static int __init led_blink_module_init(void)
{
	int result;

	printk("Blink Init\n");

	s_p_gpio_registers = (gpio_registers_t *)__io_address(GPIO_BASE);
	set_gpio_function(led_gpio_pin, 0b001);	// Set as output

	setup_timer(&s_blink_timer, blink_timer_handler, 0);
	result = mod_timer(&s_blink_timer, jiffies + msecs_to_jiffies(s_blink_period));
	
	BUG_ON(result < 0);
}

/*
 * The below macro informs the kernel as to which function to use as
 * the init function.
 */
module_init(led_blink_module_init);

/*
 * Similary, the exit function is run once, upon module unloading, and
 * the module_exit() macro identifies which function is the exit
 * function.
 */
static void __exit led_blink_module_exit(void)
{
	printk("Blink Exit\n");
	set_gpio_function(led_gpio_pin, 0);	// Configure pin as input
	del_timer(&s_blink_timer);
}

module_exit(led_blink_module_exit);
