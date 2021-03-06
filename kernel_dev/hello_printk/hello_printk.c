/*
 * "Hello, world!" minimal kernel module
 *
 * Valerie Henson <val@nmt.edu>
 *
 */

/*
 * The below are header files provided by the kernel which are
 * required for all modules.  They include things like the definition
 * of the module_init() macro.
 */
#include <linux/init.h>
#include <linux/kernel.h>	// Needed for KERN_INFO
#include <linux/module.h>

static int hello_data __initdata = 3;	// __initdata is similar to __init macro, but for vars instead of funcs
										// __init causes the init function to be discared and its mem freed once the init function finished for built in drivers 

/*
 * This is the init function, which is run when the module is first
 * loaded.  The __init keyword tells the kernel that this code will
 * only be run once, when the module is loaded.
 */

static int __init
hello_init(void)
{
	printk(KERN_INFO "Hello, world! %d\n", hello_data);
	return 0;
}

/*
 * The below macro informs the kernel as to which function to use as
 * the init function.
 */

module_init(hello_init);

/*
 * Similary, the exit function is run once, upon module unloading, and
 * the module_exit() macro identifies which function is the exit
 * function.
 */

static void __exit
hello_exit(void)
{
	printk("Goodbye, world!\n");
}

module_exit(hello_exit);

/*
 * MODULE_LICENSE() informs the kernel what license the module source
 * code is under, which affects which symbols it may access in the
 * main kernel.  Certain module licenses will "taint" the kernel,
 * indicating that non-open or untrusted code has been loaded.
 * Modules licensed under GPLv2 do not taint the kernel and can access
 * all symbols, but declaring it so is a legal statement that the
 * source code to this module is licensed under GPLv2, and so you must
 * provide the source code if you ship a binary version of the module.
 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Valerie Henson <val@nmt.edu>");
MODULE_DESCRIPTION("\"Hello, world!\" minimal module");
MODULE_VERSION("printk");
