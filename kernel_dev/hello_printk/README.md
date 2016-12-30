# hello_printk

"Hello World" kernel module example. Use printk() to print debug messages to the kernel

## Setup

- Update/Upgrade using 
  -`$ sudo apt-get update -y && $ sudo apt-get upgrade -y
- Update the kernel using 
  -`$ sudo rpi-update
- Reboot the pi using 
  -`$ sudo reboot
- Get rpi source:
  -`$ sudo wget https://raw.githubusercontent.com/notro/rpi-source/master/rpi-source -O /usr/bin/rpi-source
- Make it executeable
  -` $ sudo  chmod +x /usr/bin/rpi-source
- Tag as latest version
  - $ /usr/bin/rpi-source -q --tag-update
- Get the kernel files and headers
  - $ rpi-source

## Make the modules
- Run make and check for any errors
  - `$ make

## Load and Unload 
- Load the kernel module using insmod
  - `$ sudo insmod hello_printk.ko
- Check for the "Hello, World!" kernel messages using: 
  - `$ dmesg
- If successful, unload the kernel modules using rmmod and look for the "Goodbye, world" message
  - `$ sudo rmmod hello_printk.ko
- 
