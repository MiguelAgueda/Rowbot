#include <unistd.h>        //Needed for I2C port
#include <fcntl.h>         //Needed for I2C port
#include <sys/ioctl.h>     //Needed for I2C port
#include <linux/i2c-dev.h> //Needed for I2C port
#include <stdio.h>
#include <stdlib.h>
#include <iostream>


struct I2C_IN_PACK
{
	bool DriveAutonomously = false;
	bool SaveScanData = false;
	bool Shutdown = false;
	int8_t joystick_x = 0;
	int8_t joystick_y = 0;
};

struct I2C_OUT_PACK
{
	int8_t virt_joystick_x = 0;
	int8_t virt_joystick_y = 0;
};

bool connect_i2c();  // Initialize I2C connection. 
I2C_IN_PACK rdwr_buffer(I2C_OUT_PACK);
