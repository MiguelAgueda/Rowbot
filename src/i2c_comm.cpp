#include "i2c_comm.h"

char *filename = (char *)"/dev/i2c-1"; // MCU connected to I2C bus number 1.
int addr = 0x77;											 // The I2C address of the MCU.
int file_i2c;


bool connect_i2c()
{
	/*
	Connect to I2C bus and establish connection to slave device on `addr`.
	Returns:
		`true`: Connection successful.
		`false`: Connection not successful.
	*/
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		printf("Failed to open the i2c bus. Error No. %d\n", file_i2c);
		return false;
	}

	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		return false;
	}

	return true;
}

bool write_buffer(I2C_OUT_PACK buf_to_write)
{
	/*
	Helper function for writing to I2C bus.
	*/
	struct I2C_OUT_PACK* buf_ptr;
	buf_ptr = &buf_to_write;
	int length = sizeof(buf_to_write); // Number of bytes to write to MCU.
	// if (!connect_i2c())								 // Connect to I2C bus.
	// 	return false;

	//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
	if (write(file_i2c, buf_ptr, length) != length)
	{
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
		return false;
	}

	return true;
}

I2C_IN_PACK read_buffer()
{
	/*
	Helper function for reading from I2C bus.
	*/
	I2C_IN_PACK return_package;
	size_t size = sizeof(return_package);
	int8_t input_buffer[size];

	if (read(file_i2c, input_buffer, size) != size)
	{
		/* ERROR HANDLING: i2c transaction failed */
		printf("I2C: Failure to read all bytes in structure, from read_buffer().\n");
		return return_package;
	}
	// Debugging I2C
	// for (int i = 0; i < size; i++)  // Loop over `input_buffer`.
	// {
	// 	std::cout << "Buffer Index " << (int)i << ": " << (int)input_buffer[i] << "\n";
	// }
	// Access data in order of I2C_IN_PACK.
	return_package.DriveAutonomously = input_buffer[0];
	return_package.SaveScanData = input_buffer[1];
	return_package.Shutdown = input_buffer[2];
	return_package.joystick_x = input_buffer[3];
	return_package.joystick_y = input_buffer[4];

	return return_package;
}

I2C_IN_PACK rdwr_buffer(I2C_OUT_PACK buf_to_write)
{
	/*
	Write and Read I2C data. Before calling `rdwr_buffer`, call `connect_i2c`.
	Parameter:
		I2C_OUT_PACK: Object to write over I2C.

	Return:
		I2C_IN_PACK: Object read over I2C. 
	*/
	write_buffer(buf_to_write);
	I2C_IN_PACK return_data = read_buffer();
	return return_data;
}