#include <lidarmodule.hpp>

using namespace rp::standalone::rplidar;

RPlidarDriver *driver; // Initialize pointer for LiDAR driver.
u_result op_result;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

bool LIDAR_ALREADY_SETUP = false;

bool checkRPLIDARHealth(RPlidarDriver *driver)
{
	/* This method collects the current health information from the RPLiDAR 
	 * device. 
	 * Returns:
	 * 		true - Device is okay, ready to go.
	 * 		false - Device could not be connected to / Device not in good health.
	 */
	u_result op_result;
	rplidar_response_device_health_t healthinfo;

	op_result = driver->getHealth(healthinfo);
	if (IS_OK(op_result))
	{
		// printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR)
		{
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			// driver->reset();
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}

void setup_lidar()
{
	/* Runs setup routine for RPLiDAR.
	 * Assigns global pointer, `driver`, to an instance of RPLiDAR driver on
	 * the specified port with baudrate 115200; the baudrate for RPLiDAR A2.
	 */
	// if (LIDAR_ALREADY_SETUP)
	// 	return;

	const char *opt_com_path = NULL;
	opt_com_path = "/dev/ttyUSB0";

	// Create the driver instance.
	driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (!driver) // If driver creation was not successful,
	{
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}

	rplidar_response_device_info_t devinfo;
	bool connectSuccess = false;
	// make connection...
	driver = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (IS_OK(driver->connect(opt_com_path, 115200)))
	{
		op_result = driver->getDeviceInfo(devinfo);

		if (IS_OK(op_result))
		{
			connectSuccess = true;
		}
		else
		{
			delete driver;
			driver = NULL;
		}
	}

	if (!connectSuccess)
	{
		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_com_path);

		RPlidarDriver::DisposeDriver(driver);
		driver = NULL;
	}

	if (!checkRPLIDARHealth(driver))
	{
		std::cout << "LiDAR did not pass health check!\n";
		RPlidarDriver::DisposeDriver(driver);
		driver = NULL;
	}
	driver->startMotor();
	// Setup for scan process.
	std::vector<RplidarScanMode> scanModes;
	driver->getAllSupportedScanModes(scanModes);
	driver->startScanExpress(false, scanModes[1].id);

	LIDAR_ALREADY_SETUP = true;
}

void shutdown_lidar()
{
	driver->stop();			 // Stop scanning.
	driver->stopMotor(); // Stop spinning RPLiDAR motor.
}

float deg2rad(float deg)
{
    return (deg * arma::datum::pi / 180);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_lidar_data()
{
	/*
	Retrieve a scan from the LiDAR.
	Format scan into series of two vectors.
	Return DataToSave object containing formatted LiDAR data.
	*/
	rplidar_response_measurement_node_hq_t nodes[1024]; // Create response node of size 1024.
	size_t count = _countof(nodes);

	op_result = driver->grabScanDataHq(nodes, count); // Hq method coincides with rplidar_..._node_hq_t.

	if (IS_FAIL(op_result))
	{
		printf("Failed to get scan data.");
		// arma::fmat Empty(3, 1, arma::fill::zeros);
        pcl::PointCloud<pcl::PointXYZ>::Ptr Empty (new pcl::PointCloud<pcl::PointXYZ>);
        return Empty;
	}
	else
	{
        int n_rays = (int)count;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(n_rays, 1));
        int i = 0;
		for (auto& point : *cloud)
		{
			float theta = nodes[i].angle_z_q14 * 90.f / (1 << 14);
			float dist = nodes[i].dist_mm_q2 / (1 << 2);
            dist = dist / 1000;  // Convert distance from [mm] to [m].
            point.x = dist * std::cos(deg2rad(theta));
            point.y = dist * std::sin(deg2rad(theta));
            point.z = 0; 
            ++i;
		}
    
    return cloud;
	}
}