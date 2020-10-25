/*
 * RPLiDAR access functions implementation.
 * 
 * This module handles communication with the RPLiDAR A2.
 */

#include <lidarmodule.hpp>

using namespace rp::standalone::rplidar;

RPlidarDriver *driver; // Initialize pointer for LiDAR driver.
u_result op_result;

std::fstream LIDAR_FILE;  // For saving IMU data.
std::chrono::steady_clock::time_point t_start;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

bool LIDAR_ALREADY_SETUP = false;

/* This method collects the current health information from the RPLiDAR device. 
 *
 * Returns
 * -------
 * 		True - Device is okay, ready to go.
 * 		False - Device could not be connected to / not in good health.
 */
bool checkRPLIDARHealth(RPlidarDriver *driver)
{
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

/* Runs setup routine for RPLiDAR.
 *
 * Assigns global pointer, `driver`, to an instance of RPLiDAR driver on
 * the specified port with baudrate 115200; the baudrate for RPLiDAR A2.
 */
void setup_lidar()
{
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
	    opt_com_path = "/dev/ttyUSB1";
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

    const char *lidar_file = "/home/rowbot/Documents/Rowbot/datasets/test/lidar.csv";
    LIDAR_FILE.open(lidar_file, std::fstream::out);
    t_start = std::chrono::steady_clock::now();

	LIDAR_ALREADY_SETUP = true;
}

/*
 * Shutdown LiDAR scanner, stop motor.
 */
void shutdown_lidar()
{
	driver->stop();  // Stop scanning.
	driver->stopMotor();  // Stop spinning RPLiDAR motor.
    LIDAR_FILE.close();
    std::cout << "Shutting Down LiDAR.\n";
}

/*
 * Convert angular measurement from degrees to radians.
 * 
 * Parameters
 * ----------
 *      float : deg
 *          Measurement to be converted, in units of degrees.
 * 
 * Returns
 * -------
 *      float : rad
 *          Converted measurement, in units of radians.
 */
float deg2rad(float deg)
{
    float rad = deg * 3.1415926 / 180;
    return rad;
}

/*
 * Process LiDAR output into PCL point cloud.
 * 
 * Returns
 * -------
 *      pcl::PointCloud::Ptd : Pointer to latest point cloud data.
 * 
 * Assumptions
 * -----------
 *      The rover's environment is flat.
 * 
 *      Since the RPLiDAR A2 is a 2D range scanner, the z-coordinate for every
 *          point is set to zero.
 *      This effect is not accounted for in any way by the ICP algorithm,
 *          leading to a zeroing of the EKF Z-coordinate via corrective update.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr get_lidar_data(bool logging, float t_from_start)
{
	rplidar_response_measurement_node_hq_t nodes[1024]; // Create response node of size 1024.
	size_t count = _countof(nodes);

	op_result = driver->grabScanDataHq(nodes, count); // Hq method coincides with rplidar_..._node_hq_t.

	if (IS_FAIL(op_result))
	{
		printf("Failed to get scan data.");
        pcl::PointCloud<pcl::PointXYZ>::Ptr Empty (new pcl::PointCloud<pcl::PointXYZ>);
        return Empty;
	}
	else
	{
        int n_rays = (int)count;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(n_rays, 1));
        Eigen::MatrixXf raw_lidar(2, n_rays);
        int i = 0;
		for (auto& point : *cloud)
		{
			float theta = nodes[i].angle_z_q14 * 90.f / (1 << 14);
			float dist = nodes[i].dist_mm_q2 / (1 << 2);
            dist = dist / 1000;  // Convert distance from [mm] to [m].

            raw_lidar(0,i) = theta;
            raw_lidar(1,i) = dist;
            point.x = dist * std::cos(deg2rad(theta));
            point.y = dist * std::sin(deg2rad(theta));
            point.z = 0; 
            ++i;
		}

        if (logging)
        {
            // const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, 0, ", ", ",", "[", "]", "\n");
            const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", "", "", ";\n", "", "\n");
            LIDAR_FILE << t_from_start << ";\n";
            LIDAR_FILE << raw_lidar.format(CSVFormat);
        }

        return cloud;
	}
}