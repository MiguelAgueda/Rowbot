#include <rplidar.h> //RPLIDAR standard sdk, all-in-one header
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <armadillo>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


void setup_lidar();
void shutdown_lidar();
pcl::PointCloud<pcl::PointXYZ>::Ptr get_lidar_data();