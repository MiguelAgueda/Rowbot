#include <icpmodule.hpp>


ICP::ICP(void)
{
    // Set the max correspondence distance to 25 cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.25);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);

    setup_lidar();
    std::cout << "ICP Instantiated" << std::endl;
}

void ICP::update_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_km, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k)
{
    icp.setInputSource(cloud_km);  // Update source cloud to last cloud.
    icp.setInputTarget(cloud_k);  // Update destination cloud to current cloud.

    pcl::PointCloud<pcl::PointXYZ> Final;  // Create cloud to store result of icp alignment.
    icp.align(Final);  // Align the point clouds defined above.
    // pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 R_p;  // Create Eigen matrix to hold results.
    Eigen::Matrix<float, 4, 4> R_p;  // Create Eigen matrix to hold results.
    R_p = icp.getFinalTransformation();  // Obtain results from ICP.
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    R = R_p(Eigen::seq(0,2), Eigen::seq(0, 2));
    t = R_p(Eigen::seq(0,2), 3);
    // std::cout << "R " << R << std::endl;
    // std::cout << "t " << t << std::endl;
    std::lock_guard<std::mutex> lk(thread_guard);  // Lock thread access before writing to shared data.
    icp_y = R * (icp_y - t);
    updated = true;
}

void ICP::start_updater(bool logging, std::chrono::steady_clock::time_point t_start)
{
    running = true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k, cloud_km;
    std::chrono::steady_clock::time_point t_now = std::chrono::steady_clock::now();
    float t_k = (t_now - t_start).count() * 1e-9;
    cloud_km = get_lidar_data(logging, t_k);
    while(running)
    {
        t_now = std::chrono::steady_clock::now();
        t_k = (t_now - t_start).count() * 1e-9;
        cloud_k = get_lidar_data(logging, t_k);
        // perform ICP.
        update_icp(cloud_km, cloud_k);
        // Set last cloud to current cloud. This is done after all computation involving last cloud.
        *cloud_km = *cloud_k;
    }
}

void ICP::stop_updater()
{
    std::cout << "Stopping ICP Updater\n";
    running = false;
    shutdown_lidar();
}

Eigen::Vector3f ICP::get_latest_y()
{
    // Lock thread.
    std::lock_guard<std::mutex> lk(thread_guard);
    updated = false;
    return icp_y;
}



// int main (int argc, char** argv)
// {
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

//   // Fill in the CloudIn data
//   for (auto& point : *cloud_in)
//   {
//     point.x = 1024 * rand() / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand() / (RAND_MAX + 1.0f);
//     point.z = 1024 * rand() / (RAND_MAX + 1.0f);
//   }
  
//   std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
      
//   for (auto& point : *cloud_in)
//     std::cout << point << std::endl;
      
//   *cloud_out = *cloud_in;
  
//   std::cout << "size:" << cloud_out->size() << std::endl;
//   for (auto& point : *cloud_out)
//     point.x += 0.7f;

//   std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
      
//   for (auto& point : *cloud_out)
//     std::cout << point << std::endl;

//  return (0);
