#include <icpmodule.hpp>


ICP::ICP(void)
{
    // Set the max correspondence distance to 25 cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.25);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-3);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);
    std::cout << "ICP Instantiated" << std::endl;
}

void ICP::update_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_km, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k)
{
    icp.setInputSource(cloud_km);  // Update source cloud to last cloud.
    icp.setInputTarget(cloud_k);  // Update destination cloud to current cloud.

    pcl::PointCloud<pcl::PointXYZ> Final;  // Create cloud to store result of icp alignment.
    icp.align(Final);  // Align the point clouds defined above.
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 R_p;  // Create Eigen matrix to hold results.
    R_p = icp.getFinalTransformation();  // Obtain results from ICP.
    arma::fmat R_arma(R_p.data(), R_p.rows(), R_p.cols(), false, false);  // Convert Eigen to Armadillo mat.
    std::cout << "R_arma" << R_arma << std::endl;
    arma::fmat R = {{R_arma(0,0), R_arma(0,1), R_arma(0,2)},
                    {R_arma(1,0), R_arma(1,1), R_arma(1,2)},
                    {R_arma(2,0), R_arma(2,1), R_arma(2,2)}};
    arma::fvec t = {R_arma(0,3), R_arma(1,3), R_arma(2,3)};

    // R.print();
    std::cout << "R " << R << std::endl;
    // t.print();
    std::cout << "t " << t << std::endl;
    std::lock_guard<std::mutex> lk(thread_guard);  // Lock thread access before writing to shared data.
    icp_y = R.t() * (icp_y - t);
    updated = true;
}

void ICP::start_updater()
{
    running = true;
    setup_lidar();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k, cloud_km;
    cloud_km = get_lidar_data();
    while(running)
    {
        cloud_k = get_lidar_data();
        // perform ICP.
        update_icp(cloud_km, cloud_k);
        // Set last cloud to current cloud. This is done after all computation involving last cloud.
        *cloud_km = *cloud_k;
    }
}

void ICP::stop_updater()
{
    running = false;
    shutdown_lidar();
}

arma::fvec ICP::get_latest_y()
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
