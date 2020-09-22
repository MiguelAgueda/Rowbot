#include <armadillo>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <lidarmodule.hpp>


class ICP
{
    private:
        std::mutex thread_guard;
        arma::fvec icp_y = {0,0,0};

        bool running = false;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        void update_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_km, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k);
    
    public:
        ICP();
        void start_updater();
        void stop_updater();
        arma::fvec get_latest_y();
        bool updated = false;
};
