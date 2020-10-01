#include <common.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <lidarmodule.hpp>


class ICP
{
    private:
        std::mutex thread_guard;
        Eigen::Vector3f icp_y = Eigen::Vector3f::Zero();
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        void update_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_km, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k);
        bool running = false;
    
    public:
        ICP();
        void start_updater();
        void stop_updater();
        Eigen::Vector3f get_latest_y();
        bool updated = false;
};
