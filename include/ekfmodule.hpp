#include <armadillo>
#include <common.hpp>
#include <Eigen/Geometry> 
#include <chrono>
#include <cmath>

#include <icpmodule.hpp>
#include <imumodule.hpp>



struct ekf_update_return
{
    Eigen::Vector3f p_hat;
    Eigen::Vector3f v_hat;
    Eigen::Quaternionf q_hat;
    Eigen::Matrix<float, 9, 9> P_cov_hat;
};

class ES_EKF
{
private:
    // Define variables for sensor objects.
    IMU * mpu_ptr;
    ICP * icp_ptr;
    // Define sensor variances.
    const float var_imu_f = 1e-3;
    const float var_imu_w = 1e-3;
    const float var_gnss = 1e-2;
    const float var_lidar = 1e-3;

    // Define constants for later use.
    Eigen::Matrix<float, 9, 6> l_jac = Eigen::Matrix<float, 9, 6>::Zero();
    Eigen::Matrix<float, 3, 9> h_jac = Eigen::Matrix<float, 3, 9>::Zero();
    Eigen::Vector3f g; 
    g << 0, 0, -9.81;
    bool running = false;

public:
    ekf_update_return measurement_update(float sensor_var,                       // Sensor variance.
                                         Eigen::Matrix<float, 9, 9> P_cov_check, // Covariance Estimate.
                                         Eigen::Vector3f y_k,                    // Measurement for update.
                                         Eigen::Vector3f p_check,                // Current position estimate.
                                         Eigen::Vector3f v_check,                // Current velocity estimate.
                                         Eigen::Quaternionf q_check              // Current pose estimate (Quaternion).
                                         )

        void start_ekf();
};
