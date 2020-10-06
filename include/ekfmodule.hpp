/*
 * Header file for `ekfmodule.cpp`.
 */
#include <common.hpp>

#include <chrono>
#include <cmath>

#include <icpmodule.hpp>
#include <imumodule.hpp>



struct ekf_update_return
{
    Eigen::Vector3f p_hat;
    Eigen::Vector3f v_hat;
    Eigen::Quaternionf q_hat;
    Eigen::Matrix<float, 9, 9> p_cov_hat;
};

class ES_EKF
{
public:
    ES_EKF();
    void start_updater();
    void stop_updater();

private:
    std::mutex thread_guard;
    // Define variables for sensor objects.
    IMU * mpu_ptr;
    ICP * icp_ptr;

    // Define sensor variances.
    const float var_imu_f = 1e-2;
    const float var_imu_w = 1e-2;
    const float var_gnss = 1e-2;
    const float var_lidar = 1e-6;

    // Define constants for later use.
    const Eigen::Matrix<float, 3, 3> I_3 = Eigen::Matrix<float, 3, 3>::Identity();
    const Eigen::Matrix<float, 9, 9> I_9 = Eigen::Matrix<float, 9, 9>::Identity();
    Eigen::Matrix<float, 9, 6> l_jac;
    Eigen::Matrix<float, 3, 9> h_jac;
    bool running = false;

    Eigen::Matrix3f skew_symmetric(Eigen::Vector3f v);

    ekf_update_return measurement_update(float sensor_var,                       // Sensor variance.
                                         Eigen::Matrix<float, 9, 9> p_cov_check, // Covariance Estimate.
                                         Eigen::Vector3f y_k,                    // Measurement for update.
                                         Eigen::Vector3f p_check,                // Current position estimate.
                                         Eigen::Vector3f v_check,                // Current velocity estimate.
                                         Eigen::Quaternionf q_check,              // Current pose estimate (Quaternion).
                                         );
};
