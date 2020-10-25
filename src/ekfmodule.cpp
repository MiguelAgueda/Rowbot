/* 
 * Implementation of an Error State Extended Kalman Filter. 
 * 
 * For more information about this algorithm,
 * see section 3.3 of Probabilistic Robotics (Thrun, Burgard, Fox).
 * More information about this book can be found at 
 * http://www.probabilistic-robotics.org/
 * 
 * This file is part of ROWBOT which is released under NO license.
 */

#include <ekfmodule.hpp>


/*
 * EKF Default Constructor.
 * 
 * Instantiates an ICP and IMU object for measurement updates.
 */
ES_EKF::ES_EKF()
{
    icp_ptr = new ICP();
    mpu_ptr = new IMU();
    std::cout << "Sleeping for 5 seconds.\n";
    // useconds_t useconds = 5000;
    sleep(5);

    std::cout << "EKF Constructed" << std::endl;
}


/* 
 * Perform correction to state using corrective measurement.
 * 
 * Parameters
 * ----------
 *      sensor_var : float
 *      Variance of sensor that collected the measurement.
 *      
 *      p_cov_check : Eigen::Matrix<float, 9, 9>
 *      Measurement covariance matrix.
 *  
 *      y_k : Eigen::Vector3f
 *      Measurement vector.
 * 
 *      p_check : Eigen::Vector3f
 *      Current "best guess" for position vector.
 * 
 *      v_check : Eigen::Vector3f
 *      Current "best guess" for velocity vector.
 * 
 *      q_check : Eigen::Quaternionf
 *      Current "best guess" for state quaternion.
 * 
 * Returns
 * -------
 *      corrected_state : ekf_update_return object.
 *      Object containing corrected state variables.
 */
ekf_update_return ES_EKF::measurement_update(float sensor_var,                       // Sensor variance.
                                             Eigen::Matrix<float, 9, 9> p_cov_check, // Covariance Estimate.
                                             Eigen::Vector3f y_k,                    // Measurement for update.
                                             Eigen::Vector3f p_check,                // Current position estimate.
                                             Eigen::Vector3f v_check,                // Current velocity estimate.
                                             Eigen::Quaternionf q_check             // Current pose estimate (Quaternion).
                                            )
{

    // Define return object [p_hat, v_hat, q_hat, p_cov_hat].
    ekf_update_return corrected_state;

    // 3.1 Compute Kalman Gain.
    Eigen::Matrix3f Q = sensor_var * I_3;                                    // Additive noise.
    Eigen::Matrix3f to_invert = h_jac * p_cov_check * h_jac.transpose() + Q; // Computation used directly below.
    Eigen::Matrix<float, 9, 3> K_gain = p_cov_check * h_jac.transpose() * to_invert.inverse();

    // 3.2 Compute Error State, 9D vector.
    Eigen::Vector<float, 9> error_state = K_gain * (y_k - p_check);

    // 3.3 Correct Predicted State.
    // 3.3.0 Collect corrections from error_state.
    Eigen::Vector3f es_p, es_v, es_q;
    es_p = error_state(Eigen::seq(0, 2));
    es_v = error_state(Eigen::seq(3, 5));
    es_q = error_state(Eigen::seq(6, 8));

    // 3.3.1 Apply error_state (es) corrections to state.
    corrected_state.p_hat = p_check + es_p;
    corrected_state.v_hat = v_check + es_v;
    // Create quaternion using error_state.
    Eigen::Matrix3f es_q_axis_angle;
    es_q_axis_angle = Eigen::AngleAxisf(es_q(0), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(es_q(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(es_q(2), Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf es_quaternion(es_q_axis_angle);
    corrected_state.q_hat = es_quaternion * q_check;

    // 3.4 Compute Corrected Covariance.
    corrected_state.p_cov_hat = (I_9 - (K_gain * h_jac)) * p_cov_check;

    return corrected_state;
}


/*
 * Start EKF update loop.
 */
void ES_EKF::start_updater()
{
    running = true;
    // Initialize start time placeholder.
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
    // Initialize last time stamp placeholder.
    std::chrono::steady_clock::time_point t_km = std::chrono::steady_clock::now();

    // Start threads for GNSS / IMU / ICP objects.
    std::thread icp_thread(&ICP::start_updater, icp_ptr, true, t_start);
    std::thread mpu_thread(&IMU::start_updater, mpu_ptr);

    // Fill in initializers for Jacobians.
    l_jac = Eigen::Matrix<float, 9, 6>::Zero();
    l_jac(Eigen::seq(3, 8), Eigen::all) = Eigen::Matrix<float, 6, 6>::Identity();
    h_jac = Eigen::Matrix<float, 3, 9>::Zero();
    h_jac(Eigen::all, Eigen::seq(0, 2)) = I_3;

    // Initialize position, velocity, and orientation states to zeros.
    Eigen::Vector3f p_km = Eigen::Vector3f::Zero();
    Eigen::Vector3f v_km = Eigen::Vector3f::Zero();
    Eigen::Quaternionf q_km(0, 0, 0, 0);
    // Initialize gravity vector.
    Eigen::Vector3f gravity;  
    // IF gravity is accounted for in IMU update:
    gravity = Eigen::Vector3f::Zero();  
    // ELSE:
    // gravity << 0, 0, -9.81;

    Eigen::Matrix<float, 9, 9> P_cov_km = Eigen::Matrix<float, 9, 9>::Zero();
    Eigen::Matrix<float, 9, 9> F = I_9;
    Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Identity();

    std::cout << "Starting EKF... \n";
    while (running)
    {
        if (mpu_ptr->updated)
        {
            /* 0.0
             * Compute necessary prerequisite data.
             * We need the change in time, delta_t, as well as the latest IMU data, imu_X.
             */
            // 0.1 Compute `delta_t` for current time step.
            std::chrono::steady_clock::time_point t_k = std::chrono::steady_clock::now();
            float delta_t = (t_k - t_km).count() * 1e-9; // Convert [ns] to [s].
            t_km = t_k;                                  // Update last time stamp.

            // 0.2 Compute time from program start, used for data logging.
            float t_from_start = (t_k - t_start).count() * 1e-9;

            // 0.3 GET IMU F, W;
            // Initialize containers for IMU data.
            Eigen::Matrix<float, 2, 4> IMU_X;
            Eigen::Vector3f imu_f;
            Eigen::Vector<float, 4> imu_quat;
            // Update IMU_X matrix with latest data.
            IMU_X = mpu_ptr->get_latest(true, t_from_start);
            // Separate matrix into vector components for specific force, quaternion parameters.
            imu_f = IMU_X(0, Eigen::seq(0, 2)); // Specific force, units of [g].
            imu_quat = IMU_X(1, Eigen::all);    // Quaternion params [w, x, y, z]

            /* 1
             * Update state with IMU update.
             */

            // Initialize variables for state update step.
            Eigen::Matrix<float, 9, 9> P_cov_est;
            Eigen::Vector3f p_est, v_est;
            Eigen::Quaternionf q_est;

            // Convert quaternion representation to rotation matrix.
            Eigen::Matrix3f C_ns = q_km.normalized().toRotationMatrix();
            // Use rotation matrix, with imu_f, to update position and velocity.
            p_est = p_km + (delta_t * v_km) + ((std::pow(delta_t, 2) / 2) * ((C_ns * imu_f) + gravity));
            v_est = v_km + delta_t * ((C_ns * imu_f) + gravity);

            // Compute quaternion from IMU.
            Eigen::Quaternionf q_imu(imu_quat(0), imu_quat(1), imu_quat(2), imu_quat(3));
            // Update state quaternion with quaternion computed using imu_w.
            q_est = q_km * q_imu;

            // 1.1 Linearize the motion model, compute Jacobians.
            Eigen::Vector3f to_skew = C_ns * imu_f;
            Eigen::Matrix3f F_mid_right = -skew_symmetric(to_skew) * delta_t;

            F(Eigen::seq(0, 2), Eigen::seq(3, 5)) = I_3 * delta_t;
            F(Eigen::seq(3, 5), Eigen::seq(6, 8)) = F_mid_right;

            Q(Eigen::seq(0, 2), Eigen::seq(0, 2)) = var_imu_f * std::pow(delta_t, 2) * I_3;
            Q(Eigen::seq(3, 5), Eigen::seq(3, 5)) = var_imu_w * std::pow(delta_t, 2) * I_3;

            /* 2
             * Propagate Uncertainty.
             */

            P_cov_est = (F * P_cov_km * F.transpose()) + (l_jac * Q * l_jac.transpose());

            /* 3
             * Check for availability of corrective measurements (LiDAR, GNSS).
             */

            if (icp_ptr->updated)
            {
                // Perform update step with LiDAR data.
                ekf_update_return corrected_state;
                Eigen::Vector3f y_lidar = icp_ptr->get_latest_y();
                std::cout << "LiDAR Estimate: \n"
                          << y_lidar << std::endl;

                corrected_state = measurement_update(var_lidar, P_cov_est, y_lidar, p_est, v_est, q_est);
                p_est = corrected_state.p_hat;
                v_est = corrected_state.v_hat;
                q_est = corrected_state.q_hat;
                P_cov_est = corrected_state.p_cov_hat;
            }

            /* 4
             * Update state variables (encompassing scope) with estimates from this time step.
             */

            p_km = p_est;
            v_km = v_est;
            q_km = q_est;
            P_cov_km = P_cov_est;

            std::cout << "P_km \n"
                      << p_km << "\n\n";
        }  // End "if (mpu_ptr->updated)" block.
    }  // End "while (running)" block.

    // When running is set to false,
    icp_ptr->stop_updater();
    icp_thread.join();

    mpu_ptr->stop_updater();
    mpu_thread.join();

} // End "start_ekf()" block.


/*
 * Stop updater loop.
 */
void ES_EKF::stop_updater()
{
    running = false;
}


/*
 * Compute skew-symmetric matrix from input vector, v.
 * 
 * Parameters
 * ----------
 *      v : Eigen::Vector3f
 *          Vector used to construct skew-symmetric matrix.
 * 
 * Returns
 * -------
 *      skewed : Eigen::Matrix3f
 *          Skew-symmetric matrix of input vector, v.
 */
Eigen::Matrix3f ES_EKF::skew_symmetric(Eigen::Vector3f v)
{

    Eigen::Matrix3f skewed;
    skewed << 0, -v[2], v[1],
        v[2], 0, -v[0],
        -v[1], v[0], 0;

    return skewed;
}