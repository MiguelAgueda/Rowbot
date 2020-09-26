#include <ekfmodule.hpp>

// Define variable for 3x3 identity matrix.
const Eigen::Matrix3f I_3 = Eigen::Matrix<float, 3, 3>::Identity();
const Eigen::Matrix<float, 9, 9> I_9 = Eigen::Matrix<float, 9, 9>::Identity();

ES_EKF::ES_EKF()
{
/*
 * EKF Default Constructor.
 */

    icp_ptr = new ICP();
    imu_ptr = new IMU();
}

Eigen::Matrix3f ES_EKF::skew_symmetric(Eigen::Vector3f v)
{
/*
 * Compute skew-symmetric matrix from input vector v.
 * 
 * Parameters
 * ----------
 *      v : Eigen::Vector3f
 *      Vector used to construct skew-symmetric matrix.
 * 
 * Returns
 * -------
 *      skewed : Eigen::Matrix3f
 *      Skew-symmetric matrix of input vector, v.
 */

    Eigen::Matrix3f skewed;
    skewed << 0, -v[2], v[1],
              v[2], 0, -v[0],
              -v[1], v[0], 0;

    return skewed;
}

ekf_update_return ES_EKF::measurement_update(float sensor_var,                       // Sensor variance.
                                             Eigen::Matrix<float, 9, 9> P_cov_check, // Covariance Estimate.
                                             Eigen::Vector3f y_k,                    // Measurement for update.
                                             Eigen::Vector3f p_check,                // Current position estimate.
                                             Eigen::Vector3f v_check,                // Current velocity estimate.
                                             Eigen::Quaternionf q_check              // Current pose estimate (Quaternion).
)
{
    /* Perform correction to state using corrective measurement.
 * 
 * Parameters
 * ----------
 *      sensor_var : float
 *      Variance of sensor that collected the measurement.
 *      
 *      P_cov_check : Eigen::Matrix<float, 9, 9>
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

    // Define return struct.
    ekf_update_return corrected_state;

    // 3.1 Compute Kalman Gain.
    Eigen::Matrix3f R = I_3 * sensor_var;
    Eigen::Matrix3f to_invert = h_jac * P_cov_check * h_jac.t() + R;
    Eigen::Matrix<float, 9, 3> K_gain = P_cov_check * h_jac.t() * arma::inv(to_invert);

    // 3.2 Compute Error State. ES[0:3] contains p_correction, ES[3:6] contains v_correction.
    Eigen::Vector<float, 9> error_state = K_gain * (y_k - p_check);

    // 3.3 Correct Predicted State.
    // 3.3.0 Collect corrections from error_state.
    Eigen::Vector3f es_p, es_v, es_q;
    es_p = error_state(Eigen::seq(0, 2));
    es_v = error_state(Eigen::seq(3, 5));
    es_q = error_state(Eigen::seq(6, 8));
    // 3.3.1 Apply error_state (es) corrections to state.
    corrected_state.p_hat = corrected_state.p_hat + es_p;
    corrected_state.v_hat = corrected_state.v_hat + es_v;
    // Create quaternion using error_state.
    Eigen::Matrix3f es_q_axis_angle;
    es_q_axis_angle = Eigen::AngleAxisf(es_q(0), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(es_q(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(es_q(2), Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf es_quaternion(es_q_axis_angle);
    corrected_state.q_hat = es_quaternion * q_check;

    // 3.4 Compute Corrected Covariance.
    corrected_state.p_cov_hat = (I_9 - (K_gain * h_jac)) * corrected_state.p_cov_hat;

    return corrected_state;
}

void ES_EKF::start_ekf()
{
    running = true;

    // Start threads for GNSS / IMU / ICP objects.
    std::thread icp_thread(&ICP::start_updater, icp_ptr);
    std::thread imu_thread(&IMU::start_updater, imu_ptr);

    // Initialize state to zeros.
    Eigen::Vector3f p_km = Eigen::Vector3f::Zero();
    Eigen::Vector3f v_km = Eigen::Vector3f::Zero();

    Matrix3f euler_init;
    euler_init = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q_km(euler_init); // Convert Euler to Quaternion.

    Eigen::Matrix<float, 9, 9> P_cov_km = Eigen::Matrix<float, 9, 9>::Zero();

    Eigen::Matrix<float, 9, 9> F = Eigen::Matrix<float, 9, 9>::Identity();
    Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Identity();

    auto t_km = std::chrono::steady_clock::now();
    std::cout << "Starting EKF... \n";

    while (running)
    {
    if (imu_ptr->updated)
    {
        /* 0.0
         * Compute necessary prerequisite data.
         * We need the change in time, delta_t, as well as the latest IMU data, imu_X.
         */
        // 0.1 Compute `delta_t` for current time step.
        auto t_k = std::chrono::steady_clock::now();
        float delta_t = (t_k - t_km).count() * 1e-9; // Convert [ns] to [s].

        // 0.2 GET IMU F, W;
        // Initialize containers for IMU data.
        Eigen::Matrix<float, 2, 3> IMU_X;
        Eigen::Vector3f imu_f, imu_w;
        // Update IMU_X matrix with latest data.
        IMU_X = imu_ptr->get_latest();
        // Separate matrix into vector components for specific force, angular velocity.
        imu_f = IMU_X(0, Eigen::all); // Units of [g].
        imu_w = IMU_X(1, Eigen::all); // Units of [rad/s].

        /* 1
         * Update state with IMU update.
         */

        // Initialize variables for state update step.
        Eigen::Matrix<float, 9, 9> P_cov_est;
        Eigen::Vector3f p_est, v_est;
        Eigen::Quaternionf q_est;

        // Convert quaternion representation to rotation matrix.
        Eigen::Matrix3f C_ns = q_km.toRotationMatrix();
        // Use rotation matrix, with imu_f, to update position and velocity.
        p_est = p_km + (v_km * delta_t) + ((std::pow(delta_t, 2) / 2) * ((C_ns * g) + g));
        v_est = v_km + delta_t * ((C_ns * imu_f) + g);

        // Compute Euler angle from IMU.
        Eigen::Matrix3f euler;

        euler = Eigen::AngleAxisf(imu_w[0] * delta_t, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(imu_w[1] * delta_t, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(imu_w[2] * delta_t, Eigen::Vector3f::UnitZ());

        // Compute quaternion from IMU's euler.
        Eigen::Quaternionf q_imu(euler);
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
        if (lidar->updated)
        {
            // Perform update step with LiDAR data.
            ekf_update_return corrected_state;
            Eigen::Vector3f y_lidar = lidar->get_latest();
            corrected_state = measurement_update(var_lidar, P_cov_est, y_lidar, p_est, v_est, q_est);
            p_est = corrected_state.p_hat;
            v_est = corrected_state.v_hat;
            q_est = corrected_state.q_hat;
            P_cov_est = corrected_state.P_cov_hat;
        }

        // if (gnss->updated)
        // {
        //     // Perform update step with GNSS data.
        // }


        /* 4
         * Update state variables (encompassing scope) with estimates from this time step.
         */
        p_km = p_est;
        v_km = v_est;
        q_km = q_est;
        P_cov_km = P_cov_est;
    }  // End "if (imu_ptr->updated)" block.
    }  // End "while (running)" block.
}  // End "start_ekf()" block.