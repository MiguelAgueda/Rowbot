#include <imumodule.hpp>

const float g_scalar = 9.81;
const Eigen::Vector3f g_vector(0, 0, g_scalar);
// g_vector << 0, 0, g_scalar;

IMU::IMU(void)
{
    std::cout << "IMU Instantiated" << std::endl;

    bool imu_setup = setup_imu();
    if (!imu_setup)
    {
        stop_updater();
        std::cout << "IMU Setup Not Successful\n";
    }

    calibrate_imu(1);

}


bool IMU::setup_imu()
{
    mpu.initialize();
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    mpu.setIntDataReadyEnabled(true);
}

void IMU::update_imu()
{
    // If data ready bit set, all data registers have new data
    // std::cout << "Checking MPU Data Read Status\n";
    // const float g = 9.81;
    if (mpu.getIntDataReadyStatus())  // check if data is ready via interrupt.
    {   
        // std::cout << "MPU Was Ready!\n";
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        float Ax, Ay, Az, Gx, Gy, Gz;
        const float g_factor = 16384;  // Conversion from raw values to .
        Ax = ax * g_scalar / g_factor;  // Convert to [g].
        Ay = ay * g_scalar / g_factor;  // Convert to [g].
        Az = az * g_scalar / g_factor;  // Convert to [g].
        // Convert gyroscope measurements to [deg/s].
        const float dps_factor = 131;  // Conversion from raw value to [deg/s].
        const float d2r_factor = 3.1415926 / 180;  // Conversion from [deg/s] to [rad/s].
        Gx = (gx / dps_factor) * d2r_factor;  // Convert to [rad/s].
        Gy = (gy / dps_factor) * d2r_factor;  // Convert to [rad/s].
        Gz = (gz / dps_factor) * d2r_factor;  // Convert to [rad/s].

        // Lock thread before accessing shared data.
        std::lock_guard<std::mutex> lk(thread_guard);
        // Write values to `imu_vals`.
        IMU_VALS << Ax, Ay, Az,
                    Gx, Gy, Gz;
        
        IMU_VALS -= IMU_OFFSET;
        // Inform outer loop that data has been updated.
        updated = true;
    }
}

void IMU::calibrate_imu(int n)
{
    std::cout << "Calibrating IMU \n";
    Eigen::Matrix<float, 2, 3> temp_imu_offset = Eigen::Matrix<float, 2, 3>::Zero();

    for (int i = 0; i < n; i++)  // For number of samples requested.
    {
        update_imu();  // Update IMU.
        Eigen::Matrix<float, 2, 3> IMU_X = get_latest();  // Get sample.
        temp_imu_offset += IMU_X;  // Add sample to offset counter.
    }

    temp_imu_offset /= n;  // Compute average offset value.
    temp_imu_offset(0, Eigen::seq(0,2)) -= g_vector;
    IMU_OFFSET = temp_imu_offset;
    std::cout << "Finished Calibrating IMU \n";
    std::cout << "IMU Offset Variables: \n" << IMU_OFFSET << "\n\n";
}

void IMU::start_updater()
{
    running = true;

    while (running)
    {
        update_imu();
    }
}

void IMU::stop_updater()
{
    running = false;
}

Eigen::Matrix<float, 2, 3> IMU::get_latest()
{
    std::lock_guard<std::mutex> lk(thread_guard);
    updated = false;
    return IMU_VALS;
}
