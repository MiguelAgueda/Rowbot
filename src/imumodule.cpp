#include <imumodule.hpp>


IMU::IMU(void)
{
    std::cout << "IMU Instantiated" << std::endl;
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
    if (mpu.getIntDataReadyStatus())  // check if data is ready via interrupt.
    {   
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        float Ax, Ay, Az, Gx, Gy, Gz;
        const float g_factor = 16384;
        Ax = ax / g_factor;
        Ay = ay / g_factor;
        Az = (az / g_factor) - 1.0;
        // Convert gyroscope measurements to [deg/s].
        const float dps_factor = 131;
        Gx = gx / dps_factor;
        Gy = gy / dps_factor;
        Gz = gz / dps_factor;

        // Lock thread before accessing shared data.
        std::lock_guard<std::mutex> lk(thread_guard);
        // Write values to `imu_vals`.
        imu_vals = {{Ax, Gx}, {Ay, Gy}, {Az, Gz}};
        // Inform outer loop that data has been updated.
        updated = true;
    }
}

void IMU::start_updater()
{
    running = true;
    bool imu_setup = setup_imu();
    if (!imu_setup)
        stop_updater();

    while (running)
    {
        update_imu();
    }
}

void IMU::stop_updater()
{
    running = false;
}

arma::fmat IMU::get_latest()
{
    std::lock_guard<std::mutex> lk(thread_guard);
    updated = false;
    return imu_vals;
}
