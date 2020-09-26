#include <armadillo>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "I2Cdev.h"
#include "MPU6050.h"


class IMU
{
private:
    std::mutex thread_guard;
    // static const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_2G;
    // static const MPUIMU::Gscale_t GSCALE = MPUIMU::GFS_250DPS;
    MPU6050 mpu;
    Eigen::Matrix<float, 2, 3> imu_vals = Eigen::Matrix<float, 2, 3>::Zero();
    void update_imu();
    bool setup_imu();
    bool running = false;

public:
    IMU();
    void start_updater();
    void stop_updater();
    Eigen::Matrix<float, 2, 3> get_latest();
    bool updated = false;
};