#include <common.hpp>
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
    Eigen::Matrix<float, 2, 3> IMU_VALS = Eigen::Matrix<float, 2, 3>::Zero();
    Eigen::Matrix<float, 2, 3> IMU_OFFSET = Eigen::Matrix<float, 2, 3>::Zero();
    bool setup_imu();
    void update_imu();
    void calibrate_imu(int);
    bool running = false;

public:
    IMU();
    void start_updater();
    void stop_updater();
    Eigen::Matrix<float, 2, 3> get_latest();
    bool updated = false;
};