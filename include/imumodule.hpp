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
    arma::fmat imu_vals = {{0, 0}, {0, 0}, {0, 0}};
    void update_imu();
    bool setup_imu();
    bool running = false;

public:
    IMU();
    void start_updater();
    void stop_updater();
    arma::fmat get_latest();
    bool updated = false;
};