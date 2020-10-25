#include <common.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"


class IMU
{
private:
    std::mutex thread_guard;
    Eigen::Matrix<float, 2, 4> IMU_VALS = Eigen::Matrix<float, 2, 4>::Zero();
    Eigen::Vector3f IMU_RAW_OFFSET = Eigen::Vector3f::Zero();
    void calibrate_imu(int);
    bool setup_imu();
    void update_imu();
    bool update_raw_imu();
    bool running = false;
    std::fstream IMU_FILE;  // For saving IMU data.
    std::chrono::steady_clock::time_point t_start;

    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

public:
    IMU();
    void start_updater();
    void stop_updater();
    Eigen::Matrix<float, 2, 4> get_latest(bool, float);
    bool updated = false;
};