#include <common.hpp>
#include <fstream>
#include <queue>


struct TimedIMUData
{
    float t;
    Eigen::Matrix<float, 2, 3> data;
};

struct TimedLidarData
{
    float t;
    Eigen::Matrix<float, 2, Eigen::Dynamic> data;
};

struct TimedControlData
{
    float t;
    Eigen::Vector3f u;
};

class DataSave
{
public:
    DataSave();
    void open_file(const char*, const char*, const char*);
    void push_to_queue(TimedIMUData);
    void push_to_queue(TimedLidarData);
    void push_to_queue(TimedControlData);
    void start_data_saver();
    void stop_data_saver();

private:
    bool running;
    std::fstream IMU_FILE;  // For saving IMU data.
    std::fstream LIDAR_FILE;  // For saving scan data.
    std::fstream CONTROL_FILE;  // For saving control data.
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
    std::queue<TimedIMUData> IMUQ;
    std::queue<TimedLidarData> LidarQ;
    std::queue<TimedControlData> ControlQ;
    inline bool queues_empty();
    void start_imu_data_handler();
    void start_lidar_data_handler();
    void start_control_data_handler();
};