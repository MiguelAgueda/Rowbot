#include <datasavem.hpp>

DataSave::DataSave()
{
    std::cout << "Data Saver Instantiated!" << std::endl;
}

void DataSave::open_file(const char* lidar_file, const char* imu_file, const char* control_file)
{
    // "/home/rowbot/Documents/Rowbot/rplidar/sdk/output/Linux/Release/ScanImgSave.csv"
    IMU_FILE.open(imu_file, std::ios::app);
    LIDAR_FILE.open(lidar_file, std::ios::app);
    CONTROL_FILE.open(control_file, std::ios::app);
}

void DataSave::push_to_queue(TimedIMUData data)
{
    IMUQ.push(data);
}

void DataSave::push_to_queue(TimedLidarData data)
{
    LidarQ.push(data);
}

void DataSave::push_to_queue(TimedControlData data)
{
    ControlQ.push(data);
}

inline bool DataSave::queues_empty()
{
    return (IMUQ.size() == 0 && LidarQ.size() == 0 && ControlQ.size() == 0);
}

void DataSave::start_imu_data_handler()
{
    while (running)
    {
        if (!IMUQ.empty())
        {
            TimedIMUData data = IMUQ.front();
            IMUQ.pop(); 

            IMU_FILE << data.t << "\n";
            IMU_FILE << data.data.format(CSVFormat);
        }
    }
}

void DataSave::start_lidar_data_handler()
{
    while (running)
    {
        if (!LidarQ.empty())
        {
            TimedLidarData data = LidarQ.front();
            LidarQ.pop(); 

            LIDAR_FILE << data.t << "\n";
            LIDAR_FILE << data.data.format(CSVFormat);
        }
    }
}

void DataSave::start_control_data_handler()
{
    while (running)
    {
        if (!ControlQ.empty())
        {
            TimedControlData data = ControlQ.front();
            ControlQ.pop(); 

            CONTROL_FILE << data.t << "\n";
            CONTROL_FILE << data.data.format(CSVFormat);
        }
    }
}

void DataSave::start_data_saver()
{
    std::thread imu_data_thread([this] { start_imu_data_handler(); });
    std::thread lidar_data_thread([this] { start_lidar_data_handler(); });
    std::thread control_data_thread([this] { start_control_data_handler(); });
}

void DataSave::stop_data_saver()
{
    while (!queues_empty());  // Wait until queues are empty.
    running = false;
    // imu_data_thread.join();
    // lidar_data_thread.join();
    // control_data_thread.join();
}

// 		CSV_FILE << "Scan," << counter << "\n";
// 		CSV_FILE << "Joystick X," << (int)MCU_PACKAGE.joystick_x << "\n";
// 		CSV_FILE << "Joystick Y," << (int)MCU_PACKAGE.joystick_y << "\n";
// 		CSV_FILE << "Theta";  // Following is the writing of each element in theta vector.
// 		for (uint i = 0; i < scan.theta_deg.size(); i++)
// 		{
// 			CSV_FILE << "," << scan.theta_deg[i];
// 		}
// 		CSV_FILE << "\n";  // End line for next entry.

// 	else if (PREV_SAVE_STATE)
// 	{
// 		printf("Turning Off Data Save.\n");
// 		CSV_FILE.close();
