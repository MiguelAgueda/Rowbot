/*
 * Inertial Measurement Unit Implementation.
 * 
 * This specific implementation uses an MPU6050 IMU.
 * Gravity's acceleration is negated in this implementation.
 * This will cause the gravity vector, in the Kalman filter, to be a zero-vector.
 */

#include <imumodule.hpp>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
Quaternion QUAT;  // Quaternion container [w, x, y, z].
VectorInt16 ACCEL;  // Acceleration sensor measurements [x, y, z]. 

const float g_scalar = 9.81;

/*
 * IMU Class Constructor.
 * 
 * Sets up IMU and on-board digital motion processor. 
 * Also performs zero-offset calibration on IMU.
 */
IMU::IMU(void)
{
    std::cout << "IMU Instantiated" << std::endl;

    bool imu_setup = setup_imu();
    if (!imu_setup)
    {
        stop_updater();
        std::cout << "IMU Setup Not Successful\n";
    }

    calibrate_imu(150);  // Perform calibration with 150 measurements.

    const char *imu_file = "/home/rowbot/Documents/Rowbot/datasets/test/imu.csv";
    IMU_FILE.open(imu_file, std::fstream::out);
    t_start = std::chrono::steady_clock::now();
}

/*
 * Perform setup routine for MPU6050 IMU.
 * 
 * Initializes MPU6050 on-board digital motion processor.
 * 
 * Returns
 * -------
 *      bool : Status of IMU setup routine.
 *          True - Setup Successful.
 *          False - Setup Not Successful.
 */
bool IMU::setup_imu()
{
    mpu.initialize();
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    // Load and configure the DMP.
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // Make sure it worked, returned 0 if so.
    if (devStatus == 0) {
        // Turn on the DMP, now that it's ready.
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        // Flip DMP Ready flag to update global-scope.
        printf("DMP ready!\n");
        dmpReady = true;

        // Get expected DMP packet size for later comparison.
        packetSize = mpu.dmpGetFIFOPacketSize();
        return true;
    } else {
        // ERROR!
        // 1 = initial memory load failed.
        // 2 = DMP configuration updates failed.
        // (if it's going to break, usually the code will be 1).
        printf("DMP Initialization failed (code %d)\n", devStatus);
        return false;
    }
}

/*
 * Update global acceleration and quaternion containers with raw IMU data.
 * 
 * Returns
 * -------
 *      bool : Status of ACCEL and QUAT update.
 *          True - Globals updated with latest values.
 *          False - Globals not updated with latest values.
 */
bool IMU::update_raw_imu()
{
    // Get current FIFO count.
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {  // If FIFO count is greater than buffer size.
        // Reset, as to continue cleanly.
        mpu.resetFIFO();
        printf("FIFO overflow!\n");
        return false;  // Return false since ACCEL has not been updated.
    } 
    else if (fifoCount >= 42) 
    {
        // Read a packet from FIFO.
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&QUAT, fifoBuffer);
        mpu.dmpGetAccel(&ACCEL, fifoBuffer);
        return true;  // ACCEL has been updated.
    }
    else
    {
        return false;
    }
    
}

/*
 * Update global acceleration and quaternion containers with converted IMU data.
 * 
 * Converts raw IMU output to units of G, or gravitational force.
 * The effects of gravity are negated via calibration.
 * 
 * Assumptions
 * -----------
 *      The environment of operation is flat.
 *      The rover will remain flat on the ground, at all times,
 *          with all wheels touching the plane assumed during calibration.
 *      
 *      These assumptions have been made due to unreliable MPU6050 measurements.
 *      Results may vary.
 */
void IMU::update_imu()
{
    if (update_raw_imu())
    {
        const float g_factor = 8192 / g_scalar;  // Conversion from raw units to G.
        float Ax, Ay, Az;

        // std::chrono::steady_clock::time_point t_now = std::chrono::steady_clock::now();
        // float t_k = (t_now - t_start).count();

        Ax = (ACCEL.x - IMU_RAW_OFFSET(0)) / g_factor;
        Ay = (ACCEL.y - IMU_RAW_OFFSET(1)) / g_factor;
        Az = (ACCEL.z - IMU_RAW_OFFSET(2)) / g_factor;

        // Lock thread before accessing shared data.
        std::lock_guard<std::mutex> lk(thread_guard);
        IMU_VALS << Ax, Ay, Az, 0,
                    QUAT.w, QUAT.x, QUAT.y, QUAT.z;

        updated = true;
    }
}

/*
 * Perform zero-offset calibration of IMU.
 * 
 * This calibration negates the effects of gravity in the calibration frame.
 * 
 * If the experiences change in pitch or roll, the inverse effects of gravity will be 
 *      projected back onto the measurement.
 */
void IMU::calibrate_imu(int n)
{
    std::cout << "Calibrating IMU \n";
    Eigen::Vector3f temp_imu_offset = Eigen::Vector3f::Zero();
    int i = 0;
    while (i < n)  // For number of samples requested.
    {
        if (update_raw_imu())
        {
            i += 1;
            Eigen::Matrix<float, 2, 4> IMU_X = get_latest(false, 0.0);  // Get sample.
            // temp_imu_offset += IMU_X(0, Eigen::seq(0,2));  // Add sample to offset counter.
            temp_imu_offset += Eigen::Vector3f(ACCEL.x, ACCEL.y, ACCEL.z);  // Add sample to offset counter.
            std::cout << i << ": ";
            std::cout << "temp_imu_offset: \n" << temp_imu_offset << "\n";

        }       
    }

    temp_imu_offset /= i;  // Compute average offset value.
    // temp_imu_offset(0, Eigen::seq(0,2)) -= g_vector;
    IMU_RAW_OFFSET = temp_imu_offset;
    std::cout << "Finished Calibrating IMU \n";
    std::cout << "IMU Offset Variables: \n" << IMU_RAW_OFFSET << "\n\n";
}

/*
 * Start IMU update loop.
 * 
 * This function is meant to be started concurrently from another control-loop.
 */
void IMU::start_updater()
{
    running = true;


    while (running)
    {
        update_imu();
        // usleep(100);
    }
}

/*
 * Stop IMU update loop via global flag.
 */
void IMU::stop_updater()
{
    IMU_FILE.close();
    running = false;
}

/*
 * Get latest IMU output from global containers.
 * 
 * IMU output may be in raw or converted form, depending upon update step carried out.
 * 
 * Returns
 * -------
 *      2x4 Eigen matrix of type float
 *          Cells [0, 0:2] contain acceleration [x y z].
 *          Cells [1, :] contain quaternion parameters [w x y z].
 */
Eigen::Matrix<float, 2, 4> IMU::get_latest(bool logging, float t_from_start)
{
    if (logging)
    {
        // const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, 0, ", ", ",", "[", "]", "\n");
        const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", "", "", ";\n", "", "\n");
        IMU_FILE << t_from_start << ";\n";
        IMU_FILE << IMU_VALS.format(CSVFormat);
    }

    // Lock thread before accessing shared data.
    std::lock_guard<std::mutex> lk(thread_guard);
    updated = false;
    return IMU_VALS;
}
