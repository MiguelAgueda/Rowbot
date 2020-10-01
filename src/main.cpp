#include <common.hpp>
#include <signal.h>
#include <fstream>
#include <cmath>
#include <bits/stdc++.h> 
#include <i2c_comm.h>
#include <time.h>

#include <ekfmodule.hpp>


bool ctrl_c_pressed;
bool PREV_AUTO_STATE;
bool PREV_SAVE_STATE;
I2C_IN_PACK MCU_PACKAGE;
I2C_OUT_PACK DRIVE_INSTR;

std::ofstream CSV_FILE; // For saving scan data.


void ctrlc(int)
{
	/* This method is called when the user exits the program via `ctrl-c`.
	 * This will set a global variable, ctrl-c-pressed, to true, causing a 
	 * break in the control loop. 
	 */
	ctrl_c_pressed = true;
}


void assign_virtual_joystick(float x, float y)
{
	/*
	Assigns global virtual joystick values calculated from steering angle.
	Parameters:
		steer_angle_deg - A float type containing angle in range [-90, 90].
	*/
	DRIVE_INSTR.virt_joystick_x = x * 127;
	DRIVE_INSTR.virt_joystick_y = y * 127;
}


int main(int argc, const char *argv[])
{
    // ICP icp;
    // std::thread icp_thread(icp.start_updater);
    // ICP * icp_ptr = new ICP();
    // IMU * imu_ptr = new IMU();
    // std::thread icp_thread(&ICP::start_updater, icp_ptr);
    // std::thread imu_thread(&IMU::start_updater, imu_ptr);
	signal(SIGINT, ctrlc);
    ES_EKF * ekf_ptr = new ES_EKF();
    
    std::thread ekf_thread(&ES_EKF::start_updater, ekf_ptr);

    while (true)
    {
        sleep(1);
    // std::cout << "i";
    //     arma::fvec y_curr;
    //     if (icp_ptr->updated)
    //     {
    //         std::cout << "Updated!" << std::endl;
    //         y_curr = icp_ptr->get_latest_y();
    //         y_curr.print();
    //     }

    //     arma::fmat imu_vals;
    //     if (imu_ptr->updated)
    //     {
    //         imu_vals = imu_ptr->get_latest();
    //         std::cout << "IMU: " << imu_vals << std::endl;
    //     }

		if (ctrl_c_pressed)
		{
            std::cout << "Ctrl-C Pressed!\n\n";
            ekf_ptr -> stop_updater();
            ekf_thread.join();
			// if (CSV_FILE.is_open())
			// {
			// 	CSV_FILE.close();
			// }
			// shutdown_lidar();
			// printf("Exiting Program.\n");
			// return 0;
		}
    }
}

	// connect_i2c();
	// // Debugging OpenCV linking issues. Should be version 4.3.0.
	// // printf("opencv version: %d.%d.%d\n",CV_VERSION_MAJOR,CV_VERSION_MINOR,CV_VERSION_REVISION);
	// cv::VideoCapture cap("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)128, height=(int)128, framerate=21/1, format=(string)NV12 ! nvvidconv flip-method=2 ! video/x-raw ! appsink ");
	// // Load keras model using frugally-deep.
	// // const auto model = fdeep::load_model("/home/rowbot/Documents/Rowbot/rplidar/sdk/app/ultra_simple/export_model.json");
	// int counter = 0;
	// time_t start_time = time(0);
	// std::cout << "Program Ready!\n";

	// while (true)
	// {
	// 	MCU_PACKAGE = rdwr_buffer(DRIVE_INSTR); // Read/write data from/to arduino.

	// 	if (MCU_PACKAGE.Shutdown)
	// 	{
	// 		printf("Shutdown Initiated.");
	// 		if (CSV_FILE.is_open())
	// 		{
	// 			CSV_FILE << "\nShutdown Initiated While Saving Scan Data";
	// 			CSV_FILE.close();
	// 		}
	// 		system("sudo shutdown now");
	// 	}

	// 	if (MCU_PACKAGE.DriveAutonomously)
	// 	{
	// 		if (!PREV_AUTO_STATE) // Need to setup for auto driving.
	// 		{
	// 			counter = 0;
	// 			PREV_AUTO_STATE = true;
	// 			setup_lidar();
	// 			// setup_camera(64, 64);  // Setup camera with parameters `height, width`.
	// 			printf("\nAutonomous Driving Initiated.\n");
	// 			start_time = time(0);
	// 		}
	// 		cv::Mat cam_image;
	// 		cap.read(cam_image);
	// 		cv::Mat cam_cropped = process_image(cam_image);

	// 		// Get scan as 2D image, run image through prediction model.
	// 		cv::Mat scan_image = get_model_input(64);
	// 		if (counter == 45)
	// 			cv::imwrite("/home/rowbot/Documents/Rowbot/rplidar/sdk/output/ScanImg.jpeg", scan_image);

	// 		const auto cam_input = fdeep::tensor_from_bytes(cam_cropped.ptr(),
	//         	static_cast<std::size_t>(cam_cropped.rows),
	//         	static_cast<std::size_t>(cam_cropped.cols),
	//         	static_cast<std::size_t>(cam_cropped.channels()),
	//         	0.0f, 1.0f);

	// 		const auto scan_input = fdeep::tensor_from_bytes(scan_image.ptr(),
	//         	static_cast<std::size_t>(scan_image.rows),
	//         	static_cast<std::size_t>(scan_image.cols),
	//         	static_cast<std::size_t>(scan_image.channels()),
	//         	0.0f, 1.0f);

	//     	const auto result = model.predict({scan_input, cam_input});
	// 		// std::cout << fdeep::show_tensors(result) << std::endl;
	// 		const auto x_y_joystick = *result[0].as_vector();
	// 		// Send drive instructions based on steering prediction.
	// 		assign_virtual_joystick(x_y_joystick[0], x_y_joystick[1]);

	// 		/* Below was used for a categorical prediction model. */
	// 		// const auto x_vector = *result[0].as_vector();
	// 		// const auto y_vector = *result[1].as_vector();

	// 		// const int max_x_idx = std::distance(x_vector.begin(),std::max_element(x_vector.begin(), x_vector.end()));
	// 		// const int max_y_idx = std::distance(y_vector.begin(),std::max_element(y_vector.begin(), y_vector.end()));
			
	// 		// assign_virtual_joystick(
	// 		// 	from_categorical(max_x_idx, x_vector.size()),
	// 		// 	from_categorical(max_y_idx, y_vector.size())
	// 		// 	);
	// 		counter += 1;  // Increment counter to keep track of data.
	// 	}
	// 	else if (PREV_AUTO_STATE) // Switched autonomous driving off.
	// 	{
	// 		printf("Turning Off Autonomous Driving.\n");
	// 		DRIVE_INSTR.virt_joystick_x = 0;
	// 		DRIVE_INSTR.virt_joystick_y = 0;
	// 		shutdown_lidar();
	// 		PREV_AUTO_STATE = false;
	// 		double total_time = difftime(time(0), start_time);
	// 		std::cout << "Program averaged: " << counter  << " cycles in " << total_time << " seconds\n";
	// 	}

	// 	if (MCU_PACKAGE.SaveScanData)
	// 	{
	// 		// int scan_counter;
	// 		if (!PREV_SAVE_STATE) // Setup for saving lidar data.
	// 		{
	// 			counter = 0;
	// 			printf("Data Save Initiated.");
	// 			// scan_counter = 0;
	// 			PREV_SAVE_STATE = true;
	// 			setup_lidar();
	// 			// setup_camera(64, 64);  // Setup camera with parameters `height, width`.
	// 			CSV_FILE.open("/home/rowbot/Documents/Rowbot/rplidar/sdk/output/Linux/Release/ScanImgSave.csv", std::ios::app);
	// 			start_time = time(0);
	// 		}
	// 		DataToSave scan = get_lidar_data();

	// 		cv::Mat img, cropped;
	// 		cap.read(img);
	// 		cropped = process_image(img);

	// 		if (img.empty())
	// 		{
	// 			std::cout << "Empty frame in main!";
	// 			img = cv::Mat::zeros(64, 64, CV_8U);
	// 		}

	// 		CSV_FILE << "Scan," << counter << "\n";
	// 		CSV_FILE << "Joystick X," << (int)MCU_PACKAGE.joystick_x << "\n";
	// 		CSV_FILE << "Joystick Y," << (int)MCU_PACKAGE.joystick_y << "\n";
	// 		CSV_FILE << "Theta";  // Following is the writing of each element in theta vector.
	// 		for (uint i = 0; i < scan.theta_deg.size(); i++)
	// 		{
	// 			CSV_FILE << "," << scan.theta_deg[i];
	// 		}
	// 		CSV_FILE << "\n";  // End line for next entry.

	// 		CSV_FILE << "Distance";  // Following is the writing of each element in the distance vector.
	// 		for (uint i = 0; i < scan.distance_mm.size(); i++)
	// 		{
	// 			CSV_FILE << "," << scan.distance_mm[i];
	// 		}
	// 		CSV_FILE << "\n";
			
	// 		CSV_FILE << "Image,";
	// 		CSV_FILE << cv::format(cropped, cv::Formatter::FMT_CSV) << "\n";

	// 		counter += 1;  // Increment `counter` to keep track of data.
	// 	}
	// 	else if (PREV_SAVE_STATE)
	// 	{
	// 		printf("Turning Off Data Save.\n");
	// 		CSV_FILE.close();
	// 		shutdown_lidar();
	// 		PREV_SAVE_STATE = false;
	// 		double total_time = difftime(time(0), start_time);
	// 		std::cout << "Program averaged: " << counter  << " cycles in " << total_time << " seconds\n";
	// 	}
	// }
// }
