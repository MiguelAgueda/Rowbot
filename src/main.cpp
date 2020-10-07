/*
 * Main control loop.
 * 
 * This module initializes an EKF object and starts its update loop.
 */
#include <common.hpp>
#include <signal.h>
#include <cmath>
#include <i2c_comm.h>
#include <time.h>

#include <ekfmodule.hpp>


bool ctrl_c_pressed;
bool PREV_AUTO_STATE;
bool PREV_SAVE_STATE;
I2C_IN_PACK MCU_PACKAGE;
I2C_OUT_PACK DRIVE_INSTR;


/* This method is called when the user exits the program via `ctrl-c`.

 * This will set a global variable, ctrl-c-pressed, to true, causing a 
 * break in the control loop. 
 */
void ctrlc(int)
{
	ctrl_c_pressed = true;
}


int main(int argc, const char *argv[])
{
	signal(SIGINT, ctrlc);
    ES_EKF * ekf_ptr = new ES_EKF();
    
    std::thread ekf_thread(&ES_EKF::start_updater, ekf_ptr);

    while (true)
    {
		if (ctrl_c_pressed)
		{
            std::cout << "Ctrl-C Pressed!\n\n";
            ekf_ptr -> stop_updater();
            ekf_thread.join();
		}
    }
}
