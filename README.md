Fork of Slamtec RPLIDAR Public SDK for C++
==================================

Introduction
------------
This fork may include some helpful information for those confused by the official SDK documentation.
I have updated the section on running the built program. 

Quick Start
-----------

### On Windows

If you have Microsoft Visual Studio installed, just open sdk/workspaces/vc14/sdk_and_demo.sln, and build the soultion.

After building, the executables can be found in the `rplidar_sdk\sdk\output\win32\Debug\` folder. 


Demo Applications
-----------------

To execute the application, we will use Powershell.

Before executing anything, navigate to the directory containing the executables.

Then, you can execute the program using the following,

### ultra_simple

This demo application simply connects to an RPLIDAR device and outputs the scan data to the console.
This is where the official SDK documentation lost me. 

    ultra_simple <serial_port_device>  # This isn't helpful.

The above command will result in an error via Windows Powershell.

To execute the program, do this instead:

    .\ultra_simple COM_NUMBER  # Use this instead.

If your RPLidar is connected to COM8, your command should be:

    .\ultra_simple COM8
