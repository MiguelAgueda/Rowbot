cmake_minimum_required(VERSION 2.8.3)

project(main)

### Find PointCloudLibrary.
find_package(PCL 1.2 REQUIRED)

### Define path to downloaded libraries.
set(DL_LIBS "/home/rowbot/Downloads")

### Define RPLiDAR sdk.
set(RPLIDAR_SDK_PATH "./sdk")


### Define RPLiDAR sdk source files for later compilation.
FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

include_directories(
  include
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${RPLIDAR_SDK_PATH}/src/
  ${PCL_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
# link_directories(/home/rowbot/Documents/rowbot_cpp/sdk/sdk/src)

add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES "src/*.cpp")

add_executable (main ${SOURCES} ${RPLIDAR_SDK_SRC})
target_link_libraries (main ${PCL_LIBRARIES} armadillo m)
