cmake_minimum_required (VERSION 3.10)
project (VIO)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Packages
find_package (OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (Threads REQUIRED)

# Common dependency target
add_library (vio_core INTERFACE)

# Attach all the includes/libraries to the core
target_include_directories (vio_core INTERFACE 
    include 
    lib 
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries (vio_core INTERFACE 
    ${OpenCV_LIBS} 
    Eigen3::Eigen 
    Threads::Threads
)

# Executables
add_executable (live_play src/live_play.cpp)
target_link_libraries (live_play PRIVATE vio_core)

add_executable (data_play src/data_play.cpp)
target_link_libraries (data_play PRIVATE vio_core)

add_executable (imu_listener tools/imu_listener.cpp)
target_link_libraries (imu_listener PRIVATE vio_core)

add_executable (camera_listener tools/camera_listener.cpp)
target_link_libraries (camera_listener PRIVATE vio_core)

add_executable (stereo_calib tools/stereo_calib.cpp)
target_link_libraries (stereo_calib PRIVATE vio_core)

add_executable (stereo_capture tools/stereo_capture.cpp)
target_link_libraries (stereo_capture PRIVATE vio_core)

add_executable (kalman_tests tests/kalman_tests.cpp)
target_link_libraries (kalman_tests PRIVATE vio_core)