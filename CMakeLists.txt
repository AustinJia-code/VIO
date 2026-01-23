cmake_minimum_required (VERSION 3.10)
project (VIO)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Find Packages
find_package (OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (Threads REQUIRED)

# 2. Create a "Common" dependency target
# This is an "INTERFACE" library because it has no .cpp files of its own
add_library(vio_core INTERFACE)

# 3. Attach all the includes/libraries to the core
target_include_directories(vio_core INTERFACE 
    include 
    lib 
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(vio_core INTERFACE 
    ${OpenCV_LIBS} 
    Eigen3::Eigen 
    Threads::Threads
)

# 4. Now your executables are super simple to define
add_executable (vio src/main.cpp)
target_link_libraries (vio PRIVATE vio_core)

add_executable (imu_listener tools/imu_listener.cpp)
target_link_libraries (imu_listener PRIVATE vio_core)

add_executable (stereo_calib tools/stereo_calib.cpp)
target_link_libraries (stereo_calib PRIVATE vio_core)