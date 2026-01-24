/**
 * @file imu_listener.cpp
 * @brief Listens and prints raw IMU data
 */

#include <unistd.h>
#include "defs.h"
#include "imu.hpp"

int main ()
{
    IMU imu = {};

    while (true)
    {
        IMUData imu_raw = {};
        imu.read (imu_raw);

        std::cout << "Nanosecs: \n" << imu_raw.time_ns << "\n"
                  << "Accel: \n" << imu_raw.accel << "\n"
                  << "Gyro: \n" << imu_raw.gyro << "\n"
                  << std::endl;
        
        usleep (sec_to_us (sec_t {0.1}));
    }
}