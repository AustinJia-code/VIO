/**
 * @file main.cpp
 * @brief Runner for VIO
 */

#include <iostream>

#include "imu.hpp"
#include "cam.hpp"
#include "matcher.hpp"
#include "ekf.hpp"
#include "helpers.h"

/**
 * Read from imu and cam, fuse imu integration and cam feature 
 * match estimations with EKF.
 */
int main ()
{
    // Init
    IMU imu;
    Cam cam;
    Matcher matcher;
    EKF ekf;

    IMUData imu_data;
    CamData lcam_data, rcam_data;
    Pose cam_pose;

    // Loop
    std::cout << "VIO Initialized, estimating..." << std::endl;
    ns_t last_imu_ns = get_time_ns ();

    while (true)
    {
        // Imu read and predict
        try
        {
            imu.read (imu_data);
            ekf.predict (imu_data);
        }
        catch (const std::exception& e)
        {
            // Pass
        }
        
        // Camera 
        cam.read (lcam_data, rcam_data);

        // If we dont get images, sleep to avoid spinning and cont
        if (lcam_data.dirty && rcam_data.dirty)
        {
            usleep (sec_to_us (sec_t {0.001}));
            continue;
        }
            
        // Matcher finds our motion between this frame and the last
        matcher.match (lcam_data, rcam_data, cam_pose);
        
        // If the Matcher successfully found a pose, then update ekf
        if (!cam_pose.dirty)
            continue;
        
        ekf.update (cam_pose);
        
        Pose estimate;
        ekf.get_estimate (estimate);
        std::cout << "=== POSE ESTIMATE === \n"
                  << "Time (Sec): " << ns_to_sec (estimate.time_ns) << "\n" 
                  << "[X, Y, Z]: " << estimate.pos << "\n"
                  << "[Roll, Pitch, Yaw]: " << quat_to_euler (estimate.rot)
                  << std::endl;
    }

    return EXIT_SUCCESS;
}