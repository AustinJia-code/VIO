/**
 * @file live_play.cpp
 * @brief Live runner for VIO on real hardware
 */

#include <iostream>

#include "imu.hpp"
#include "stereo.hpp"
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
    Stereo cam;
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
        /** IMU READ AND PREDICT **/
        auto imu_opt = imu.read ();
        if (imu_opt)
            ekf.predict (*imu_opt);

        /** CAMERA READ AND PREDICT **/
        auto frames_opt = cam.read ();
        if (!frames_opt)
        {
            usleep (sec_to_us (sec_t {0.001}));
            continue;
        }

        auto cam_est_opt = matcher.match (frames_opt->first,
                                          frames_opt->second);
        if (!cam_est_opt)
        {
            usleep (sec_to_us (sec_t {0.001}));
            continue;
        }
        
        ekf.update (*cam_est_opt);
        
        /** GET ESTIMATE **/
        Pose fused_estimate = ekf.get_estimate ();
        std::cout << "=== POSE ESTIMATE === \n"
                  << "Time (Sec): " << ns_to_sec (fused_estimate.time_ns) << "\n" 
                  << "[X, Y, Z]: " << fused_estimate.pos << "\n"
                  << "[Roll, Pitch, Yaw]: " << quat_to_euler (fused_estimate.rot)
                  << std::endl;
    }

    return EXIT_SUCCESS;
}