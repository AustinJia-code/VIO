/**
 * @file data_play.cpp
 * @brief Dataset runner for VIO
 */

#include "matcher.hpp"
#include "euroc_player.hpp"
#include "ekf.hpp"
#include "helpers.h"
#include <iostream>

int main ()
{
    std::string data_path = "../data/euroc_datasets/machine_hall/MH_01_easy/mav0";
    
    Matcher matcher ("../data/calib/euroc_calib.yaml");
    EurocPlayer player (data_path);
    EKF ekf;

    ns_t init_ts;

    player.load_ground_truth (data_path);

    std::cout << "Starting Full VIO Pipeline on Dataset..." << std::endl;

    bool initialized = false;

    // Loop until out of data
    while (auto frames = player.get_next_stereo ())
    {
        auto [l, r] = *frames;
        uint64_t current_ts = l.time_ns;
        auto imu_batch = player.get_imu_until (current_ts);

        // First frame init
        if (!initialized)
        {
            if (imu_batch.empty ())
                continue;

            // Get starting position from Ground Truth
            Eigen::Vector3d start_pos = player.get_gt (current_ts);
            init_ts = current_ts;
            
            // Align rotation with gravity (cancel out tilt)
            Eigen::Vector3d avg_accel = Eigen::Vector3d::Zero ();
            for (const auto& imu : imu_batch)
                avg_accel += imu.accel;
            avg_accel /= imu_batch.size ();

            // Rotate the IMU's 'up' to the world's 'up'
            Eigen::Quaterniond init_rot = Eigen::Quaterniond::FromTwoVectors
            (
                avg_accel, Eigen::Vector3d (0, 0, G)
            );

            ekf.set_state (start_pos, Eigen::Vector3d::Zero (), init_rot);
            
            initialized = true;
            std::cout << "EKF Initialized! Pos: " << start_pos.transpose() 
                      << " | Tilt Aligned." << std::endl;
            continue;
        }

        for (const IMUData& imu : imu_batch)
            ekf.predict(imu);

        // Process Vision
        // auto delta_pose = matcher.match (l, r);
        
        // if (delta_pose)
        //     ekf.update(*delta_pose);

        // Compare with Ground Truth
        Pose fused_state = ekf.get_estimate ();
        Eigen::Vector3d gt_pos = player.get_gt (current_ts);

        double drift = (fused_state.pos - gt_pos).norm ();

        std::cout << "TS: " << std::fixed  << std::setprecision (5) 
                            << ns_to_sec (current_ts - init_ts) 
                  << " | EKF Pos: " << fused_state.pos.transpose () 
                  << " | GT: " << gt_pos.transpose () 
                  << " | Drift: " << drift << "m" << std::endl;
    }

    std::cout << "Dataset Processing Complete." << std::endl;
    return 0;
}