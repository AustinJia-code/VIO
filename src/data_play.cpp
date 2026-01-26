/**
 * @file data_play.cpp
 * @brief Dataset runner for VIO
 */

#include "matcher.hpp"
#include "euroc_player.hpp"
#include "ekf.hpp"
#include <iostream>

int main ()
{
    std::string data_path = "../data/euroc_datasets/machine_hall/MH_01_easy/mav0";
    
    Matcher matcher ("../data/calib/euroc_calib.yaml");
    EurocPlayer player (data_path);
    EKF ekf;

    player.load_ground_truth (data_path);

    std::cout << "Starting Full VIO Pipeline on Dataset..." << std::endl;

    while (auto frames = player.get_next_stereo ())
    {
        auto [l, r] = *frames;
        uint64_t current_ts = l.time_ns;

        // Get all IMU readings that happened between the last frame and this one
        auto imu_batch = player.get_imu_until (current_ts);
        for (const IMUData& imu : imu_batch)
            ekf.predict (imu);

        // Process Vision
        auto delta_pose = matcher.match (l, r);
        
        if (delta_pose)
            ekf.update(*delta_pose);

        // Compare with Ground Truth
        Pose fused_state = ekf.get_estimate ();
        Eigen::Vector3d gt_pos = player.get_gt (current_ts);
        double drift = (fused_state.pos - gt_pos).norm ();

        std::cout << "TS: " << current_ts 
                  << " | EKF Pos: " << fused_state.pos.transpose () 
                  << " | GT: " << gt_pos.transpose () 
                  << " | Drift: " << drift << "m" << std::endl;
    }

    std::cout << "Dataset Processing Complete." << std::endl;
    return 0;
}