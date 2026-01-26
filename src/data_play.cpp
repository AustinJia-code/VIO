/**
 * @file data_play.cpp
 * @brief Dataset runner for VIO
 */

#include "feature_tracker.hpp"
#include "euroc_player.hpp"
#include "ekf.hpp"
#include "helpers.h"
#include <iostream>

int main ()
{
    std::cout << "Starting Full VIO Pipeline on Dataset..." << std::endl;

    // TODO: Fix data leakage
    /*** INIT ***/
    std::string data_path = "../data/euroc_datasets/machine_hall/MH_01_easy/mav0";
    EurocPlayer player (data_path);
    
    // Get biases off first few frames (assume at rest)
    player.load_ground_truth (data_path);
    auto frames = player.get_next_stereo ();
    auto [l, r] = *frames;
    ns_t current_ts = l.time_ns;
    ns_t init_time = l.time_ns;
    Eigen::Vector3d init_pos = player.get_gt (current_ts);

    std::vector<IMUData> imu_batch = {};

    // Magic number from # still frames * 4 (about 4 imu samples per image)
    while (imu_batch.size () < 1000)
    {
        auto frames = player.get_next_stereo ();
        auto [l, r] = *frames;
        uint64_t current_ts = l.time_ns;

        auto new_batch = player.get_imu_until (current_ts);
        imu_batch.insert (imu_batch.end (),
                          new_batch.begin (), new_batch.end ());
    }

    Eigen::Vector3d sum_accel = Eigen::Vector3d::Zero ();
    Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero ();
    for (const auto& imu : imu_batch)
    {
        sum_accel += imu.accel;
        sum_gyro += imu.gyro;
    }

    Eigen::Vector3d avg_accel = sum_accel / (double) imu_batch.size ();
    Eigen::Vector3d avg_gyro = sum_gyro / (double) imu_batch.size ();

    // Calculate initial orientation (Align IMU X-up to World Z-up)
    Eigen::Quaterniond init_rot = Eigen::Quaterniond::FromTwoVectors
    (
        avg_accel, Eigen::Vector3d (0, 0, avg_accel.norm ())
    );

    // Initial Accel Bias: The difference between measured magnitude and gravity
    Eigen::Vector3d initial_accel_bias = Eigen::Vector3d::Zero (); 

    EKF ekf;
    ekf.set_state (init_pos, Eigen::Vector3d::Zero (), init_rot);
    ekf.set_bias (initial_accel_bias, avg_gyro);

    /*** LOOP ***/
    FeatureTracker feature_tracker ("../data/calib/euroc_calib.yaml");
    while (auto frames = player.get_next_stereo ())
    {
        auto [l, r] = *frames;
        uint64_t current_ts = l.time_ns;
        auto imu_batch = player.get_imu_until (current_ts);

        for (const IMUData& imu : imu_batch)
            ekf.predict(imu);

        // Process Vision
        // auto cam_pose = feature_tracker.get_pose (l, r);
        // if (cam_pose)
        //     ekf.update (*cam_pose);

        // Compare with Ground Truth
        Pose fused_state = ekf.get_estimate ();
        Eigen::Vector3d gt_pos = player.get_gt (current_ts);

        double drift = (fused_state.pos - gt_pos).norm ();

        std::cout << "TS: " << ns_to_sec (current_ts - init_time)
                  << " | EKF Pos: " << fused_state.pos.transpose () 
                  << " | GT: " << gt_pos.transpose () 
                  << " | Drift: " << drift << "m" << std::endl;
    }

    std::cout << "Dataset Processing Complete." << std::endl;
    return 0;
}