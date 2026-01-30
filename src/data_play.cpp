/**
 * @file data_play.cpp
 * @brief Dataset runner for VIO
 */

#include "visual_odometry.hpp"
#include "euroc_player.hpp"
#include "ekf.hpp"
#include "streamer.hpp"
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
        current_ts = l.time_ns;

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
    Streamer streamer;
    streamer.send_reset ();
    VisualOdometry vo ("../data/calib/euroc_calib.yaml",
                       "../data/euroc_datasets/machine_hall/MH_01_easy/mav0/cam0/sensor.yaml",
                       "../data/euroc_datasets/machine_hall/MH_01_easy/mav0/cam1/sensor.yaml");

    while (auto frames = player.get_next_stereo ())
    {
        auto [l, r] = *frames;
        ns_t current_ts {l.time_ns};
        auto imu_batch = player.get_imu_until (current_ts);

        // EKF
        for (const IMUData& imu : imu_batch)
            ekf.predict (imu);

        // Process Vision 
        // TODO: Handle camera offset from IMU?
        Pose cam_pose = vo.process_frame(l.img, r.img, l.time_ns);
        Pose global_pose = vo.get_global_pose();

        // Transform from camera frame to body frame using T_BS_cam0
        cv::Mat T_BS = vo.get_calibration ().T_BS_cam0;  // You loaded this!

        // Extract rotation and translation from T_BS
        Eigen::Matrix3d R_BS;
        Eigen::Vector3d t_BS;
        for (int i = 0; i < 3; i++) {
            t_BS(i) = T_BS.at<double>(i, 3);
            // for (int j = 0; j < 3; j++) {
            //     R_BS(i, j) = T_BS.at<double>(i, j);
            // }
        }

        R_BS << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
                
        // Transform VO pose to body frame
        Pose body_pose;
        body_pose.pos = R_BS * global_pose.pos + t_BS + init_pos;
        body_pose.rot = init_rot * Eigen::Quaterniond (R_BS) * global_pose.rot;
        body_pose.time_ns = global_pose.time_ns;

        // global_pose.pos[0] *= -1;
        // global_pose.pos[1] *= -1;

        // Now body_pose should match ground truth frame
        ekf.update (body_pose);

        Pose fused_state = ekf.get_estimate ();
        Eigen::Vector3d gt_pos = player.get_gt (current_ts);

        double drift = (fused_state.pos - gt_pos).norm ();

        // Telemetry
        TelemetryPacket pkt = {body_pose.pos.x (),
                               body_pose.pos.y (),
                               body_pose.pos.z (),
                               gt_pos.x (),
                               gt_pos.y (),
                               gt_pos.z (),
                               current_ts};
        streamer.send (pkt);
        
        std::cout << "TS: " << ns_to_sec (current_ts - init_time)
                  << " | EKF Pos: " << fused_state.pos.transpose () 
                  << " | GT: " << gt_pos.transpose () 
                  << " | Drift: " << drift << "m" << std::endl;

        // usleep (sec_to_us (sec_t {0.05}));
    }

    std::cout << "Dataset Processing Complete." << std::endl;
    return 0;
}