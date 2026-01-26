/**
 * @file feature_tracker.hpp
 * @brief Handles feature tracking and visual odometry
 */

#pragma once

#include "defs.h"
#include "consts.h"
#include <unistd.h>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <optional>

/**
 * Sparse feature mapper for depth
 */
class FeatureTracker
{
private:
    Pose camera_world_pose;
    cv::Mat K1, D1, Q;

public:
    /**
     * Init
     */
    FeatureTracker (const std::string& calib_path = CALIB_PATH)
    {
        camera_world_pose = {};

        // Load Q matrix from calibration yml
        cv::FileStorage fs (calib_path, cv::FileStorage::READ);
        if (!fs.isOpened ())
            throw std::runtime_error ("Failed to open calibration file");

        fs["K1"] >> K1;
        fs["D1"] >> D1;
        fs["Q"] >> Q;
        fs.release ();
    }

    /**
     * Initialize camera pose from EKF (call this once at startup)
     */
    void set_pose (const Eigen::Vector3d& pos,
                          const Eigen::Quaterniond& rot)
    {
        camera_world_pose.pos = pos;
        camera_world_pose.rot = rot;
    }

    /**
     * Returns ABSOLUTE camera pose in world frame
     * @warning optional
     */
    std::optional<Pose> get_pose (const CamData& l_cd, const CamData& r_cd)
    {
        // Find movement relative to previous captures
        // Return previous location + movement 
        return std::nullopt;
    }
};