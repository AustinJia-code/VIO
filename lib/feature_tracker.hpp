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


public:
    /**
     * Init
     */
    FeatureTracker (const std::string& calib_path = CALIB_PATH)
    {
    }

    /**
     * Initialize camera pose from EKF (call this once at startup)
     */
    void initialize_pose (const Eigen::Vector3d& pos,
                          const Eigen::Quaterniond& rot)
    {
    }

    /**
     * Returns ABSOLUTE camera pose in world frame
     * @warning optional
     */
    std::optional<Pose> get_pose (const CamData& l_cd, const CamData& r_cd)
    {
        return std::nullopt;
    }
};