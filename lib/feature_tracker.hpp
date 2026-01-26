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
#include <vector>
#include <optional>

/**
 * Keeps track of points temporally
 */
struct Feature
{
    cv::Point2f pt2d;
    cv::Point3f pt3d;
    cv::Mat desc;
};

/**
 * Sparse feature mapper for depth
 */
class FeatureTracker
{
private:
    bool debug;

    cv::Ptr<cv::ORB> detector;  // corner/feature detector
    cv::Ptr<cv::DescriptorMatcher> matcher;

    cv::Mat K1, D1, Q;

    std::vector<Feature> prev_fts;
    cv::Mat prev_descs;

    // Separate camera tracking b/c ransac returns translation
    Eigen::Vector3d cam_world_pos;
    Eigen::Quaterniond cam_world_rot;
    bool cam_initialized;

    /**
     * Internal helper to visualize tracking
     */
    void show_debug (const cv::Mat& img, 
                     const std::vector<cv::DMatch>& t_matches, 
                     const std::vector<cv::KeyPoint>& cur_kp)
    {
        cv::Mat out;
        cv::cvtColor (img, out, cv::COLOR_GRAY2BGR);

        for (const auto& m : t_matches)
        {
            // Draw a line from previous 2D position to current 2D position
            cv::Point2f pt_prev = prev_fts[m.queryIdx].pt2d;
            cv::Point2f pt_cur = cur_kp[m.trainIdx].pt;
            
            cv::line (out, pt_prev, pt_cur, cv::Scalar (0, 255, 0), 1);
            cv::circle (out, pt_cur, 2, cv::Scalar (0, 0, 255), -1);
        }

        std::string info = "Matches: " + std::to_string (t_matches.size ());
        cv::putText (out, info, cv::Point (30, 30), cv::FONT_HERSHEY_SIMPLEX,
                     0.8, cv::Scalar (255, 255, 0), 2);
        
        cv::imshow ("VIO Debugger", out);
        cv::waitKey (0); 
    }

    /**
     * Feature extraction and matching with previous descriptors
     * @attention Get a load of this function header :-(
     */
    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> 
    extract_and_stereo_match (const CamData& l, const CamData& r, 
                              cv::Mat& l_desc, 
                              std::vector<cv::DMatch>& matches)
    {
        std::vector<cv::KeyPoint> l_kp, r_kp;
        cv::Mat r_desc;
        
        detector->detectAndCompute (l.img, cv::noArray (), l_kp, l_desc);
        detector->detectAndCompute (r.img, cv::noArray (), r_kp, r_desc);

        if (!l_desc.empty () && !r_desc.empty ())
            matcher->match(l_desc, r_desc, matches);
        
        return {l_kp, r_kp};
    }

    /**
     * Depth on strong features
     */
    std::vector<Feature> triangulate (const std::vector<cv::DMatch>& matches,
                                      const std::vector<cv::KeyPoint>& l_kp,
                                      const std::vector<cv::KeyPoint>& r_kp,
                                      const cv::Mat& l_desc)
    {
        std::vector<Feature> features;
        for (const auto& m : matches)
        {
            float u = l_kp[m.queryIdx].pt.x;
            float v = l_kp[m.queryIdx].pt.y;
            float disparity = u - r_kp[m.trainIdx].pt.x;

            if (std::abs (l_kp[m.queryIdx].pt.y -
                          r_kp[m.trainIdx].pt.y)> Y_TOL_PX ||
                disparity < 0.1f)
                continue;

            cv::Mat pos = Q * (cv::Mat_<double> (4,1) << u, v, disparity, 1.0);
            double w = pos.at<double> (3);

            Feature f;
            f.pt2d = l_kp[m.queryIdx].pt;
            f.pt3d = cv::Point3f (pos.at<double> (0) / w,
                                  pos.at<double> (1) / w,
                                  pos.at<double> (2) / w);
            f.desc = l_desc.row (m.queryIdx);
            features.push_back (std::move (f));
        }

        return features;
    }

    /**
     * Estimate movement between frames
     */
    std::optional<Pose> estimate_motion (const std::vector<cv::KeyPoint>& l_kp,
                                         const cv::Mat& l_desc,
                                         const cv::Mat& l_img)
    {
        if (prev_fts.empty ())
            return std::nullopt;

        std::vector<cv::DMatch> t_matches;
        matcher->match (prev_descs, l_desc, t_matches);

        if (debug)
            show_debug (l_img, t_matches, l_kp);

        std::vector<cv::Point3f> pts3d;
        std::vector<cv::Point2f> pts2d;

        // Find how camera fits into previous 3D world with current 2D view
        for (const auto& m : t_matches)
        {
            pts3d.push_back (prev_fts[m.queryIdx].pt3d);
            pts2d.push_back (l_kp[m.trainIdx].pt);
        }

        if (pts3d.size() < MIN_MATCHES)
            return std::nullopt;

        cv::Mat rvec, tvec;
        // Superfunction to find perspective based on 3d and 2d points,
        // as well as do majority vote. Wow.
        if (cv::solvePnPRansac (pts3d, pts2d, K1, D1, rvec, tvec))
        {
            cv::Mat R_mat;
            cv::Rodrigues (rvec, R_mat);
            Eigen::Matrix3d R_eigen;
            cv::cv2eigen (R_mat, R_eigen);

            Pose p;
            Eigen::Vector3d t_eigen;
            cv::cv2eigen (tvec, t_eigen);
            
            p.pos = -R_eigen.transpose () * t_eigen;
            
            // The rotation from previous frame to current frame
            p.rot = Eigen::Quaterniond (R_eigen.transpose ()); 
            
            return p;
        }

        return std::nullopt;
    }

public:
    /**
     * Init
     */
    FeatureTracker (const std::string& calib_path = CALIB_PATH)
    {
        detector = cv::ORB::create (FEATURE_N);
        matcher = cv::DescriptorMatcher::create (
            cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

        // Load Q matrix from calibration yml
        cv::FileStorage fs (calib_path, cv::FileStorage::READ);
        if (!fs.isOpened ())
            throw std::runtime_error ("Failed to open calibration file");

        fs["K1"] >> K1;
        fs["D1"] >> D1;
        fs["Q"] >> Q;
        fs.release ();

        cam_world_pos = Eigen::Vector3d::Zero ();
        cam_world_rot = Eigen::Quaterniond::Identity ();
        cam_initialized = false;

        debug = true;
    }

    /**
     * Initialize camera pose from EKF (call this once at startup)
     */
    void initialize_pose (const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot)
    {
        cam_world_pos = pos;
        cam_world_rot = rot;
        cam_initialized = true;
    }

    /**
     * Returns ABSOLUTE camera pose in world frame
     * @warning optional
     */
    std::optional<Pose> get_pose (const CamData& l_cd, const CamData& r_cd)
    {
        cv::Mat l_desc;
        std::vector<cv::DMatch> stereo_matches;

        auto [l_kp, r_kp] = extract_and_stereo_match (l_cd, r_cd, l_desc,
                                                      stereo_matches);
        if (stereo_matches.empty ())
        {
            std::cout << "No matches" << std::endl;
            return std::nullopt;
        }

        auto cur_fts = triangulate (stereo_matches, l_kp, r_kp, l_desc);

        // Get RELATIVE motion from previous camera frame
        auto relative_motion = estimate_motion (l_kp, l_desc, l_cd.img);

        // Save features for next frame
        if (!cur_fts.empty ())
        {
            std::cout << "No Features" << std::endl;
            prev_descs = std::move (l_desc); 
            prev_fts = std::move (cur_fts);
        }

        if (!relative_motion || !cam_initialized)
        {
            std::cout << "No motion" << std::endl;
            return std::nullopt;
        }    

        // Accumulate relative motion into absolute world pose
        // Transform camera-frame delta into world frame
        Eigen::Vector3d delta_world = cam_world_rot * relative_motion->pos;
        cam_world_pos += delta_world;
        
        // Compose rotations
        cam_world_rot = cam_world_rot * relative_motion->rot;
        cam_world_rot.normalize ();

        // Return ABSOLUTE pose
        Pose absolute_pose;
        absolute_pose.time_ns = l_cd.time_ns + (r_cd.time_ns - l_cd.time_ns) / 2;
        absolute_pose.pos = cam_world_pos;
        absolute_pose.rot = cam_world_rot;

        return absolute_pose;
    }
};