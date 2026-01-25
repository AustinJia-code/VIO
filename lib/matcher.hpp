/**
 * @file matcher.hpp
 * @brief Handles depth and sparse feature matching
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
class Matcher
{
private:
    cv::Ptr<cv::ORB> detector;  // corner/feature detector
    cv::Ptr<cv::DescriptorMatcher> matcher;

    cv::Mat K1, D1, Q;

    std::vector<Feature> prev_fts;
    cv::Mat prev_descs;

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
                                         const cv::Mat& l_desc)
    {
        if (prev_fts.empty ())
            return std::nullopt;

        std::vector<cv::DMatch> t_matches;
        matcher->match (prev_descs, l_desc, t_matches);

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
            Pose p;
            p.pos = Eigen::Vector3d (tvec.at<double> (0),
                                     tvec.at<double> (1),
                                     tvec.at<double> (2));

            // axis angle to quat
            cv::Mat R;
            cv::Rodrigues (rvec, R);
            Eigen::Matrix3d eigen_R;
            cv::cv2eigen (R, eigen_R);
            p.rot = Eigen::Quaterniond (eigen_R);
            
            return p;
        }

        return std::nullopt;
    }

public:
    /**
     * Init
     */
    Matcher ()
    {
        detector = cv::ORB::create (FEATURE_N);
        matcher = cv::DescriptorMatcher::create (
            cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

        // Load Q matrix from calibration yml
        cv::FileStorage fs (CALIB_PATH, cv::FileStorage::READ);
        if (!fs.isOpened ())
            throw std::runtime_error ("Failed to open calibration file");

        fs["K1"] >> K1;
        fs["D1"] >> D1;
        fs["Q"] >> Q;
        fs.release ();
    }

    /**
     * Returns change in pose from last invocation
     * @warning optional
     */
    std::optional<Pose> match (const CamData& l_cd, const CamData& r_cd)
    {
        cv::Mat l_desc;
        std::vector<cv::DMatch> stereo_matches;

        // Get features
        auto [l_kp, r_kp] = extract_and_stereo_match (l_cd, r_cd, l_desc,
                                                      stereo_matches);
        if (stereo_matches.empty ())
            return std::nullopt;

        // Triangulate
        auto cur_fts = triangulate (stereo_matches, l_kp, r_kp, l_desc);

        // Estimate Motion
        auto pose_opt = estimate_motion (l_kp, l_desc);

        // Save stuff
        if (!cur_fts.empty ())
        {
            prev_descs = std::move (l_desc); 
            prev_fts = std::move (cur_fts);
        }

        if (pose_opt)
            pose_opt->time_ns = l_cd.time_ns + (r_cd.time_ns - l_cd.time_ns) / 2;

        return pose_opt;
    }
};