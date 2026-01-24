/**
 * @file matcher.hpp
 * @brief Handles depth and sparse feature matching
 */

#pragma once

#include "defs.h"
#include "consts.h"
#include <opencv2/features2d.hpp>
#include <vector>

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
     * Match features and triangulate for depth
     * Stores change in pose from last invocation into out
     * 
     * @warning Check out for data before using, not guarunteed
     */
    void match (const cv::Mat& left_img, const cv::Mat& right_img,
                Pose& out)
    {
        std::vector<cv::KeyPoint> left_kp, right_kp;
        cv::Mat left_desc, right_desc;

        detector->detectAndCompute (left_img, cv::noArray (), left_kp, left_desc);
        detector->detectAndCompute (right_img, cv::noArray (), right_kp, right_desc);

        // No good features
        if (left_desc.empty () || right_desc.empty ())
            return;

        std::vector<cv::DMatch> matches;
        matcher->match (left_desc, right_desc, matches);

        // Filter matches by ensuring horizontal constraint
        std::vector<Feature> cur_fts;
        for (const auto& match : matches)
        {
            float dy = std::abs (left_kp[match.queryIdx].pt.y - 
                                 right_kp[match.trainIdx].pt.y);
            
            // No good
            if (dy > Y_TOL_PX)
                continue;

            float u = left_kp[match.queryIdx].pt.x;
            float v = left_kp[match.queryIdx].pt.y;
            float disparity = u - right_kp[match.trainIdx].pt.x;

            // Negative disparity impossible
            if (disparity < 0.1f)
                continue;

            // Triangulate
            // [X, Y, Z, W] = Q * [u, v, disparity, 1]
            cv::Mat pos = Q * (cv::Mat_<double> (4,1) << u, v, disparity, 1.0);

            float w = pos.at<double> (3);

            Feature f;
            f.pt2d = left_kp[match.queryIdx].pt;
            f.pt3d = cv::Point3f (pos.at<double>(0) / w,
                                  pos.at<double>(1) / w,
                                  pos.at<double>(2) / w);
            f.desc = left_desc.row (match.queryIdx).clone ();

            cur_fts.push_back (f);
        }

        // Temporal matching
        if (prev_fts.size () == 0)
        {
            std::swap (prev_fts, cur_fts);
            return;
        }

        std::vector<cv::Point3f> pts_3d_old;
        std::vector<cv::Point2f> pts_2d_cur;

        cv::Mat prev_descriptors;
        for (Feature& f : prev_fts)
            prev_descriptors.push_back (f.desc);
        
        std::vector<cv::DMatch> temporal_matches;
        matcher->match (prev_descriptors, left_desc, temporal_matches);

        for (const auto& match : temporal_matches)
        {
            pts_3d_old.push_back (prev_fts[match.queryIdx].pt3d);
            pts_2d_cur.push_back (left_kp[match.queryIdx].pt);
        }

        if (pts_3d_old.size () < MIN_MATCHES)
        {
            std::swap (prev_fts, cur_fts);
            return;
        }

        // Superfunction to find perspective based on 3d and 2d points,
        // as well as do majority vote. Wow.
        cv::Mat rvec, tvec, inliers;
        bool success = cv::solvePnPRansac (pts_3d_old, pts_2d_cur, K1, D1,
                                           rvec, tvec, false, 100, 2.0, 0.99,
                                           inliers);

        if (!success)
        {
            std::swap (prev_fts, cur_fts);
            return;
        }

        cv::Mat R;
        cv::Rodrigues (rvec, R);

        Eigen::Matrix3d eigen_R;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                eigen_R (i,j) = R.at<double> (i,j);
                    
        out.pos = Eigen::Vector3d (tvec.at<double> (0),
                                   tvec.at<double> (1),
                                   tvec.at<double> (2));
        out.rot = Eigen::Quaterniond (eigen_R);
    }
};