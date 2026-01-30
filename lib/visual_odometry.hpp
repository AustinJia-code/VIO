/**
 * @file VisualOdometry.cpp
 * @brief Visual Odometry class for stereo camera motion estimation
 * 
 * Computes absolute camera motion (with metric scale) between consecutive
 * stereo frames using depth from stereo matching and 3D-2D PnP.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <cstdint>
#include "defs.h"

/**
 * Camera calibration for euroc dataset
 */
class CameraCalibration
{
private:
    /**
     * Helper for absolutely buck wild calib format
     */
    bool load_euroc (const std::string& path, cv::Mat& T_BS, cv::Size& size)
    {
        std::ifstream file (path);
        if (!file.is_open ())
        {
            std::cerr << "Could not open: " << path << std::endl;
            return false;
        }
        
        std::vector<double> T_BS_data;
        std::vector<int> resolution;
        std::string line;
        bool in_T_BS_data = false;
        
        while (std::getline (file, line))
        {
            // Skip empty lines
            if (line.empty ())
                continue;
            
            // Remove leading/trailing whitespace
            size_t first = line.find_first_not_of (" \t\r\n");
            if (first == std::string::npos)
                continue;
            size_t last = line.find_last_not_of (" \t\r\n");
            line = line.substr (first, last - first + 1);
            
            // Skip comments
            if (line[0] == '#')
                continue;
            
            // Check for T_BS data section
            if (line.find ("data:") != std::string::npos && !in_T_BS_data)
            {
                in_T_BS_data = true;
                size_t bracket_start = line.find ('[');
                if (bracket_start != std::string::npos)
                    line = line.substr (bracket_start);
                else
                    continue;
            }
            
            // Parse T_BS data
            if (in_T_BS_data)
            {
                std::string cleaned = line;
                size_t pos;
                while ((pos = cleaned.find ('[')) != std::string::npos)
                    cleaned.erase (pos, 1);
        
                while ((pos = cleaned.find (']')) != std::string::npos)
                    cleaned.erase (pos, 1);
            
                
                std::istringstream iss (cleaned);
                std::string token;
                while (std::getline (iss, token, ','))
                {
                    size_t t_first = token.find_first_not_of (" \t\r\n");
                    if (t_first == std::string::npos)
                        continue;
                    size_t t_last = token.find_last_not_of (" \t\r\n");
                    token = token.substr (t_first, t_last - t_first + 1);
                    
                    if (!token.empty ())
                    {
                        try {
                            T_BS_data.push_back (std::stod (token));
                        } catch (...) {}
                    }
                }
                
                if (line.find (']') != std::string::npos)
                    in_T_BS_data = false;
            }
            
            // Parse resolution
            if (line.find ("resolution:") != std::string::npos)
            {
                size_t bracket_start = line.find ('[');
                size_t bracket_end = line.find (']');
                if (bracket_start != std::string::npos && bracket_end != std::string::npos)
                {
                    std::string res_str = line.substr (bracket_start + 1, bracket_end - bracket_start - 1);
                    std::istringstream iss (res_str);
                    std::string token;
                    while (std::getline (iss, token, ','))
                    {
                        size_t t_first = token.find_first_not_of (" \t\r\n");
                        if (t_first == std::string::npos)
                            continue;
                        size_t t_last = token.find_last_not_of (" \t\r\n");
                        token = token.substr (t_first, t_last - t_first + 1);
                        
                        if (!token.empty ())
                        {
                            try {
                                resolution.push_back (std::stoi (token));
                            } catch (...) {}
                        }
                    }
                }
            }
        }
        
        file.close ();
        
        // Validate and convert T_BS
        if (T_BS_data.size () != 16)
        {
            std::cerr << "Invalid T_BS data size: " << T_BS_data.size() << std::endl;
            return false;
        }
        
        T_BS = cv::Mat (4, 4, CV_64F);
        for (int i = 0; i < 16; i++)
            T_BS.at<double> (i / 4, i % 4) = T_BS_data[i];
        
        // Set resolution
        if (resolution.size() == 2)
            size = cv::Size(resolution[0], resolution[1]);
        
        return true;
    }

public:
    cv::Mat K1, K2;           // Intrinsic matrices
    cv::Mat D1, D2;           // Distortion coefficients
    cv::Mat R1, R2;           // Rectification transforms (identity for rectified)
    cv::Mat P1, P2;           // Projection matrices
    cv::Mat Q;                // Disparity-to-depth mapping
    cv::Mat T_BS_cam0;        // Camera 0 extrinsics (body to sensor)
    cv::Mat T_BS_cam1;        // Camera 1 extrinsics
    cv::Size image_size;
    double baseline;          // Stereo baseline in meters
    
    /**
     * Load euroc calib files
     */
    bool load (const std::string& stereo_path, 
               const std::string& cam0_path, 
               const std::string& cam1_path)
    {
        // Load stereo calibration (K1, D1, Q)
        std::cout << "Loading stereo calibration: " << stereo_path << std::endl;
        cv::FileStorage fs_stereo (stereo_path, cv::FileStorage::READ);
        if (!fs_stereo.isOpened ())
        {
            std::cerr << "Failed to open: " << stereo_path << std::endl;
            return false;
        }
        
        fs_stereo["K1"] >> K1;
        fs_stereo["D1"] >> D1;
        fs_stereo["Q"] >> Q;
        fs_stereo.release( );
        
        // K2 is same as K1 for EuRoC (same cam model)
        K2 = K1.clone ();
        D2 = D1.clone ();
        
        // Load cam0 sensor config
        std::cout << "Loading cam0 config: " << cam0_path << std::endl;
        if (!load_euroc (cam0_path, T_BS_cam0, image_size))
        {
            std::cerr << "Failed to load cam0 config" << std::endl;
            return false;
        }
        
        // Load cam1 sensor config
        std::cout << "Loading cam1 config: " << cam1_path << std::endl;
        cv::Size dummy_size;
        if (!load_euroc (cam1_path, T_BS_cam1, dummy_size))
        {
            std::cerr << "Failed to load cam1 config" << std::endl;
            return false;
        }
        
        // Calculate baseline from Q matrix
        // Q(3,2) = -1/Tx where Tx is baseline
        baseline = -1.0 / Q.at<double> (3, 2);
        
        // For pre-rectified images (EuRoC), R1 and R2 are identity
        R1 = cv::Mat::eye (3, 3, CV_64F);
        R2 = cv::Mat::eye (3, 3, CV_64F);
        
        // Projection matrices for rectified stereo
        // P1 = [K 0], P2 = [K [Tx; 0; 0]]
        P1 = cv::Mat::zeros (3, 4, CV_64F);
        K1.copyTo (P1 (cv::Rect (0, 0, 3, 3)));
        
        P2 = cv::Mat::zeros (3, 4, CV_64F);
        K1.copyTo (P2 (cv::Rect (0, 0, 3, 3)));
        P2.at<double> (0, 3) = K1.at<double> (0, 0) * baseline;  // fx * baseline
        
        return true;
    }
};

/**
 * Feature struct for stereo input
 */
struct FeatureMatches
{
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    std::vector<cv::DMatch> matches;
    cv::Mat descriptors1;
    cv::Mat descriptors2;
};

/**
 * Visual odometer from stereo input
 */
class VisualOdometry
{
private:
    // Calibration
    CameraCalibration calib_;
    bool calibrated_;
    
    // Previous frame data
    bool has_previous_frame_;
    cv::Mat prev_left_rect_;
    cv::Mat prev_depth_;
    FeatureMatches prev_features_;

    Eigen::Vector3d global_pos_;
    Eigen::Quaterniond global_rot_;
    ns_t last_timestamp_;
    
    // Pipeline methods
    std::pair<cv::Mat, cv::Mat> rectify_images (const cv::Mat& left_raw, 
                                                const cv::Mat& right_raw)
    {
        cv::Mat left_rect, right_rect;
        
        // Undistort (EuRoC images are pre-rectified, just undistort)
        cv::undistort (left_raw, left_rect, calib_.K1, calib_.D1, calib_.K1);
        cv::undistort (right_raw, right_rect, calib_.K2, calib_.D2, calib_.K2);
        
        return {left_rect, right_rect};
    }
        
    /**
     * Build depth map
     */
    cv::Mat compute_depth_map (const cv::Mat& left_rect, const cv::Mat& right_rect)
    {
        // Create stereo matcher
        int minDisparity = 0;
        int numDisparities = 16 * 6;  // Must be divisible by 16
        int blockSize = 11;
        
        cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create
        (
            minDisparity,
            numDisparities,
            blockSize,
            8 * 3 * blockSize * blockSize,    // P1
            32 * 3 * blockSize * blockSize,   // P2
            1,                                // disp12MaxDiff
            63,                               // preFilterCap
            10,                               // uniquenessRatio
            100,                              // speckleWindowSize
            32,                               // speckleRange
            cv::StereoSGBM::MODE_SGBM_3WAY
        );
        
        // Compute disparity
        cv::Mat disparity;
        stereo->compute (left_rect, right_rect, disparity);
        
        // Convert to float
        cv::Mat disparity_float;
        disparity.convertTo (disparity_float, CV_32F, 1.0/16.0);
        
        // Compute depth map manually
        cv::Mat depth = cv::Mat::zeros (disparity_float.size(), CV_32F);
        
        double focal_length = calib_.K1.at<double> (0, 0);
        double fb = focal_length * calib_.baseline;  // f * b
        
        for (int y = 0; y < disparity_float.rows; y++)
        {
            for (int x = 0; x < disparity_float.cols; x++)
            {
                float d = disparity_float.at<float> (y, x);
                if (d > 0.5)    // Valid disparity threshold
                {  
                    float z = fb / d;
                    if (z > 0 && z < 100)  // Realistic depth range
                        depth.at<float> (y, x) = z;
                }
            }
        }
        
        return depth;
    }
    
    /**
     * Track features from prev to curr frame using Lucas-Kanade
     */
    std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
    track_features (const cv::Mat& prev_img, const cv::Mat& curr_img)
    {
        // Detect corners in previous frame
        std::vector<cv::Point2f> prev_corners;
        cv::goodFeaturesToTrack (prev_img, prev_corners, 2000, 0.01, 10);

        if (prev_corners.size () < 10)
            return {{}, {}};

        // Sub-pixel refinement
        cv::cornerSubPix (prev_img, prev_corners, cv::Size (5, 5), cv::Size (-1, -1),
                          cv::TermCriteria (cv::TermCriteria::EPS +
                                           cv::TermCriteria::COUNT, 30, 0.01));

        // Forward track: prev -> curr
        std::vector<cv::Point2f> curr_corners;
        std::vector<uchar> status_fwd;
        std::vector<float> err_fwd;
        cv::calcOpticalFlowPyrLK (prev_img, curr_img, prev_corners, curr_corners,
                                  status_fwd, err_fwd, cv::Size (21, 21), 3);

        // Backward track: curr -> prev (consistency check)
        std::vector<cv::Point2f> back_corners;
        std::vector<uchar> status_bwd;
        std::vector<float> err_bwd;
        cv::calcOpticalFlowPyrLK (curr_img, prev_img, curr_corners, back_corners,
                                  status_bwd, err_bwd, cv::Size (21, 21), 3);

        // Keep only points that pass forward-backward consistency
        std::vector<cv::Point2f> good_prev, good_curr;
        for (size_t i = 0; i < prev_corners.size (); i++)
        {
            if (!status_fwd[i] || !status_bwd[i])
                continue;

            float fb_err = cv::norm (prev_corners[i] - back_corners[i]);
            if (fb_err > 1.0)
                continue;

            if (curr_corners[i].x < 0 || curr_corners[i].x >= curr_img.cols ||
                curr_corners[i].y < 0 || curr_corners[i].y >= curr_img.rows)
                continue;

            good_prev.push_back (prev_corners[i]);
            good_curr.push_back (curr_corners[i]);
        }

        return {good_prev, good_curr};
    }

    Pose estimate_motion (const std::vector<cv::Point2f>& prev_points,
                          const std::vector<cv::Point2f>& curr_points,
                          const cv::Mat& prev_depth,
                          ns_t timestamp_ns)
    {
        Pose pose;
        pose.time_ns = timestamp_ns;

        // Camera intrinsics
        double fx = calib_.K1.at<double> (0, 0);
        double fy = calib_.K1.at<double> (1, 1);
        double cx = calib_.K1.at<double> (0, 2);
        double cy = calib_.K1.at<double> (1, 2);

        std::vector<cv::Point3f> points_3d;  // 3D positions at t-1
        std::vector<cv::Point2f> points_2d;  // 2D positions at t

        for (size_t i = 0; i < prev_points.size (); i++)
        {
            int x = static_cast<int> (prev_points[i].x + 0.5);
            int y = static_cast<int> (prev_points[i].y + 0.5);

            if (x >= 0 && x < prev_depth.cols && y >= 0 && y < prev_depth.rows)
            {
                float depth = prev_depth.at<float> (y, x);

                if (depth > 0.5 && depth < 50.0)
                {
                    // Back-project to 3D using sub-pixel tracked coordinates
                    float Z = depth;
                    float X = (prev_points[i].x - cx) * Z / fx;
                    float Y = (prev_points[i].y - cy) * Z / fy;

                    points_3d.push_back (cv::Point3f (X, Y, Z));
                    points_2d.push_back (curr_points[i]);
                }
            }
        }
        
        if (points_3d.size () < 6)
        {
            std::cerr << "ERROR: Not enough 3D-2D correspondences for PnP! Got " 
                      << points_3d.size () << ", need at least 6" << std::endl;
            return pose;  // Return identity pose
        }
        
        // Solve PnP
        cv::Mat rvec, tvec;
        cv::Mat inliers;
        bool success = cv::solvePnPRansac
                        (
                            points_3d,
                            points_2d,
                            calib_.K1,
                            cv::Mat (), // No distortion
                            rvec,
                            tvec,
                            false,      // useExtrinsicGuess
                            200,        // iterationsCount
                            3.0,        // reprojectionError
                            0.99,       // confidence
                            inliers     // inliers
                        );
        
        if (!success)
        {
            std::cerr << "ERROR: PnP failed!" << std::endl;
            return pose;
        }
        
        // Convert rotation vector to matrix
        cv::Mat R_cv;
        cv::Rodrigues (rvec, R_cv);
        
        // Convert to Eigen
        Eigen::Matrix3d R_eigen;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R_eigen(i, j) = R_cv.at<double> (i, j);
        
        Eigen::Vector3d t_cv
        (
            tvec.at<double> (0),
            tvec.at<double> (1),
            tvec.at<double> (2)
        );
        
        // PnP gives transformation from world (t=0) to camera (t=1)
        // We need camera motion from t=0 to t=1
        // Camera position in world frame: -R^T * t
        pose.rot = Eigen::Quaterniond (R_eigen.transpose ());  // Inverse rotation
        pose.pos = -R_eigen.transpose () * t_cv;               // Camera motion

        return pose;
    }

public:
    /**
     * Constructor
     */
    VisualOdometry (const std::string& stereo_calib_path,
                    const std::string& cam0_calib_path,
                    const std::string& cam1_calib_path)
          : calibrated_ (false),
            has_previous_frame_ (false),
            global_pos_ (Eigen::Vector3d::Zero ()),
            global_rot_ (Eigen::Quaterniond::Identity ()),
            last_timestamp_ (0)
    {    
        calibrated_ = calib_.load (stereo_calib_path, cam0_calib_path, cam1_calib_path);
        
        if (calibrated_)
            std::cout << "Visual Odometry initialized successfully!" << std::endl;
        else
            std::cerr << "ERROR: Failed to initialize Visual Odometry!" << std::endl;
    }
    
    /**
     * Process a new stereo frame and compute motion relative to previous frame
     * @return Pose representing camera motion from previous frame to current frame
     *         Returns identity pose for first frame
     */
    Pose process_frame (const cv::Mat& left_img, 
                   const cv::Mat& right_img,
                   ns_t timestamp_ns)
    {
        Pose pose;
        pose.time_ns = timestamp_ns;
        
        if (!calibrated_ || left_img.empty () || right_img.empty ())
            return pose;

        // Rectify images
        auto [curr_left_rect, curr_right_rect] = rectify_images (left_img, right_img);
        
        // Compute depth map for current frame
        cv::Mat curr_depth = compute_depth_map (curr_left_rect, curr_right_rect);
        
        // If this is the first frame, just store it
        if (!has_previous_frame_)
        {
            prev_left_rect_ = curr_left_rect.clone ();
            prev_depth_ = curr_depth.clone ();
            has_previous_frame_ = true;
            return pose;  // Identity pose
        }
        
        // Track features between previous and current frame
        auto [prev_pts, curr_pts] = track_features (prev_left_rect_, curr_left_rect);

        // Estimate motion w/ PnP
        pose = estimate_motion (prev_pts, curr_pts, prev_depth_, timestamp_ns);
        
        // Update global pose if motion was successfully estimated
        if (pose.pos.norm () > 1e-6)  // Check if we got valid motion
        {
            global_pos_ = global_pos_ + global_rot_ * pose.pos;
            global_rot_ = global_rot_ * pose.rot;
            last_timestamp_ = timestamp_ns;
        }

        // Update previous frame
        prev_left_rect_ = curr_left_rect.clone ();
        prev_depth_ = curr_depth.clone ();
        
        return pose;
    }

    /**
     * Reset the visual odometry (clears previous frame)
     */
    void reset ()
    {
        has_previous_frame_ = false;
        prev_left_rect_.release();
        prev_depth_.release();
        prev_features_ = FeatureMatches();
        std::cout << "Visual Odometry reset." << std::endl;
    }
    
    /**
     * Check if calibration loaded successfully
     */
    bool is_calibrated () const
    {
        return calibrated_;
    }
    
    /**
     * Get camera calibration
     */
    const CameraCalibration& get_calibration () const
    {
        return calib_;
    }

    /**
     * Set global pose
     */
    void set_global_pose (const Eigen::Vector3d& pos,
                          const Eigen::Quaterniond& quat)
    {
        global_pos_ = pos;
        global_rot_ = quat;
    }

    /**
     * Global pose w.r.t init
     */
    Pose get_global_pose () const
    {
        Pose pose;
        pose.pos = global_pos_;
        pose.rot = global_rot_;
        pose.time_ns = last_timestamp_;
        return pose;
    }
};