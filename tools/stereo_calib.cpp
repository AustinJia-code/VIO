/**
 * @file stereo_calib.cpp
 * @brief Calibrate separate stereo cameras and verify rectification
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include "consts.h"

/**
 * Eye debugging
 */
void draw_epipolar_lines (cv::Mat& canvas, int num_lines = 20)
{
    for (int i = 0; i < canvas.rows; i += canvas.rows / num_lines)
    {
        cv::line (canvas, cv::Point (0, i), cv::Point (canvas.cols, i),
                  cv::Scalar (0, 255, 0), 1);
    }
}

/**
 * Calibrate from images
 */
int main()
{
    const std::string out_path = CALIB_PATH;
    const std::string in_dir = CALIB_DIR;
    const cv::Size board_size (CALIB_W, CALIB_H);
    const float checker_mm = CALIB_MM;

    std::vector<std::vector<cv::Point3f>> obj_pts;
    std::vector<std::vector<cv::Point2f>> img_pts_l, img_pts_r;
    
    // Define 3D coordinates of corners in board coordinate system
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < board_size.height; i++)
        for (int j = 0; j < board_size.width; j++)
            objp.push_back (cv::Point3f (j * checker_mm, i * checker_mm, 0));

    std::vector<cv::String> left_names;
    cv::glob (in_dir + "left_*.jpg", left_names);
    
    cv::Size img_size;
    cv::Mat last_l, last_r;

    std::cout << "Processing " << left_names.size () << " pairs..." << std::endl;

    for (size_t k = 0; k < left_names.size (); k++)
    {
        // Construct the right filename based on the left index
        std::string l_path = left_names[k];
        std::string r_path = l_path;
        size_t pos = r_path.find ("left_");
        r_path.replace (pos, 5, "right_");

        cv::Mat left = cv::imread (l_path);
        cv::Mat right = cv::imread (r_path);
        if (left.empty () || right.empty ()) continue;
        
        img_size = left.size ();
        std::vector<cv::Point2f> corners_l, corners_r;

        bool found_l = cv::findChessboardCorners (left, board_size, corners_l, 
                        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        bool found_r = cv::findChessboardCorners (right, board_size, corners_r, 
                        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found_l && found_r)
        {
            cv::Mat gray_l, gray_r;
            cv::cvtColor (left, gray_l, cv::COLOR_BGR2GRAY);
            cv::cvtColor (right, gray_r, cv::COLOR_BGR2GRAY);

            cv::cornerSubPix (gray_l, corners_l, cv::Size (11, 11), cv::Size (-1, -1),
                              cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
            cv::cornerSubPix (gray_r, corners_r, cv::Size (11, 11), cv::Size (-1, -1),
                              cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

            img_pts_l.push_back (corners_l);
            img_pts_r.push_back (corners_r);
            obj_pts.push_back (objp);
            
            last_l = left.clone ();
            last_r = right.clone ();
            std::cout << "[OK] Found corners in pair " << k << std::endl;
        }
        else
        {
            std::cout << "[FAIL] Could not find corners in pair " << k << std::endl;
        }
    }

    if (obj_pts.size() < 10)
    {
        std::cerr << "Need at least 10 valid pairs, found: "
                  << obj_pts.size () << std::endl;
        return -1;
    }

    // Calibration
    cv::Mat K1, D1, K2, D2, R, T, E, F;
    std::cout << "\nRunning stereo calibration..." << std::endl;

    // We let stereoCalibrate estimate intrinsics and extrinsics simultaneously for better global fit
    double rms = cv::stereoCalibrate (obj_pts, img_pts_l, img_pts_r,
                                      K1, D1, K2, D2, img_size,
                                      R, T, E, F,
                                      cv::CALIB_RATIONAL_MODEL, // Better for wide-angle IMX219
                                      cv::TermCriteria (cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-5));

    std::cout << "Stereo RMS Error: " << rms << std::endl;
    if (rms > 1.0) std::cout << "WARNING: RMS > 1.0. Calibration might be inaccurate!" << std::endl;

    // Rectification
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify (K1, D1, K2, D2, img_size, R, T, R1, R2, P1, P2, Q);

    // Save to YML
    cv::FileStorage fs (out_path, cv::FileStorage::WRITE);
    fs << "K1" << K1 << "D1" << D1 << "K2" << K2 << "D2" << D2 
       << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 
       << "P1" << P1 << "P2" << P2 << "Q" << Q << "size" << img_size;
    fs.release ();

    // Visualization
    cv::Mat m1l, m2l, m1r, m2r, rect_l, rect_r;
    cv::initUndistortRectifyMap (K1, D1, R1, P1, img_size, CV_32FC1, m1l, m2l);
    cv::initUndistortRectifyMap (K2, D2, R2, P2, img_size, CV_32FC1, m1r, m2r);
    cv::remap (last_l, rect_l, m1l, m2l, cv::INTER_LINEAR);
    cv::remap (last_r, rect_r, m1r, m2r, cv::INTER_LINEAR);

    cv::Mat canvas;
    cv::hconcat (rect_l, rect_r, canvas);
    draw_epipolar_lines (canvas);

    std::cout << "Done. Calibration saved to " << out_path << std::endl;
    cv::imshow ("Verify: Chessboard corners must align on green lines", canvas);
    cv::waitKey (0);

    return 0;
}