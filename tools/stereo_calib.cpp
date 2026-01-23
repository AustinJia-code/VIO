/**
 * @file stereo_calib.cpp
 * @brief Calibrate stereo camera, output yml
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

int main ()
{
    std::string out_path = "../data/calib/stereo_calib.yml";
    std::string in_path = "../data/calib/checker-35mm-9-7/*.jpg";

    int checker_cols = 9;            // inner corners per row
    int checker_rows = 7;            // inner corners per column
    float checker_mm = 35.0f;        // mm

    cv::Size board_size (checker_cols, checker_rows);

    // Paths to calibration images
    std::vector<cv::String> image_names;
    cv::glob (in_path, image_names);
    std::vector<cv::Mat> left_images;
    std::vector<cv::Mat> right_images;

    std::vector<cv::Mat> calib_images;
    for (cv::String image_path : image_names)
    {
        cv::Mat frame = cv::imread (image_path);
        left_images.push_back (frame (cv::Range (0, frame.rows), 
                                      cv::Range (0, frame.cols / 2)));
        right_images.push_back (frame (cv::Range (0, frame.rows), 
                                       cv::Range (frame.cols / 2, frame.cols)));
    }

    std::vector<std::vector<cv::Point3f>> obj_pts;
    std::vector<std::vector<cv::Point2f>> img_pts_l, img_pts_r;

    // Create the known 3D chessboard corner positions
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < checker_rows; i++)
        for (int j = 0; j < checker_cols; j++)
            objp.push_back (cv::Point3f (j * checker_mm, i * checker_mm, 0));

    // Evaluate all images
    for (size_t i = 0; i < left_images.size (); i++)
    {
        cv::Mat left = left_images[i];
        cv::Mat right = right_images[i];

        std::vector<cv::Point2f> corners_left, corners_right;
        bool found_left = cv::findChessboardCorners (left, board_size, corners_left);
        bool found_right = cv::findChessboardCorners (right, board_size, corners_right);

        if (!found_left || !found_right)
            continue;

        cv::Mat gray_left, gray_right;
        cv::cvtColor (left, gray_left, cv::COLOR_BGR2GRAY);
        cv::cvtColor (right, gray_right, cv::COLOR_BGR2GRAY);

        cv::cornerSubPix (gray_left, corners_left,
                        cv::Size (11,11), cv::Size (-1,-1),
                        cv::TermCriteria (cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 
                                          30, 0.001));

        cv::cornerSubPix (gray_right, corners_right,
                        cv::Size (11,11), cv::Size (-1,-1),
                        cv::TermCriteria (cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                          30, 0.001));


        img_pts_l.push_back (corners_left);
        img_pts_r.push_back (corners_right);
        obj_pts.push_back (objp);

        std::cout << "Pair " << i+1 << " processed.\n";
    }

    // Calibrate individual cameras
    cv::Mat K1, D1, K2, D2;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::calibrateCamera (obj_pts, img_pts_l, cv::Size (1280, 960), K1, D1, rvecs, tvecs);
    cv::calibrateCamera (obj_pts, img_pts_r, cv::Size (1280, 960), K2, D2, rvecs, tvecs);

    // Calibrate stereo
    cv::Mat R, T, E, F;
    cv::stereoCalibrate (obj_pts, img_pts_l, img_pts_r,
                         K1, D1, K2, D2, cv::Size (1280, 960),
                         R, T, E, F,
                         cv::CALIB_FIX_INTRINSIC,
                         cv::TermCriteria (cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 
                                           100, 1e-5));

    // Rectify
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify (K1, D1, K2, D2, cv::Size (1280, 960),
                        R, T, R1, R2, P1, P2, Q);

    // Save
    cv::FileStorage fs (out_path, cv::FileStorage::WRITE);
    fs << "K1" << K1 << "D1" << D1 << "K2" << K2 << "D2" << D2
       << "R" << R << "T" << T << "E" << E << "F" << F
       << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
    fs.release ();

    std::cout << "Calibration saved to " << out_path << std::endl;

    return 0;
}