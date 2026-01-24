/**
 * @file cam.hpp
 * @brief Raw stereo amera interface
 * @cite https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera
 * @note According to the wiki, the frames not hardware synched with each other,
 *       nor are they synched with the IMU. Awesome :-D.
 */

#pragma once

#include "defs.h"
#include "consts.h"
#include "helpers.h"
#include <stdexcept>
#include <chrono>
#include <opencv2/opencv.hpp>

class Cam
{
private:
    cv::VideoCapture left_cam;
    cv::VideoCapture right_cam;

    // Remapping
    cv::Mat map1_l, map2_l, map1_r, map2_r;

public:
    /**
     * Initialize
     */
    Cam ()
    {
        // Load calibration
        cv::FileStorage fs (CALIB_PATH, cv::FileStorage::READ);
        if (!fs.isOpened ())
            throw std::runtime_error ("Failed to open calibration file");

        cv::Mat K1, D1, K2, D2, R1, R2, P1, P2;
        cv::Size img_size;

        fs["K1"] >> K1; fs["D1"] >> D1;
        fs["K2"] >> K2; fs["D2"] >> D2;
        fs["R1"] >> R1; fs["R2"] >> R2;
        fs["P1"] >> P1; fs["P2"] >> P2;
        fs["size"] >> img_size;
        fs.release ();

        // Pre-compute rectification maps
        cv::initUndistortRectifyMap (K1, D1, R1, P1, img_size,
                                     CV_32FC1, map1_l, map2_l);
        cv::initUndistortRectifyMap (K2, D2, R2, P2, img_size,
                                     CV_32FC1, map1_r, map2_r);

        // Open cams
        left_cam.open (LEFT_CAM_ID, cv::CAP_V4L2);
        if (!left_cam.isOpened ())
            throw std::runtime_error ("Could not open left camera");
        
        right_cam.open (RIGHT_CAM_ID, cv::CAP_V4L2); 
        if (!right_cam.isOpened ())
            throw std::runtime_error ("Could not open right camera");

        left_cam.set (cv::CAP_PROP_FRAME_WIDTH, W_PX);
        left_cam.set (cv::CAP_PROP_FRAME_HEIGHT, H_PX);
        right_cam.set (cv::CAP_PROP_FRAME_WIDTH, W_PX);
        right_cam.set (cv::CAP_PROP_FRAME_HEIGHT, H_PX);
    }

    /**
     * Clean outputs, then store undistorted and rectified caps
     * 
     * @note Always reads left before right
     * @warning Check CamData dirty bits for success
     */
    void read (CamData& left_out, CamData& right_out)
    {
        left_out = {};
        right_out = {};
        
        // Synchronized grab
        if (!left_cam.grab ())
            return;
        left_out.time_ns = get_time_ns ();
        
        if (!right_cam.grab ())
            return;
        right_out.time_ns = get_time_ns ();

        cv::Mat raw_l, raw_r;
        left_cam.retrieve (raw_l);
        right_cam.retrieve (raw_r);

        cv::remap (raw_l, left_out.img, map1_l, map2_l, cv::INTER_LINEAR);
        left_out.dirty = true;

        cv::remap (raw_r, right_out.img, map1_r, map2_r, cv::INTER_LINEAR);
        right_out.dirty = true;
    }

    /**
     * Cleanup
     */
    ~Cam ()
    {
        left_cam.release ();
        right_cam.release ();
    }
};