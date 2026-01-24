/**
 * @file stereo_capture.cpp
 * @brief Capture tool for RPi5 Stereo Calibration
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/stat.h>
#include "consts.h"

/**
 * Capture images for calibration
 */
int main()
{
    // Init cams
    cv::VideoCapture left_cam (LEFT_CAM_ID, cv::CAP_V4L2);
    cv::VideoCapture right_cam (RIGHT_CAM_ID, cv::CAP_V4L2);

    left_cam.set (cv::CAP_PROP_FRAME_WIDTH, W_PX);
    left_cam.set (cv::CAP_PROP_FRAME_HEIGHT, H_PX);
    right_cam.set (cv::CAP_PROP_FRAME_WIDTH, W_PX);
    right_cam.set (cv::CAP_PROP_FRAME_HEIGHT, H_PX);

    if (!left_cam.isOpened () || !right_cam.isOpened ())
    {
        std::cerr << "Error: Could not open one or both cameras!" << std::endl;
        std::cerr << "Ensure libcamera isn't holding the locks." << std::endl;
        return -1;
    }

    std::string out_dir = CALIB_DIR;
    std::vector<cv::String> left_names;
    cv::glob (out_dir + "left_*.jpg", left_names);
    int count = left_names.size ();
    
    std::cout << "--- Stereo Capture Tool ---" << std::endl;
    std::cout << "SPACE: Capture Frame | ESC: Exit" << std::endl;

    while (true)
    {
        cv::Mat left_frame, right_frame, canvas;
        left_cam >> left_frame;
        right_cam >> right_frame;

        if (left_frame.empty () || right_frame.empty ()) continue;

        // Show side-by-side for preview
        cv::hconcat (left_frame, right_frame, canvas);
        cv::imshow ("Stereo Preview - Press SPACE to capture", canvas);

        char key = (char) cv::waitKey (1);
        if (key == 27)      // ESC
            break;
        if (key == ' ')     // Spacebar
        {
            std::string name_l = out_dir + "left_" + std::to_string (count) + ".jpg";
            std::string name_r = out_dir + "right_" + std::to_string (count) + ".jpg";
            
            cv::imwrite (name_l, left_frame);
            cv::imwrite (name_r, right_frame);
            
            std::cout << "Saved pair " << count << std::endl;
            count++;
        }
    }

    return EXIT_SUCCESS;
}