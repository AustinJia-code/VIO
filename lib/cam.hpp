/**
 * @file cam.hpp
 * @brief Raw Camera interface
 * @cite https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera
 * @note According to the wiki, the frames not hardware synched with each other,
 *       nor are they synched with the IMU. Awesome :-D.
 */

#pragma once

#include "defs.h"

class Cam
{
private:
    /**
     * Undistort and rectify frames in place
     */
    void process ()
    {

    }

public:
    /**
     * Init from calibration file
     */
    Cam (const std::string& file_path)
    {

    }

    /**
     * Get absolute position from camera
     */
    void read (CamData& left_out, CamData& right_out)
    {
        process ();
        return;
    }
};