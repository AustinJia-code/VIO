/**
 * @file sensor.hpp
 * @brief Generic odometer sensor interface
 */

#pragma once

#include "def.h"

/**
 * Pure abstract odometer sensor class
 */
class Odo
{
private:
    vec3_t pos;

public:
    /**
     * Update pos
     */
    virtual void update () = 0;
};


/**
 * Camera odometer interface
 */
class Cam : public Odo
{
public:
    /**
     * Init from calibration file
     */
    Cam (const std::string& file_path);

    void update () override;
};


/**
 * IMU odometer interface
 */
class IMU : public Odo
{
public:
    void update () override;
};