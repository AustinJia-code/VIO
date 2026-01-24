/**
 * @file defs.h
 * @brief typedefs and structs
 */

#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

/*** TYPEDEFS ***/
using ns_t = int64_t;       // Nanoseconds
using us_t = int64_t;       // Microseconds
using ms_t = int64_t;       // Milliseconds
using sec_t = double;       // Seconds
using byte_t = u_int8_t;
using data_t = float;


/*** STRUCTS ***/
/**
 * Generic data container
 */
struct DataContainer
{
    ns_t time_ns;
    bool dirty;
};

struct Pose : public DataContainer 
{
    Eigen::Vector3d pos;    // Eigen stores as doubles, we use float
    Eigen::Quaterniond rot; // All quats stored as {x, y, z, w}
};

struct CamData : public DataContainer
{
    cv::Mat img;
};

struct IMUData : DataContainer
{
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
};