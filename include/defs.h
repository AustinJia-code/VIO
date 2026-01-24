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
struct Pose                 // Pose
{
    ns_t time_ns;
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
};

struct CamData              // Cam data
{
    ns_t time_ns;
    cv::Mat img;
};

struct IMUData              // IMU data
{
    ns_t time_ns;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
};