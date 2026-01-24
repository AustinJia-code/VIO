/**
 * @file ekf.hpp
 * @brief Combines IMU and Cam using an EKF
 */

#pragma once

#include "defs.h"

class EKF
{
private:
    Eigen::Matrix<double, 15, 1> x;   // [pos, vel, rot, accel_bias, gyro_bias]
    Eigen::Matrix<double, 16, 16> P;  // Covariance Matrix (rot quaternion)
    Eigen::Matrix<double, 15, 15> Q;  // Process Noise

public:
    /**
     * Constructor
     */
    EKF ()
    {
        x.setZero ();
        P.setIdentity ();
    }
    
    /**
     * Predict with raw IMU data (relative)
     */ 
    void predict (const IMUData& data, double dt)
    {

    }
    
    /**
     * Update with processed Cam pose ("absolute")
     */
    void update (const Pose& pose, double confidence)
    {

    }
};