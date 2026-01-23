/**
 * @file ekf.hpp
 * @brief Combines IMU and Cam using an EKF
 */

#pragma once

#include "defs.h"

class EKF
{
private:
    Eigen::VectorXd x;  // State: [pos, vel, rot, accel_bias, gyro_bias]
    Eigen::MatrixXd P;  // Covariance Matrix
    Eigen::MatrixXd Q;  // Process Noise

public:
    /**
     * Constructor
     */
    EKF ()
    {
        x.setZero (15);             // 5 x 3D vectors
        P.setIdentity (15, 15);
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