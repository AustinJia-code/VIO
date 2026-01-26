/**
 * @file ekf.hpp
 * @brief Combines IMU and Stereo using an EKF
 */

#pragma once

#include "defs.h"
#include "consts.h"

#include "helpers.h"

class EKF
{
private:
    // Treat rotation as quaternion in x
    Eigen::Matrix<double, 16, 1> x;   // [pos, vel, rot, accel_bias, gyro_bias]
    Eigen::Matrix<double, 15, 15> P;  // Covariance Matrix
    Eigen::Matrix<double, 15, 15> Q;  // Process noise

    const Eigen::Vector3d G_vec = {0, 0, -G};

    ns_t last_ns;
    ns_t last_pred_ns;

public:
    /**
     * Constructor
     */
    EKF ()
    {
        x.setZero ();
        x.segment<4> (6) << 0, 0, 0, 1; // Initial rotation
        P.setIdentity () * 0.1;         // Start with some uncertainty

        Q.setZero ();                   // TODO: Tune
        Q.diagonal () += Eigen::VectorXd::Constant (15, 0.001);

        Q.setZero ();
        Q.block<3,3> (0,0) = Eigen::Matrix3d::Identity () * 0.01;  // position
        Q.block<3,3> (3,3) = Eigen::Matrix3d::Identity () * 0.05;  // velocity  
        Q.block<3,3> (6,6) = Eigen::Matrix3d::Identity () * 0.02;  // rotation
        Q.block<3,3> (9,9) = Eigen::Matrix3d::Identity () * 0.001; // accel bias
        Q.block<3,3> (12,12) = Eigen::Matrix3d::Identity () * 0.001; // gyro bias
    }
    
    /**
     * Predict with raw IMU data (relative)
     */ 
    void predict (const IMUData& data)
    {
        sec_t dt = ns_to_sec (data.time_ns - last_pred_ns);
        
        // If time travel or big jump, don't risk integration
        if (dt <= 0 || dt > 0.1)
        {
            last_pred_ns = data.time_ns;
            return;
        }
            
        Eigen::Vector3d pos = x.segment<3> (0);
        Eigen::Vector3d vel = x.segment<3> (3);
        Eigen::Quaterniond q (x (9), x (6), x (7), x (8)); // w, x, y, z

        // Accel to world frame using old rotation
        Eigen::Vector3d accel_bias = x.segment<3> (10);
        Eigen::Vector3d accel_world = q * (data.accel - accel_bias) + G_vec;

        // p1 = p0 + v0 * dt + 0.5 * a0 * dt^2
        Eigen::Vector3d new_pos = pos + vel * dt + 0.5 * accel_world * dt * dt;
        // v1 = v0 + a0 * dt
        Eigen::Vector3d new_vel = vel + accel_world * dt;

        // Integrate gyro for rotation
        Eigen::Vector3d gyro_bias = x.segment<3> (13);
        Eigen::Vector3d omega = (data.gyro - gyro_bias) * dt;
        double angle_mag = omega.norm ();
        if (angle_mag > 1e-6)
        {
            Eigen::Quaterniond delta_q (
                Eigen::AngleAxisd (angle_mag, omega.normalized ()));
            q *= delta_q;
            q.normalize ();
        }

        // Store
        x.segment<3> (0) = new_pos;
        x.segment<3> (3) = new_vel;
        x (6) = q.x ();
        x (7) = q.y ();
        x (8) = q.z ();
        x (9) = q.w ();

        // Add uncertainty
        P += Q * dt;
        last_ns = data.time_ns;
        last_pred_ns = last_ns;
    }
    
    /**
     * Update with ABSOLUTE camera pose in world frame
     */
    void update (const Pose& cam_absolute_pose)
    {
        Eigen::Vector3d pos_err = cam_absolute_pose.pos - x.segment<3> (0);

        Eigen::Quaterniond q_ekf (x (9), x (6), x (7), x (8));
        Eigen::Quaterniond q_cam = cam_absolute_pose.rot;
        
        // Kalman gain (trust camera less than IMU for short-term)
        double k_pos = 0.2;  // 20% trust in camera position
        double k_rot = 0.3;  // 30% trust in camera rotation
        
        // Weighted update
        x.segment<3> (0) += k_pos * pos_err;

        Eigen::Quaterniond q_fused = q_ekf.slerp (k_rot, q_cam);
        q_fused.normalize ();
        
        x (6) = q_fused.x ();
        x (7) = q_fused.y (); 
        x (8) = q_fused.z ();
        x (9) = q_fused.w ();

        P.diagonal () *= (1.0 - 0.5 * k_pos);
        last_ns = cam_absolute_pose.time_ns;
    }

    /**
     * Get current pose estimate
     */
    Pose get_estimate () const
    {
        Pose out = {};
        out.time_ns = last_ns;
        out.pos = x.segment<3> (0);
        out.rot = Eigen::Quaterniond (x (9), x (6), x (7), x (8));

        return out;
    }

    /**
     * Set the state of the EKF
     */
    void set_state (const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                    const Eigen::Quaterniond& rot)
    {
        x.segment<3> (0) = pos;
        x.segment<3> (3) = vel;

        x (6) = rot.x ();
        x (7) = rot.y ();
        x (8) = rot.z ();
        x (9) = rot.w ();
    }

    // Set biases
    void set_bias (const Eigen::Vector3d& accel_bias,
                   const Eigen::Vector3d& gyro_bias)
    {
        x.segment<3> (10) = accel_bias;
        x.segment<3> (13) = gyro_bias;
    }
};