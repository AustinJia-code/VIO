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
    void update (const Pose& cam_pose)
    {
        // Innovation (Residual)
        Eigen::Vector3d pos_err = cam_pose.pos - x.segment<3>(0);
        Eigen::Quaterniond q_ekf (x (9), x (6),  x(7), x (8));

        Eigen::Vector3d rot_err = quat_to_axis_angle (q_ekf.normalized ().inverse () *
                                                      cam_pose.rot.normalized ());

        Eigen::Matrix<double, 6, 1> z_err;
        z_err << pos_err, rot_err;

        // Measurement Matrix H (6x15)
        // Maps state [p, v, q, ba, bg] to measurement [p, q]
        Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero ();
        H.block<3,3> (0,0) = Eigen::Matrix3d::Identity (); // position maps to position
        H.block<3,3> (3,6) = Eigen::Matrix3d::Identity (); // rotation maps to rotation

        // Measurement Noise R (cam trust)
        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity () * 0.01;

        // Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
        auto S = H * P * H.transpose () + R;
        Eigen::Matrix<double, 15, 6> K = P * H.transpose () * S.inverse ();

        // Update State
        Eigen::Matrix<double, 15, 1> dx = K * z_err;
        x.segment<3>(0) += dx.segment<3> (0); // pos
        x.segment<3>(3) += dx.segment<3> (3); // vel
        
        // Rotation update via small-angle
        Eigen::Quaterniond dq (Eigen::AngleAxisd (dx.segment<3> (6).norm (),
                                                  dx.segment<3> (6).normalized ()));
        if(dx.segment<3>(6).norm () < 1e-6)
            dq = Eigen::Quaterniond::Identity ();

        q_ekf = q_ekf * dq;
        x.segment<4>(6) << q_ekf.x (), q_ekf.y (), q_ekf.z (), q_ekf.w ();

        x.segment<3>(10) += dx.segment<3> (9);  // accel bias
        x.segment<3>(13) += dx.segment<3> (12); // gyro bias

        // Update Covariance: P = (I - KH)P
        P = (Eigen::Matrix<double, 15, 15>::Identity () - K * H) * P;
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