/**
 * @file ekf.hpp
 * @brief Combines IMU and Cam using an EKF
 */

#pragma once

#include "defs.h"
#include "consts.h"

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
    }
    
    /**
     * Predict with raw IMU data (relative)
     */ 
    void predict (const IMUData& data)
    {
        if (!data.dirty)
            return;

        ns_t dt = ns_to_sec (data.time_ns - last_pred_ns);
        // If time travel or big jump, don't risk integration
        if (dt <= 0 || dt > 0.1)
        {
            last_pred_ns = data.time_ns;
            return;
        }
            
        Eigen::Vector3d pos = x.segment<3> (0);
        Eigen::Vector3d vel = x.segment<3> (3);
        Eigen::Quaterniond q (x (9), x (6), x (7), x (8)); // w, x, y, z
        Eigen::Vector3d accel_bias = x.segment<3> (10);
        Eigen::Vector3d gyro_bias = x.segment<3> (13);

        // Accel to world frame using old rotation
        Eigen::Vector3d accel_world = q * (data.accel - accel_bias) + G_vec;

        // p1 = p0 + v0 * dt + 0.5 * a0 * dt^2
        Eigen::Vector3d new_pos = pos + vel * dt + 0.5 * accel_world * dt * dt;
        // v1 = v0 + a0 * dt
        Eigen::Vector3d new_vel = vel + accel_world * dt;

        // Integrate gyro for rotation
        Eigen::Vector3d omega = (data.gyro - gyro_bias) * dt;
        double angle_mag = omega.norm ();
        if (angle_mag > 1e-6)
        {
            Eigen::Quaterniond delta_q (
                Eigen::AngleAxisd (angle_mag, omega.normalized ()));
            q *= delta_q;
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
     * Update with processed Cam pose ("absolute")
     */
    void update (const Pose& pose)
    {
        if (!pose.dirty)
            return;

        Eigen::Vector3d pos_err = pose.pos - x.segment<3> (0);

        Eigen::Quaterniond q_pred (x (9), x (6), x (7), x (8));
        Eigen::Quaterniond q_meas = pose.rot;
        Eigen::Quaterniond q_err = q_meas * q_pred.inverse ();

        // Kalman Gain
        // TODO: K = P * H' * (H * P * H' + R)^-1
        // R = measurement noise
        double R_pos = 0.01;    // 1cm uncertainty
        double R_rot = 0.02;    // Small radian uncertainty

        // Camera trust gain
        double k = 0.8;

        x.segment<3> (0) += k * pos_err;

        // Slerp for smooth rotation mean
        Eigen::Quaterniond q_slerp = q_pred.slerp (k, q_meas);
        x (6) = q_slerp.x ();
        x (7) = q_slerp.y (); 
        x (8) = q_slerp.z ();
        x (9) = q_slerp.w ();

        // Remove uncertainty
        P.diagonal () *= (1.0 - k);
        last_ns = get_time_ns ();
    }

    /**
     * Get current pose estimate
     */
    void get_estimate (Pose& out) const
    {
        out.time_ns = last_ns;
        out.pos = x.segment<3> (0);
        out.rot = Eigen::Quaterniond (x (9), x (6), x (7), x (8));
    }
};