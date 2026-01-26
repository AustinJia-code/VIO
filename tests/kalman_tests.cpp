#include <iostream>
#include <assert.h>
#include "ekf.hpp"
#include "helpers.h"

/**
 * Test IMU dead reckoning on synthetic straight line
 * No noise
 */
void test_straight_line ()
{
    EKF ekf;
    // Start at origin, stationary, identity rotation
    ekf.set_state (Eigen::Vector3d::Zero (), Eigen::Vector3d::Zero (),
                   Eigen::Quaterniond::Identity ());

    ns_t dt_ns = sec_to_ns (sec_t {0.01}); // 10ms
    ns_t current_time = dt_ns;

    // Simulate 2.0 seconds of constant 1.0 m/s^2 acceleration in X
    // We expect: 
    // Velocity = a * t = 1.0 * 2.0 = 2.0 m/s
    // Position = 0.5 * a * t^2 = 0.5 * 1.0 * 4.0 = 2.0 m
    for (int i = 0; i < 201; ++i)
    {
        IMUData data;
        data.time_ns = current_time;
        // Gravity is -9.81 in Z. To simulate 1.0 m/s^2 forward, 
        // the IMU must sense 1.0 in X and 9.81 normal in Z.
        data.accel = Eigen::Vector3d (1.0, 0.0, G); 
        data.gyro = Eigen::Vector3d::Zero ();

        ekf.predict (data);
        current_time += dt_ns;
    }

    Pose result = ekf.get_estimate ();
    
    std::cout << "--- Straight Line Test (2s @ 1m/s^2) ---" << std::endl;
    std::cout << "Expected Pos X: 2.0, Got: " << result.pos.x () << std::endl;
    std::cout << "Expected Pos Z: 0.0, Got: " << result.pos.z () << std::endl;
}

/**
 * Test IMU dead reckoning on synthetic rotation
 * No noise
 */
void test_rotation_90_deg ()
{
    EKF ekf;
    ekf.set_state (Eigen::Vector3d::Zero (), Eigen::Vector3d::Zero (),
                   Eigen::Quaterniond::Identity ());

    // Rotate 90 degrees (PI/2) over 1 second around Z axis
    // Omega = PI / 2 rad/s
    double omega_z = M_PI / 2.0;
    ns_t dt_ns = sec_to_ns (sec_t {0.01});
    ns_t current_time = dt_ns;

    for (int i = 0; i < 101; ++i)
    {
        IMUData data;
        data.time_ns = current_time;
        data.accel = Eigen::Vector3d (0, 0, G); 
        data.gyro = Eigen::Vector3d (0, 0, omega_z);

        ekf.predict (data);
        current_time += dt_ns;
    }

    Pose result = ekf.get_estimate ();
    // Quaternion for 90 deg around Z is [w: 0.707, x: 0, y: 0, z: 0.707]
    std::cout << "\n--- Rotation Test (90 deg Z) ---" << std::endl;
    std::cout << "Expect: 0.707 + 0.707i" << std::endl;
    std::cout << "Final Quat: " << result.rot.w () << " + "
                                << result.rot.z () << "i" << std::endl;
}

int main ()
{
    test_straight_line ();
    test_rotation_90_deg ();
    
    return 0;
}