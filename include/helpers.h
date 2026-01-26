/**
 * @file helpers.h
 * @brief Various helper methods
 */

#pragma once

#include "defs.h"
#include <chrono>

/**
 * Get current time in nanoseconds
 */
static inline ns_t get_time_ns ()
{
    auto now = std::chrono::steady_clock::now ();
    return ns_t {std::chrono::duration_cast<std::chrono::nanoseconds> (
                    now.time_since_epoch ()).count ()};
}

/**
 * Quaterniond to euler angles
 */
static inline Eigen::Vector3d quat_to_euler (const Eigen::Quaterniond& q)
{
    return q.toRotationMatrix ().eulerAngles (0, 1, 2);
}

/**
 * Quaterniond to axis * angle
 */
Eigen::Vector3d quat_to_axis_angle (const Eigen::Quaterniond& q_in)
{
    Eigen::Quaterniond q = q_in.normalized ();

    // Ensure shortest rotation
    if (q.w () < 0)
        q.coeffs () *= -1;

    Eigen::Vector3d v = q.vec ();
    double w = q.w ();

    double v_norm  = v.norm ();

    // Small-angle approximation
    if (v_norm  < 1e-8)
        return 2.0 * v;

    double theta = 2.0 * std::atan2 (v_norm, w);
    return theta * (v / v_norm );
}


/**
 * Nanoseconds to seconds
 */
static inline sec_t ns_to_sec (const ns_t time_ns)
{
    return sec_t (static_cast<double> (time_ns) / 1e9); 
}

/**
 * Seconds to nanoseconds
 */
static inline sec_t sec_to_ns (const sec_t time_ns)
{
    return ns_t (time_ns * 1e9); 
}

/**
 * Seconds to microseconds
 */
static inline us_t sec_to_us (const sec_t time_s)
{
    return us_t (time_s * 1e6); 
}