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
 * Nanoseconds to seconds
 */
static inline sec_t ns_to_sec (const ns_t time_ns)
{
    return sec_t (static_cast<double> (time_ns) / 1e9); 
}

/**
 * Seconds to microseconds
 */
static inline us_t sec_to_us (const sec_t time_s)
{
    return us_t (time_s * 1e6); 
}