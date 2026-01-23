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
inline static ns_t get_time_ns ()
{
    auto now = std::chrono::steady_clock::now ();
    return ns_t {std::chrono::duration_cast<std::chrono::nanoseconds> (
                    now.time_since_epoch ()).count ()};
}