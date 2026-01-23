/**
 * @file ekf.hpp
 * @brief Combines two sensors using an EKF
 */

#pragma once

#include "def.h"

/**
 * Combines two vec3_t
 */
class EKF
{
private:
    vec3_t estimate;

public:
    /**
     * Update estimate with fast, noisy prediction
     */
    void predict (vec3_t val);

    /**
     * Update estimate with slow, "accurate" measurement
     */
    void update (vec3_t val);
};