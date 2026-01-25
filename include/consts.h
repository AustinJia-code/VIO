/**
 * @file consts.h
 * @brief Constants
 */

#pragma once

/*** GENERAL ***/
#define G                   9.80665

/*** ICM-20948 ***/
#define I2C_PATH            "/dev/i2c-1"
#define I2C_SLAVE	        0x0703
#define ICM_ADDR            0x68
#define REG_BANK_SEL        0x7F
#define REG_PWR_MGMT_1      0x06
#define REG_ACCEL_XOUT_H    0x2D
#define GYRO_SCALE          32.8     // For +/- 1000 dps
#define ACCEL_SCALE         16384.0  // For +/- 2g

/*** STEREO ***/
#define CALIB_DIR           "../data/calib/checker-35mm-9-7/"
#define CALIB_PATH          "../data/calib/stereo_calib.yml"
#define CALIB_W             9       // Inner corners
#define CALIB_H             7
#define CALIB_MM            35

#define W_PX                640
#define H_PX                480
#define LEFT_CAM_ID         0
#define RIGHT_CAM_ID        1

/*** MATCHER ***/
#define FEATURE_N           500
#define Y_TOL_PX            2
#define MIN_MATCHES         8