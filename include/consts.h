/**
 * @file consts.h
 * @brief Constants
 */

#pragma once

/*** GENERAL ***/
#define G                   9.80665

/*** ICM-20948 ***/
#define IMU_PATH            "/dev/i2c-1"
#define O_RDWR              02
#define I2C_SLAVE	        0x0703
#define ICM_ADDR            0x68
#define REG_BANK_SEL        0x7F
#define REG_PWR_MGMT_1      0x06
#define REG_ACCEL_XOUT_H    0x2D
#define GYRO_SCALE          32.8     // For +/- 1000 dps
#define ACCEL_SCALE         16384.0  // For +/- 2g