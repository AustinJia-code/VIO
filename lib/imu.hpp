/**
 * @file imu.hpp
 * @brief Raw IMU interface
 */

#pragma once

#include "defs.h"
#include "consts.h"
#include <chrono>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdexcept>
#include <math.h>

/**
 * IMU odometer interface
 */
class IMU
{
private:
    int fd;

    /**
     * Write one byte over I2C
     */
    void write_byte (byte_t reg, byte_t val)
    { 
        if (ioctl (fd, I2C_SLAVE, ICM_ADDR) < 0)
            throw std::runtime_error ("Failed to acquire bus access");

        uint8_t buf[2] = {reg, val};
        
        if (write (fd, buf, 2) != 2)
            throw std::runtime_error ("I2C write incomplete");
    }

public:
    /**
     * Constructor, try to open I2C
     */
    IMU ()
    {
        if ((fd = open (IMU_PATH, O_RDWR)) < 0)
            throw std::runtime_error ("Could not open I2C bus");
        
        // Wake up sensor, set bank 0
        write_byte (REG_BANK_SEL, 0x00);
        write_byte (REG_PWR_MGMT_1, 0x01);

        sleep (sec_t {0.1});
    }

    /**
     * Read a set of IMU data into out
     */
    void read (IMUData& out)
    {
        if (ioctl (fd, I2C_SLAVE, ICM_ADDR) < 0)
            throw std::runtime_error ("Failed to acquire bus access");

        byte_t reg = REG_ACCEL_XOUT_H;
        byte_t data[12]; // 3 axes accel + 3 axes gyro (2 bytes each)

        // Burst read start
        if (write (fd, &reg, 1) != 1)
            throw std::runtime_error ("Incomplete IMU write");

        if (::read (fd, data, 12) != 12)
            throw std::runtime_error ("Incomplete IMU read"); 

        auto tse = std::chrono::steady_clock::now ().time_since_epoch ();
        auto tse_ns = std::chrono::duration_cast<std::chrono::nanoseconds> (tse);
        out.time_ns = tse_ns.count ();

        // Convert bytes to data_t
        auto combine = [] (byte_t high, byte_t low)
        {
            return static_cast<data_t> ((high << 8) | low);
        };

        // Accelerometer (Convert to m/s^2)
        out.accel.x () = (combine (data[0], data[1]) / ACCEL_SCALE) * G;
        out.accel.y () = (combine (data[2], data[3]) / ACCEL_SCALE) * G;
        out.accel.z () = (combine (data[4], data[5]) / ACCEL_SCALE) * G;

        // Gyroscope (Convert to rad/s)
        out.gyro.x () = (combine (data[6], data[7]) / GYRO_SCALE) * (M_PI / 180.0);
        out.gyro.y () = (combine (data[8], data[9]) / GYRO_SCALE) * (M_PI / 180.0);
        out.gyro.z () = (combine (data[10], data[11]) / GYRO_SCALE) * (M_PI / 180.0);
    }
};