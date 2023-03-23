// For IMU
#include <Adafruit_LSM6DS3TRC.h>

// For Magnetometer
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

#ifndef IMU_MAG_H
#define IMU_MAG_H

class IMU_MAG 
{
  private:
    float ddx, dx, x, ddx_offset, m_x, m_y, m_z, dw, dw_2, w, dw_offset, mx_off, my_off, mz_off;
    float mx_max = -47.76;
    float mx_min = -67.76;
    float my_max = 57.16;
    float my_min = 37.16;
    float mz_max = -38.65;
    float mz_min = -58.65;
    Adafruit_LSM6DS3TRC lsm6ds3trc;
    Adafruit_LIS3MDL lis3mdl;

  public:
    // Constructor
    IMU_MAG(void);

    void startup(void);

    void reset(void);

    void update_status(float);

    float read_pos(void);

    float read_vel(void);

    float read_acc(void);

    float read_w(void);

    float read_dw(void);

    float mag_x(void);

    float mag_y(void);

    float mag_z(void);
};

#endif