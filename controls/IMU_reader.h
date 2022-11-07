// For IMU
#include <Adafruit_LSM6DS3TRC.h>

// For Magnetometer
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

#include <math.h>

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

#define LIS3MDL_CLK 13
#define LIS3MDL_MISO 12
#define LIS3MDL_MOSI 11
#define LIS3MDL_CS 10

class IMU_reader {
  public:
    // Construcor
    IMU_reader(void);    

    float* read_IMU(int);

  private: 
    // Variables for Data Collection
    float x_imu=0;
    float y_imu=0;
    float z_imu=0;
    float dx_imu=0;
    float dy_imu=0;
    float dz_imu=0;
    float l_imu=0;
    float m_imu=0;
    float n_imu=0;

    // Variables for Integration
    float ddx_imu=0;
    float ddy_imu=0;
    float ddz_imu=0;
    float dx_imu_2=0;
    float dy_imu_2=0;
    float dz_imu_2=0;
    float dl_imu=0;
    float dm_imu=0;
    float dn_imu=0;
    unsigned int data_collected=0;

    // Function to run numeric integration utilizing trapezoidal rule
    float integrate(float,float,int);

    float integrate_simp(float,float,float,int);
};

class Magnetometer_reader {
  public:
    Magnetometer_reader(void);

    float* read_Magnetometer(void);
  private:

};

// Class to initialize Magnetometer
class init_Magnetometer {
  public:
    // Constructor
    init_Magnetometer(void);

  private:
    void startup(void);
};

// Class to initialize IMU
class init_IMU {
  public:
    // Constructor
    init_IMU(void);

  private:
    void startup(void);
};