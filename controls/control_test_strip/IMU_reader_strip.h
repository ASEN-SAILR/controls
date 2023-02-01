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

    // Read IMU (Called Every Timestep)
    float* read_IMU(int);

    // Initialize IMU (Run on Arduino Startup)
    void startup(void);

  private: 
    // Variables for Data Collection
    float x_imu=0;
    float dx_imu=0;

    // Variables for Integration
    float ddx_imu=0;
    float dx_imu_2=0;
    unsigned int data_collected=0;

    // Function to run numeric integration utilizing trapezoidal rule
    float integrate(float,float,int);

    float integrate_simp(float,float,float,int);

    void reset(void);
};

class Magnetometer_reader {
  public:
    // Constructor
    Magnetometer_reader(void);

    // Read Magnetometer (Called Every Timestep)
    float* read_Magnetometer(void);

    // Initialize Magnetometer (Call on Arduino Startup)
    void startup(void);
  private:

};

