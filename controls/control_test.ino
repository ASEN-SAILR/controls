#include"IMU_reader.h"

// Class variables
IMU_reader IMU;
Magnetometer_reader Magnetometer;

int mode = 0; // 0 for IMU testing, 1 for Magnetometer Testing

int timestep = 100;  //[ms]

void setup(void) {
  // Initialize IMU and Magnetometer
  init_IMU IMU_initializer;
  init_Magnetometer Mag_initializer;

  IMU_initializer.init_IMU();
  Mag_initializer.init_Magnetometer();
}

void loop(void) {
  // Data collection variable
  float* vec;
  // IMU Mode
  if(mode)
  {
    // Read in IMU position data
    vec = IMU.read_IMU(timestep);

    // Print values to terminal
    Serial.print(vec[0]);
  }
  // Magnetometer Mode
  else
  {
    // Read in Magnetometer data
    vec = Magnetometer.read_Magnetometer();

    // Print Data
    Serial.print(vec[0]);
  }

  // Time delay
  delay(timestep);
}