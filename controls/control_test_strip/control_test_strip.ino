/* Instructions for IMU/Magnetometer Use 
-----------------------------------------
- include "IMU_reader_strip.h" in header
- initialize an object of class IMU_reader for IMU, Magnetometer_reader for Magnetometer
- recommend using accurate clock for time step (delta t between loop calls)
- call read_Magnetometer() or read_IMU(time step)
- profit
-----------------------------------------
Extra Notes:
- call IMU_reader::reset() between each control command
  - This will reset the variables used to integrate position, and the position itself
*/

#include"IMU_reader_strip.h"

// Class variables
IMU_reader IMU;
Magnetometer_reader Magnetometer;

int mode = 0; // 0 for IMU testing, 1 for Magnetometer Testing

int timestep = 100;  //[ms]

// More accurate delta t from system clock
float dt = 0;
unsigned long currentTime;
unsigned long startTime;

void setup(void) {
  // Initialize IMU and Magnetometer
  IMU.startup();
  Magnetometer.startup();
}

void loop(void) {
  // Data collection variable
  float* vec;

  // Time change adjustments
  currentTime = millis();
  if(!dt)
    dt = timestep;
  else
    dt = currentTime - startTime;
  startTime = millis();

  // IMU Mode
  if(mode)
  {
    // Read in IMU position data
    vec = IMU.read_IMU(dt);

    // Print values to terminal
    if(!vec[0]) // Conditional for return 0
    {
      Serial.print("IMU x location [m]: ");
      Serial.print(vec[0]);
      Serial.print("\n");
    }
  }
  // Magnetometer Mode
  else
  {
    // Read in Magnetometer data
    vec = Magnetometer.read_Magnetometer();

    // Print Data
    Serial.print("Magnetometer [x,y,z] vector: [");
    Serial.print(vec[0]);
    Serial.print(vec[1]);
    Serial.print(vec[2]);
    Serial.print("]\n");
  }

  // Time delay
  delay(timestep);
}