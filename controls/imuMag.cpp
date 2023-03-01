#include "imuMag.h"

// Constructor
IMU_MAG::IMU_MAG(){
    return;
}

// Initialize Mag and IMU
void IMU_MAG::startup(){
  /*_______________________________________________________
  LIS3MDL 
  */

  if(!Serial){
    Serial.begin(115200);
    while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("Adafruit LIS3MDL test!");
  
  // Try to initialize!
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");


  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);   //LOWPOWERMODE, MEDIUMMODE, HIGHMODE, ULTRAHIGHMODE
  Serial.print("LIS3MDL Performance mode set");

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);     //CONTINUOUSMODE, SINGLEMODE, POWERDOWNMODE
  Serial.print("LIS3MDL Operation mode set");

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);     //0_625_HZ, 1_25_HZ ... 155_HZ
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("LIS3MDL Data rate set");
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);     //4_GUASS, 8_GAUSS, 12_GAUSS, 16_GAUSS
  Serial.print("LIS3MDL Range set");

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

  /*____________________________________________________
  LSM6D3STR
  */

  Serial.println("Adafruit LSM6DS3TR-C test!");

  if (!lsm6ds3trc.begin_I2C()) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS3TR-C Found!");

  // lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);  //2_G, 4_G, 8_G, 16_G
  Serial.print("LSM6DS3TRC Accelerometer range set");

  // lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);    //125, 250, 500, 1000, 2000, 4000
  Serial.print("LSM6DS3TRC Gyro range set");

  // lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_12_5_HZ);  //0, 12_5, 26, 52, 104, 208, 416, 833, 1_66K, 3_33K, 6_66K
  Serial.print("LSM6DS3TRC Accelerometer data rate set");

  // lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_12_5_HZ);   //"
  Serial.print("LSM6DS3TRC Gyro data rate set");

  lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2

  return;
}

// Reset Position, Velocity, Acceleration Data, as well as IMU offset
void IMU_MAG::reset(){
    // Zero Out 
    x = 0;
    dx = 0;
    ddx = 0;

    // Update Offset
    // Get a new normalized sensor event
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);

    ddx_offset = accel.acceleration.x;

    return;
}

void IMU_MAG::update_status(float timestep){
    // Local Variables
    float dx_2, ddx_2;

    // Get a new normalized sensor event
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);

    //Acceleration
    ddx_2 = ddx;
    ddx = accel.acceleration.x;
    
    //Velocity
    dx_2 = dx;
    dx += 0.5 * timestep * (ddx + ddx_2);

    //Position
    x += 0.5 * timestep * (dx + dx_2);

    // Update Magnetometer
    lis3mdl.read();

    return; 
}

// Return Position
float IMU_MAG::read_pos(){
    return x;
}

// Return Velocity
float IMU_MAG::read_vel(){
    return dx;
}

// Return Acceleration
float IMU_MAG::read_acc(){
    return ddx;
}

float IMU_MAG::mag_x(){
    return lis3mdl.x;
}

float IMU_MAG::mag_y(){
    return lis3mdl.y;
}

float IMU_MAG::mag_z(){
    return lis3mdl.z;
}