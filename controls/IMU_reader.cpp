#include"IMU_reader.h"

// Create Sensor Objects
Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DS3TRC lsm6ds3trc;

/*_____________________________________________________________________________________*/

// Constructor
IMU_reader::IMU_reader() {}

/* Read in IMU values, and respective integrations
  input: time_step between arduino calls
  output: state vector in form [x,y,z,dx,dy,dz,ddx,ddy,ddz,l,m,n,dl,dm,dn]
*/
float* IMU_reader::read_IMU(int time_step) {
  // Initialize Return Value
  float imu_state[15];  

  // Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds3trc.getEvent(&accel, &gyro, &temp);

  // TODO: Add Sensor Zeroing at Rest

  // TODO: Integrate Sensor Readings
  // Integrate Gyro Data
  if (data_collected > 0) {
    // L
    l_imu += integrate(dl_imu,gyro.gyro.x,time_step);
    imu_state[9] = l_imu;
    dl_imu = imu_state[12] = gyro.gyro.x;
    // M
    m_imu += integrate(dm_imu,gyro.gyro.y,time_step);
    imu_state[10] = m_imu;
    dm_imu = imu_state[13] = gyro.gyro.y;
    // N
    n_imu += integrate(dn_imu,gyro.gyro.z,time_step);
    imu_state[11] = n_imu;
    dn_imu = imu_state[14] = gyro.gyro.z;    
  }
  else {
    dl_imu = gyro.gyro.x;
    dm_imu = gyro.gyro.y;
    dn_imu = gyro.gyro.z;
  }

  // Integrate Accelerometer Data
  // Velocity
  if (data_collected > 0) {
    // dx
    dx_imu += integrate(ddx_imu,accel.acceleration.x,time_step);
    ddx_imu = imu_state[6] = accel.acceleration.x;    
    // dy
    dy_imu += integrate(ddy_imu,accel.acceleration.y,time_step);
    ddy_imu = imu_state[7] = accel.acceleration.y;
    // dz
    dz_imu += integrate(ddz_imu,accel.acceleration.z,time_step);
    ddz_imu = imu_state[8] = accel.acceleration.z;
  }
  else { // Collect Initial Data on Startup
    ddx_imu = accel.acceleration.x;
    ddy_imu = accel.acceleration.y;
    ddz_imu = accel.acceleration.z;
    data_collected += 1;
  }
  // Position
  if (data_collected > 1) {
    // x
    x_imu += integrate(dx_imu_2,dx_imu,time_step);
    imu_state[0] = x_imu;
    dx_imu_2 = imu_state[3] = dx_imu;
    // y
    y_imu += integrate(dy_imu_2,dy_imu,time_step);
    imu_state[1] = y_imu;
    dy_imu_2 = imu_state[4] = dy_imu;
    // z
    z_imu += integrate(dz_imu_2,dz_imu,time_step);
    imu_state[2] = z_imu;
    dz_imu_2 = imu_state[5] = dz_imu;
  }
  else if (data_collected > 0){ // Collect Initial Velocity Data
    dx_imu_2 = dx_imu;
    dy_imu_2 = dy_imu;
    dz_imu_2 = dz_imu;
    data_collected += 1;
  }

  if (data_collected > 1){
    return imu_state;
  }
  return 0;
}

/* For use in integrating IMU
  input: two values, integer time step (ms)
  output: numeric integration between two points
  Utilizes trapezoidal rule
*/
float IMU_reader::integrate(float x_a, float x_b, int t_delta) {
  // Return Value
  float val;  
  // Convert timestep to s
  float t;
  t = float(t_delta) / 1000;
  // Calculate & Return  
  val  = 0.5 * (x_a+x_b) * t;
  return val;
}

/* More accurate integration utilizing Simpson's Rule
  input: three floats, x_b being intermediate value, integer time step (ms)
  output: numeric integration between three points
  Requires three inputs, increases memory usage
*/
/*
float IMU_reader::integrate_simp(float x_a, float x_b, float x_c, int t_delta){
  float val;
  val = t_delta/3 * (x_a + 4*x_b + x_c);
  return val;
}
*/

/*_____________________________________________________________________________________*/

// Constructor
Magnetometer_reader::Magnetometer_reader() {}

/* Read in Magnetometer
  input: void
  return: [float,float,float] == [x,y,z] of magnetic field
  Implied that return value points north
*/
float* Magnetometer_reader::read_Magnetometer() {
  float north_vec[3];  // Return value (to be assigned)
  float norm;          // For normalizing the vector
  lis3mdl.read();      // get X Y and Z data at once

  // Normalize then return vector
  norm = sqrt(lis3mdl.x^2 + lis3mdl.y^2 + lis3mdl.z^2);
  north_vec[0] = lis3mdl.x/norm;
  north_vec[1] = lis3mdl.y/norm;
  north_vec[2] = lis3mdl.z/norm;

  // Return [x,y,z] of magnetometer readings
  return north_vec;
}

/*_____________________________________________________________________________________*/

// Initialize Magnetometer (Run on Arduino Startup)
void Magnetometer_reader::startup() {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LIS3MDL test!");
  
  // Try to initialize!
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
}

/*_____________________________________________________________________________________*/

// Initialize IMU (run on arduino startup)
void IMU_reader::startup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

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

  // Set Accelerometer Range to 2G
  lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  // Verify Settings in Serial
  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds3trc.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  /*
  lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (lsm6ds3trc.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DS33
  }
  */

  // Set Accelerometer Data Rate to 12.5 Hz  
  lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  // Verify Settings in Serial
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds3trc.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  /*
  // lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds3trc.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
  */

  lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2  
}