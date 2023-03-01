#include "imuMag.h"
IMU_MAG var;

void setup() {
  var.startup();

  var.reset();
}

void loop() {
  float var_x;
  float var_mag[3];

  var.update_status(0.01);

  var_x = var.read_pos();

  var_mag[0] = var.mag_x();
  var_mag[1] = var.mag_y();
  var_mag[2] = var.mag_z();

  Serial.print(var_x);
  Serial.println();
  Serial.print(var_mag[0]);
  Serial.print(var_mag[1]);
  Serial.print(var_mag[2]);
  Serial.println();



  delayMicroseconds(10000);
}
