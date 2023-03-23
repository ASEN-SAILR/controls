#include "imuMag.h"
IMU_MAG var;

void setup() {
  var.startup();

  var.reset();
}

void loop() {
  float var_x, var_dx, var_ddx, var_w, var_dw;
  float var_mag[3];

  var.update_status(0.01);

  var_x = var.read_pos();
  var_dx = var.read_vel();
  var_ddx = var.read_acc();


  var_mag[0] = var.mag_x();
  var_mag[1] = var.mag_y();
  var_mag[2] = var.mag_z();

  var_w = var.read_w();
  var_dw = var.read_dw();

  Serial.print("\nPosition: "); Serial.print(var_x);
  Serial.print(" \nVelocity: "); Serial.print(var_dx);
  Serial.print(" \nAcceleration: "); Serial.print(var_ddx);
  Serial.println();
  Serial.print("\nX: "); Serial.print(var_mag[0]);
  Serial.print(" \nY: "); Serial.print(var_mag[1]);
  Serial.print(" \nZ: ");Serial.print(var_mag[2]);
  Serial.println();
  Serial.print("\nRotation: "); Serial.print(var_w);
  Serial.print(" \nRotational Rate: "); Serial.print(var_dw);
  Serial.println();



  delayMicroseconds(10000);
}
