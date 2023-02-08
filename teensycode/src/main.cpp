#include <Arduino.h>
#include <PD.hpp>


using namespace std;

//create controller object
PD_Controller Controller;

  //define variables used for feedback control
  float setpoint = 0.0f;
  float measurement = 0.0f;
  int mode = 0;
  float pwmLeft;
  float pwmRight;
  



void setup() {
  // put your setup code here, to run once:

  //open serial communications
  Serial.begin(9600);

  //initialize input and output pins 
  pinMode(0, OUTPUT); //output for left motor
  pinMode(1, OUTPUT); //output for right motor

 //initialize controller with desired specs
 Controller.initController(Controller.specs);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //check for updated command from RASPI. update "mode"
  //
 

  //collect current measurement of state from IMU/MAG
  //

  

  //update controller to generate output 
  Controller.updateController(Controller.specs, setpoint, measurement, mode);

  //convert voltage output to PWM 
  pwmLeft = (Controller.specs->outLeft / 6) * 256;
  pwmLeft = round(pwmLeft);
  pwmLeft = int(pwmLeft);

  pwmRight = (Controller.specs->outRight / 6) * 256;
  pwmRight = round(pwmRight);
  pwmRight = int(pwmRight);

  //send output to ESCs through pins
  analogWrite(0, pwmLeft); //pwm signal to left motor
  analogWrite(1, pwmRight); //pwm signal to right motor 

  //convert signals back to floats 
  pwmLeft = float(pwmLeft);
  pwmRight = float(pwmRight);

}