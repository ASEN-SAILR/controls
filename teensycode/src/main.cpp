#include "Arduino.h"
#include "PD.hpp"
#include "teensyComms.h"
#include "imuMag.h"
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>

using namespace std;
 
// put your setup code here, to run once:
void setup() {

  //open serial communications
  Serial.begin(115200);

  //Serial.println("Teensy setup...");

  //create controller object
  PD_Controller Controller = PD_Controller();

  //instantiate imuMag object
  IMU_MAG imuMag; 

  //initialize input and output pins 
  pinMode(0, OUTPUT); //PWM output for left motor
  pinMode(1, OUTPUT); //output for AIN1
  pinMode(2, OUTPUT); //output for AIN2
  pinMode(3, OUTPUT); //output for STANDBY
  pinMode(4, OUTPUT); //output for BIN1
  pinMode(5, OUTPUT); //output for BIN2
  pinMode(6, OUTPUT); //PWM output for right motor

  //initialize controller with desired specs
  Controller.initController(Controller.specs);

  //initialize imuMag breakout board
  imuMag.startup();

  //zero values for position, velocity, and acceleration, set acceleration offset
  imuMag.reset();

  //enable ESC by driving STANDBY pin to high 
  digitalWrite(3, HIGH);

  //Serial.print("\n");
  //Serial.print("Setup complete... \n");

}

// put your main code here, to run repeatedly:
void loop() {

 
  //define controller struct
  struct
  {
    //controller gains
    float KpT = 10; //proportional gain for translation
    float KdT = .15; //derivative gain for translation
    float KpR = 0; //proportional gain for rotation
    float KdR = 0; //derivative gain for rotation

    //output limits (min and max voltage)
    float minVolts = -6; 
    float maxVolts = 6; 

    //sample time 
    float T = .01; //[seconds]

    //derivative low pass filter time constant 
    float tau = .2; //[s^-1]

    //controller memory
    float prevError; //previous error measurement
    float differentiator = 0; //required for derivative control
    float prevMeasurement = 0; //previous measurement from IMU/mag

    //controller outputs
    float outLeft; //[V]
    float outRight; //[V]
  } Specs;

  Serial.print("Struct created \n");
  Serial.print(Specs.KdR);

  //create controller object
  PD_Controller Controller = PD_Controller(); 

  //declare/construc comms object
  teensyComms comms = teensyComms();

  //instantiate imuMag object
  IMU_MAG imuMag; 
  imuMag.startup();
  imuMag.reset();

  //declare variables for imuMag
  float currentTime;
  float dt = 0;
  float startTime;
  int timestep;
  float position;
  float velocity;
  float acceleration;
  float magX;
  float magY;
  float w;

  //declare further variables used for feedback control
  float measurement = 0.0f;
  int pwmLeft;
  int pwmRight;

  //declare variables received from commands
  float magnitude = 100;
  char commandType = 't';
  
  //Serial.clear();
  Serial.print("Waiting for command");
  //wait to execute until first command is received
  //while(!Serial.available());

  while(true)
  {
    //update timestep
    currentTime = millis();
    if(!dt)
    {
      dt = timestep;
    }
    else
      dt = currentTime - startTime;
    startTime = millis();
    Specs.T = dt;

    //check current state of rover against desired state. if within 5%, stop and wait for command
    if(measurement >= .95*magnitude && measurement <= 1.05*magnitude)
    {
      commandType = 's';
      imuMag.reset();
    };

    //if there is a new command, read it and overwrite current command 
    if(Serial.available())
    {
      //read command
      comms.readCommand(&commandType, &magnitude);

      //reset imuMag state
      imuMag.reset();

      //reset measurement and controller memory
      measurement = 0;
      Specs.prevError = 0;
      Specs.prevMeasurement = 0;
    };
  

    //collect current measurement of state from IMU/MAG
    //
    //update current position, velocity, and acceleration with integration
    imuMag.update_status(dt);

    //read position, velocity, and acceleration
    position = imuMag.read_pos();
    velocity = imuMag.read_vel();
    acceleration = imuMag.read_acc();
    w = imuMag.read_w();

    //read magnetometer heading
    magX = imuMag.mag_x();
    magY = imuMag.mag_y();

    //if current command is rotation, calculate current compass heading
    if(commandType == 'r')
    {
      measurement = w;
      Serial.print("Measurement: ");
      Serial.print(w);
      Serial.print("\n");
      //Serial.println("Rotation command received and recognized!");
    };

    //if current command is translation, set position to measurement 
    if(commandType == 't')
    {
      measurement = position;
      Serial.print("Measurement: ");
      Serial.print(position);
      Serial.print("\n");
      //Serial.println("Translation command received and recognized!");
    };

    //check for stop command (for automation subsystem test)
    if(commandType == 's')
    {
      //Serial.println("Stop command received and recognized!");
      analogWrite(0, 0);
      analogWrite(6, 0);
    };

    //if desired, send magnetometer reading to RasPi
    //
    if(commandType == 'm')
    {
      //code to send magnetometer azimuth to RasPi
      measurement = atan2(magY, magX) * (180/PI);
      Serial.print('m');
      Serial.println(measurement);
      commandType = 's';
    };
    
    //continue;
    //update controller to generate output 
    //Controller.updateController(Controller.specs, magnitude, measurement, commandType);
    float Kp = 0;
    float Kd = 0;
    if(commandType == 'r')
    {
      Kp = Specs.KpR;
      Kd = Specs.KdR;
    }
    else if(commandType == 't')
    {
      Kp = Specs.KpT;
      Kd = Specs.KdT;
    }
    else if(commandType == 's')
    {
      Specs.outLeft = 0;
      Specs.outRight = 0;
    };

    //calculate error between current and desired states
    float error = magnitude - measurement;

    //proportional control
    float proportional = Kp*error;

    //derivative control aspect
    Specs.differentiator = (2.0f * Kd * (measurement - Specs.prevMeasurement)
                         + (2.0f * Specs.tau - Specs.T) * Specs.differentiator)
                         / (2.0f * Specs.tau + Specs.T);

    //compute output in voltage
    Specs.outLeft = proportional + Specs.differentiator;
    Specs.outRight = proportional + Specs.differentiator;

    //check output 
    Specs.outLeft = Controller.checkOutput2(Specs.outLeft);
    Specs.outRight = Controller.checkOutput2(Specs.outRight);

    //update controller memory
    Specs.prevError = error;
    Specs.prevMeasurement = measurement; 

    Serial.print("Output: ");
    Serial.print(Specs.outLeft);
    Serial.print("\n");

    
    //convert voltage output to PWM duty cycle
    pwmLeft = (Specs.outLeft / 6) * 256;
    pwmLeft = round(pwmLeft);
    pwmLeft = int(pwmLeft);

    pwmRight = (Specs.outRight / 6) * 256;
    pwmRight = round(pwmRight);
    pwmRight = int(pwmRight);

    //send output to ESCs through pins. if turning, drive motors in necessary directions
    if(commandType == 't' && magnitude < 0)
    {
      //drive logical pins to put motors in reverse

      //send PWM signal
    }
    else if(commandType == 't' && magnitude > 0)
    {
      //drive logical pins to drive motors forwards

      //send PWM signal
    }

    if(commandType == 'r' && magnitude < 0)
    {
      //drive logical pins to turn rover left

      //send PWM signal 
    }
    else if(commandType == 'r' && magnitude > 0)
    {
      //drive logical pins to turn rover right

      //send PWM signal
    }
    // else
    // {
    //   //stop motors
    //   analogWrite(0, 0);
    //   analogWrite(6, 0);
    //   Serial.println("ERROR: Unknown command type!");
    // }

    //convert signals back to floats 
    pwmLeft = float(pwmLeft);
    pwmRight = float(pwmRight);

  };

}