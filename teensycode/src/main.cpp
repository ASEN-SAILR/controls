#include "Arduino.h"
#include "PD.hpp"
#include "teensyComms.h"
#include "imuMag.h"
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>
#include <vector> 
#include <SPI.h>  
#include <SD.h>

using namespace std;

//create IMU/mag object. global because it worked and we aren't gonna touch it again
IMU_MAG imuMag; 

//define output pins
const int pwmLeft = 0;
const int pwmRight = 1;
const int dirLeft = 2;
const int dirRight = 3;
const int slpLeft = 4;
const int slpRight = 5;

 
// put your setup code here, to run once:
void setup() {

  //open serial communications
  Serial.begin(115200);

  //Serial.println("Teensy setup...");

  //create controller object
  PD_Controller Controller = PD_Controller();

  //initialize input and output pins 
  pinMode(pwmLeft, OUTPUT); //left PWM output
  pinMode(pwmRight, OUTPUT); //right direction
  pinMode(dirLeft, OUTPUT); //right PWM
  pinMode(dirRight, OUTPUT); //right DIR
  pinMode(slpLeft, OUTPUT); //BI1 output
  pinMode(slpRight, OUTPUT); //BI2 output
  pinMode(LED_BUILTIN, OUTPUT); //led 

  //initialize imuMag breakout board
  imuMag.startup();

  //zero values for position, velocity, and acceleration, set acceleration offset
  imuMag.reset();

  //enable ESC by driving SLP pins to high 
  digitalWrite(slpLeft, HIGH);
  digitalWrite(slpRight, HIGH);

}

// put your main code here, to run repeatedly:
void loop() {
 
  //define controller struct
  struct
  {
    //controller gains
    float KpT = 40; //proportional gain for translation
    float KdT = .15; //derivative gain for translation
    float KpR = .6; //proportional gain for rotation
    float KdR = .15; //derivative gain for rotation

    //output limits (min and max voltage)
    float minVolts = -12; 
    float maxVolts = 12; 

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

  //create controller object
  PD_Controller Controller = PD_Controller(); 

  //declare/construc comms object
  teensyComms comms = teensyComms();

  //instantiate imuMag object
  imuMag.startup();
  imuMag.reset();

  //declare variables for imuMag
  float currentTime;
  float dt = 0;
  float startTime;
  int timestep = 1;
  float position;
  float velocity;
  float acceleration;
  float magX;
  float magY;
  float w;

  //declare further variables used for feedback control
  float measurement = 0.0f;
  int valLeft;
  int valRight;

  //declare variables received from commands
  float magnitude = -999;
  char commandType = '0';

  //declare vectors for storing test data
  vector<float> posVec;
  vector<float> velVec;
  vector<float> accVec;
  vector<float> rotVec;
  
  //Serial.clear();
  //Serial.print("Waiting for command");
  //wait to execute until first command is received

  //delay(20000);
  imuMag.reset();

  Serial.clear();

  //while(!Serial.available());

  while(true)
  {
    //update timestep
    currentTime = millis();
    if(dt)
    {
      dt = currentTime - startTime;

    }
    else
      dt = timestep;
    startTime = millis();
    Specs.T = dt;
    


    //check current state of rover against desired state. if within 5%, stop and wait for command
    if(measurement >= .975*magnitude && measurement <= 1.05*magnitude && commandType != 's')
    {
      commandType = 's';
      Serial.println("d");
      //imuMag.reset();
      delay(200);
      //Serial.clear();

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

      //blink LED (for testing)
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);

      delay(200);
    };
  

    //collect current measurement of state from IMU/MAG
    //
    //imuMag.reset();
    //update current position, velocity, and acceleration with integration
    imuMag.update_status(float(dt)/1000.0);
    //imuMag.update_status(.01);

    //read position, velocity, acceleration, and rotation data
    position = imuMag.read_pos();
    velocity = imuMag.read_vel();
    acceleration = imuMag.read_acc();
    w = -imuMag.read_w() * 180/PI;

    //write timestep data to file
    // if(time)
    // {
    //   time.println(dt);
    // }

    // Serial.print("Position: ");
    // Serial.print(position);
    // Serial.print(" Velocity: ");
    // Serial.print(velocity);
    // Serial.print(" Acceleration: ");
    // Serial.print(acceleration);
    // Serial.print("\n");

    //read magnetometer heading
    magX = imuMag.mag_x();
    magY = imuMag.mag_y();

    //print data 
    // Serial.print(position);
    // Serial.print(",  ");
    // Serial.print(velocity);
    // Serial.print(", ");
    // Serial.print(acceleration);
    // Serial.print("\n");
    

    //if current command is rotation, calculate current compass heading
    if(commandType == 'r')
    {
      measurement = w;
      //Serial.print(w);
      //Serial.print("\n");
      //Serial.print("Measurement: ");
      //Serial.print(w);
      //Serial.print("\n");
      //Serial.println("Rotation command received and recognized!");
    };

    //if current command is translation, set position to measurement 
    if(commandType == 't')
    {
      measurement = position;
      //Serial.println("Translation command received and recognized!");
      if(magnitude > 0)
      {
        digitalWrite(dirLeft, LOW);
        digitalWrite(dirRight, HIGH);
      }
      if(magnitude < 0)
      {
        digitalWrite(dirLeft, HIGH);
        digitalWrite(dirRight, LOW);
      }
      // digitalWrite(dirLeft, LOW);
      // digitalWrite(dirRight, HIGH);
      float time = abs(magnitude) / .24;
      time = round(time);
      time = int(time);
      analogWrite(pwmLeft, 256);
      analogWrite(pwmRight, 240);
      delay(time * 1000);
      commandType = 's';
      delay(200);
      Serial.println("d");
      
    };

    //check for stop command (for automation subsystem test)
    if(commandType == 's')
    {
      //Serial.println("Stop command received and recognized!");
      analogWrite(pwmLeft, 0);
      analogWrite(pwmRight, 0);
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

    //  Serial.print("Output: ");
    //  Serial.print(Specs.outLeft);
    //  Serial.print("\n");

    //check output of control law to determine if near destination 
    //this is an additional check to prevent IMU from freaking out due to stopping suddenly
    if(abs(Specs.outLeft) <= 2 || abs(Specs.outRight) <= 2)
    {
      commandType = 's';
      analogWrite(pwmLeft, 0);
      analogWrite(pwmRight, 0);
      // Serial.print("Voltage Check \n");
      Serial.println("d");
      delay(200);
      //Serial.clear();
    }
    
    //convert voltage output to PWM duty cycle
    valLeft = (Specs.outLeft / 6) * 256;
    valLeft = round(valLeft);
    valLeft = int(valLeft);

    valRight = (Specs.outRight / 6) * 256;
    valRight = round(valRight);
    valRight = int(valRight);

    //send output to ESCs through pins. if turning, drive motors in necessary directions
    if(commandType == 't' && magnitude < 0)
    {
      //drive logical pins to put motors in reverse
      digitalWrite(dirLeft, HIGH);
      digitalWrite(dirRight, LOW);

      //send PWM signal
      analogWrite(pwmLeft, valLeft);
      analogWrite(pwmRight, valRight);
    }
    else if(commandType == 't' && magnitude > 0)
    {
      //drive logical pins to drive motors forwards
      digitalWrite(dirLeft, LOW);
      digitalWrite(dirRight, HIGH);

      //send PWM signal
      analogWrite(pwmLeft, valLeft);
      analogWrite(pwmRight, valRight);
    }

    if(commandType == 'r' && magnitude < 0)
    {
      //drive logical pins to turn rover left
      digitalWrite(dirLeft, HIGH);
      digitalWrite(dirRight, HIGH);

      //send PWM signal
      analogWrite(pwmLeft, valLeft);
      analogWrite(pwmRight, valRight);

      //send PWM signal 
    }
    else if(commandType == 'r' && magnitude > 0)
    {
      //drive logical pins to turn rover right
      digitalWrite(dirLeft, LOW);
      digitalWrite(dirRight, LOW);

      //send PWM signal
      analogWrite(pwmLeft, valLeft);
      analogWrite(pwmRight, valRight);
    }

    //convert signals back to floats 
    valLeft = float(valLeft);
    valRight = float(valRight);

    delay(10);

  };

}