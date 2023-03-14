#include "Arduino.h"
#include "PD.hpp"
#include "teensyComms.h"
#include "imuMag.h"
#include <math.h>

using namespace std;
 
// put your setup code here, to run once:
void setup() {


  //open serial communications
  Serial.begin(115200);

  Serial.println("Teensy setup...");

  //create controller object
  PD_Controller Controller;

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

}

// put your main code here, to run repeatedly:
void loop() {

  Serial.println("Teensy loop...");

  //create controller object
  PD_Controller Controller;

  //declare/construc comms object
  teensyComms comms = teensyComms();

  //instantiate imuMag object
  IMU_MAG imuMag; 

  //declare variables for imuMag
  int timestep = 1;
  float position;
  float velocity;
  float acceleration;
  float magX;
  float magY;
  float magZ;

  //declare variables used for feedback control
  float measurement = 0.0f;
  int pwmLeft;
  int pwmRight;

  //declare variables received from commands
  float magnitude=-1;
  char commandType='0';
  
  Serial.clear();

  //check for updated command from RASPI. update "mode"
  //
  //check for updated command
  while(true)
  {
    Serial.print("<3");
    //if there is something in the serial buffer, read it
    while(!Serial.available());

    if(Serial.available())
    {
      //read command
      comms.readCommand(&commandType, &magnitude);

      //reset imuMag state
      imuMag.reset();

      //print command
      Serial.print("\n LATEST RECEIVED COMMAND: ");
      Serial.print("\n Command Type: ");
      Serial.println(commandType);
      Serial.print("\n Magnitude: ");
      Serial.println(magnitude);
    };
  

    //collect current measurement of state from IMU/MAG
    //
    //update current position, velocity, and acceleration with integration
    imuMag.update_status(timestep);

    //read position, velocity, and acceleration
    position = imuMag.read_pos();
    velocity = imuMag.read_vel();
    acceleration = imuMag.read_acc();

    //read magnetometer heading
    magX = imuMag.mag_x();
    magY = imuMag.mag_y();
    magZ = imuMag.mag_z();

    //if current command is rotation, calculate current compass heading
    if(commandType == 'r')
    {
      //measurement = atan2(magY, magX) * (180/PI);
      Serial.println("Rotation command received and recognized!");
    };

    //if current command is translation, set position to measurement 
    if(commandType == 't')
    {
      measurement = position;
      Serial.println("Translation command received and recognized!");
    };

    //check for stop command (for automation subsystem test)
    if(commandType == 's')
    {
      Serial.println("Stop command received and recognized!");
    };

    //if desired, send magnetometer reading to RasPi
    //
    if(commandType == 'm')
    {
      //code to send magnetometer azimuth to RasPi
      measurement = atan2(magY, magX) * (180/PI);
      //Serial.write(measurement);
      Serial.println("Magnetometer command received and recognized!");
      //write magnetometer reading to serial for raspi
      
    };
    
    continue;
    //update controller to generate output 
    Controller.updateController(Controller.specs, magnitude, measurement, commandType);
    
    //convert voltage output to PWM duty cycle
    pwmLeft = (Controller.specs->outLeft / 6) * 256;
    pwmLeft = round(pwmLeft);
    pwmLeft = int(pwmLeft);

    pwmRight = (Controller.specs->outRight / 6) * 256;
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
    else
    {
      //stop motors
      analogWrite(0, 0);
      analogWrite(6, 0);
      Serial.println("ERROR: Unknown command type!");
    }

    //convert signals back to floats 
    pwmLeft = float(pwmLeft);
    pwmRight = float(pwmRight);

  };

}