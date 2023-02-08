//Sam Stewart
//ASEN Senior Design 
//Define methods declared in header file. This is the code that will contain the actual control law(s). 

using namespace std;

#include <iostream>
#include <cmath> 
#include <PD.hpp>

//initialize controller 
void PD_Controller::initController(ControllerSpecs *pd)
{
    //define controller gains (user defined)
    pd->KpT = 0.0f;
    pd->KdT = 0.0f;
    pd->KpR = 0.0f;
    pd->KdR = 0.0f;

    //define min and max voltage 
    pd->minVolts = -6.0f;
    pd->maxVolts = 6.0f;

    //define sample time 
    pd->T = 0.0f;

    //define time lowpass filter time constant 
    pd->tau = 0.0f;

    //define controller memory aspects
    pd->prevError = 0.0f;
    pd->differentiator = 0.0f;
    pd->prevMeasurement = 0.0f;

    //define outputs 
    pd->outLeft = 0.0f;
    pd->outRight = 0.0f;
}

//Check the raw voltage output calculated by the control law against the min/max rated voltages of the motor
void PD_Controller::checkOutput(ControllerSpecs *pd)
{
    //check left output
    if(pd->outLeft < pd->minVolts)
    {
        pd->outLeft = pd->minVolts;
    }
    else if(pd->outLeft > pd->maxVolts)
    {
        pd->outLeft = pd->maxVolts;
    };

    //check right output
    if(pd->outRight < pd->minVolts)
    {
        pd->outRight = pd->minVolts;
    }
    else if(pd->outRight > pd->maxVolts)
    {
        pd->outRight = pd->maxVolts;
    };
}

//WIP: given the current heading from the magnetometer and a desired heading, determine if the rover should turn left or right. 
//Could potentially rewrite using rotational data from IMU and retain same functionality 
int PD_Controller::leftRight(float current[], float desired[])
{
    int leftRight;
    leftRight = 0;
    return leftRight;
}

//update the controller with a new measurement and calculate output according to control law
//measurement will come from IMU/mag, and should be as simple as calling a function in caleb's code
void PD_Controller::updateController(ControllerSpecs *pd, float setpoint, float measurement, int mode)
{
    float Kp;
    float Kd;

    //determine if rover is translating or rotation
    if(mode == 0)
    {
        //rover is rotating. set respective gains
        Kp = pd->KpR;
        Kd = pd->KdR;
    }
    else
    {
        //rover is translating
        Kp = pd->KpT;
        Kd = pd->KdT;
    };

    //calculate the error between the setpoint (desired state) and the measurement (current state)
    float error = setpoint - measurement;

    //proportional control aspect
    float proportional = Kp*error;

    //derivative control aspect
    pd->differentiator = (2.0f * Kd * (measurement - pd->prevMeasurement)
                       + (2.0f * pd->tau - pd->T) * pd->differentiator)
                       / (2.0f * pd->tau + pd->T);


    //compute raw output 
    pd->outLeft = proportional + pd->differentiator;
    pd->outRight = proportional + pd->differentiator;

    //apply voltage limits
    checkOutput(pd);

    //store current measurement and error for next call
    pd->prevError = error;
    pd->prevMeasurement = measurement;

}

