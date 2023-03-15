//Sam Stewart 
//ASEN Senior Design 
//Header file for PD control law script. Initialize gain variables, variables for controller memory, etc

#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

struct ControllerSpecs{

    //controller gains
    float KpT; //proportional gain for translation
    float KdT; //derivative gain for translation
    float KpR; //proportional gain for rotation
    float KdR; //derivative gain for rotation

    //output limits (min and max voltage)
    float minVolts; 
    float maxVolts; 

    //sample time 
    float T; //[seconds]

    //derivative low pass filter time constant 
    float tau; //[s^-1]

    //controller memory
    float prevError; //previous error measurement
    float differentiator; //required for derivative control
    float prevMeasurement; //previous measurement from IMU/mag

    //controller outputs
    float outLeft; //[V]
    float outRight; //[V]

}; 

class PD_Controller{

    public: 
        //PD_Controller(void); //constructor
        void initController(ControllerSpecs *pd); //initialize controller
        void updateController(ControllerSpecs *pd, float setpoint, float measurement, char mode); //update controller 
        struct ControllerSpecs* specs;
    
    private: 
        void checkOutput(ControllerSpecs *pd); //check to see if controller output is in acceptable voltage range
        int leftRight(float current[], float desired[]); //determine direction to turn rover based on current and desired heading
};

#endif 
