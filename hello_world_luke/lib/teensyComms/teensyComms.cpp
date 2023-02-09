#include "teensyComms.h"
bool teensyComms::readCommand(char* command_type, float* magnitude){
    // Should probably have this read to a new line?
    float mag_buffer[] = {0,0,0,0,0,0,0,0};
    int magnitude_raw = 0;
    
    *command_type = char(Serial.read());

    int mag_length = Serial.available();
    
    //read in bytes of data
    for(int i=0; i<mag_length; i++){
        magnitude_raw = magnitude_raw | Serial.read()<<i;
    }

    *magnitude = float(magnitude_raw); 

    Serial.clear();
    return true;
}
