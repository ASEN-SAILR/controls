#include "teensyComms.h"
bool teensyComms::readCommand(char* command_type, float* magnitude){
    // Should probably have this read to a new line?
    char mag_buffer[] = {0,0,0,0,0,0,0,0};
    
    *command_type = char(Serial.read());

    int mag_length = Serial.available();
    
    //read in bytes of data
    for(int i=0; i<mag_length; i++){
        mag_buffer[i] = Serial.read();
    }

    String my_str = mag_buffer;

    
    *magnitude = my_str.toFloat();
    Serial.clear();
    return true;
}
