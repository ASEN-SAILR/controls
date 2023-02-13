#include "teensyComms.h"
bool teensyComms::readCommand(char* command_type, float* magnitude){
    // char buffer[32];
    uint8_t buffer[] = {0x55, 0x66, 0x66, 0x86, 0x40};
    

    int msg_len = Serial.available();

    Serial.print("read ");
    for(int i=0; i<msg_len; i++){
        buffer[i] = Serial.read();
        Serial.println(buffer[i]);
    }

    Serial.print("Read in: ");
    
    // float g = -4.2f;
    // memcpy(&buffer[1], &g, 4);    // receive data    

    for(int i=1; i<5; i++){
        Serial.println(buffer[i],HEX);
    }

    float f;
    memcpy(&f, &buffer[1], 4);    // receive data

    Serial.print("Float: ");
    Serial.println(f);

    // // Should probably have this read to a new line?
    // float mag_buffer[] = {0,0,0,0,0,0,0,0};
    // float magnitude_raw = 0;
    // float* ptr = &magnitude_raw;
    
    // *command_type = char(Serial.read());

    // int mag_length = Serial.available();
    
    // //read in bytes of data

    // *magnitude = float(magnitude_raw); 
    // Serial.println(*magnitude);
    Serial.clear();
    return true;
}
