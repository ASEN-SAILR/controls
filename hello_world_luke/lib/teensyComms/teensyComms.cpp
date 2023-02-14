#include "teensyComms.h"
bool teensyComms::readCommand(char* command_type, float* magnitude){
    // buffer to store bytes from serial buffer
    char buffer[32];
    

    // get the length of the rest of the bytes (should be 4 bytes)
    int msg_len = Serial.available();

    // Get bytes from serial buffer and store in `buffer`
    Serial.println("Reading in from Pi");
    for(int i=0; i<msg_len; i++){
        buffer[i] = Serial.read();
        Serial.println(buffer[i]);
    }

    // The first byte that gets sent is like a 
    // 2nd byte should be a character that corresponds to a command
    // TODO: validate the character is correct
    *command_type = buffer[0];

    // copy data from buffer and put into float* magnitude
    memcpy(magnitude, &buffer[1], 4);    // receive data

    Serial.clear();
    return true;
}
