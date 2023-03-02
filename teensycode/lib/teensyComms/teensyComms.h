#include <Arduino.h>
#include <string.h>

class teensyComms{
    private:
        /* data */
    public:
        teensyComms(/* args */){Serial.begin(115200);};
        ~teensyComms();

        bool readCommand(char* command_type, float* magnitude);
};


