#include "exClass.h"

exClass obj = exClass(5);

void setup(){
    Serial.begin(115200);
}

void loop(){
    Serial.print(obj.exPublicFunc(2));
}