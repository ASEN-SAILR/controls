#include <Arduino.h>

const int led_pin = LED_BUILTIN;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  digitalWrite(led_pin,LOW);
}

void loop() {
  // char array to store data from pi
  int data[16];

  while(true){

    Serial.write("Hello World\n");
    // if (Serial.available()){
    //   int len = Serial.available();

    //   //read in bytes of data
    //   for(int i=0; i<len; i++){

    //     data[i] = Serial.read();

    //     //set led high and low
    //     digitalWrite(led_pin,HIGH);
    //     delay(100);
    //     digitalWrite(led_pin,LOW);
    //   }

    //   // indicate that the teensy is about to send bytes back with longer
    //   //  LED pulses
    //   for(int i=0; i<3; i++){
    //     digitalWrite(led_pin,HIGH);
    //     delay(500);
    //     digitalWrite(led_pin,LOW);
    //   }

    //   for(int i=0; i<len; i++){
    //     Serial.write(data[i]);

    //     //turn led on and off
    //     digitalWrite(led_pin,HIGH);
    //     delay(100);
    //     digitalWrite(led_pin,LOW);
    //   }
    // }
  } 
}