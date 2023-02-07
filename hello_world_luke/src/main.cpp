#include <Arduino.h>
#include <string.h>
#include "teensyComms.h"

const int led_pin = LED_BUILTIN;

void blinkNTimes(int N){
  for(int i=0; i<N; i++){

    //set led high and low
    digitalWrite(led_pin,HIGH);
    delay(500);
    digitalWrite(led_pin,LOW);
    delay(500);
  }
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  digitalWrite(led_pin,LOW);
}

void loop() {
  // char array to store data from pi
  // int command[16];

  teensyComms comms = teensyComms();


  float magnitude;
  char command_type;

  while(true){

    if (Serial.available()){


      comms.readCommand(&command_type,&magnitude);


      // //read in bytes of data
      // for(int i=0; i<command_length; i++){

      //   digitalWrite(led_pin,HIGH);
      //   command[i] = Serial.read();
      //   digitalWrite(led_pin,LOW);
      // }

      // for(int i=0; i<command_length; i++){
      //   digitalWrite(led_pin,HIGH);
      //   Serial.write(command[i]);
      //   digitalWrite(led_pin,LOW);
      // }

      // indicate that the teensy is recived a string
      // (blink..blink..........blink..blink)
      // for(int j=0; j<2; j++){
      //   for(int i=0; i<2; i++){
      //     digitalWrite(led_pin,HIGH);
      //     delay(100);
      //     digitalWrite(led_pin,LOW);
      //     delay(100);
      //   }
      //   delay(400);
      // }



      switch(command_type){
        case int('r'):
            blinkNTimes(1);
          break;
        
        case int('t'):
            blinkNTimes(2);
          break;

        case int('s'):
            blinkNTimes(3);
          break;
        
        default:
          break;
      }

      // String message = sprintf("com: %c. mag: %f\n", command_type,magnitude);
      String message = "Will this send";

      for(int i=0; i<message.length(); i++){
        digitalWrite(led_pin,HIGH);
        Serial.write(message[i]);
        digitalWrite(led_pin,LOW);
      }

    }
  } 
}