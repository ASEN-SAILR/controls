#include <Arduino.h>

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
  int command[16];

  while(true){

    if (Serial.available()){
      int command_length = Serial.available();

      //read in bytes of data
      for(int i=0; i<command_length; i++){

        digitalWrite(led_pin,HIGH);
        command[i] = Serial.read();
        digitalWrite(led_pin,LOW);
      }

      for(int i=0; i<command_length; i++){
        digitalWrite(led_pin,HIGH);
        Serial.write(command[i]);
        digitalWrite(led_pin,LOW);
      }
      
      // indicate that the teensy is recived a string
      // (blink..blink..........blink..blink)
      for(int j=0; j<2; j++){
        for(int i=0; i<2; i++){
          digitalWrite(led_pin,HIGH);
          delay(100);
          digitalWrite(led_pin,LOW);
          delay(100);
        }
        delay(400);
      }

      int command_type = command[0];

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

    }
  } 
}