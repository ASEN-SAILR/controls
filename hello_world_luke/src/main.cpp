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
  int data[16];

  while(true){

    if (Serial.available()){
      int len = Serial.available();

      //read in bytes of data
      for(int i=0; i<len; i++){

        data[i] = Serial.read();

        //set led high and low
        digitalWrite(led_pin,HIGH);
        delay(100);
        digitalWrite(led_pin,LOW);
        delay(100);
      }

      // indicate that the teensy is about to send bytes back with longer
      //  LED pulses
      for(int i=0; i<3; i++){
        digitalWrite(led_pin,HIGH);
        delay(500);
        digitalWrite(led_pin,LOW);
        delay(500);
      }

      for(int i=0; i<len; i++){
        Serial.write(data[i]);

        //turn led on and off
        digitalWrite(led_pin,HIGH);
        delay(100);
        digitalWrite(led_pin,LOW);
        delay(100);
      }
    }
  } 
}