/* Firmware for ????
   Author: Jinhao Lu
   Description: Servo Angle Range Test
   Test Results: Angle ranges from 33 to 147.
*/

#include <Servo.h>

Servo myservo; 

char pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}

void loop() {
  if(Serial.available() > 0){
    pos = Serial.read();
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
  }
}

