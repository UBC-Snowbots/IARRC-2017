/* Firmware for IARRC 2017
   Author: Vincent Yuan, 
   Modified: Nick Wu
   Modified: James Asefa
   Modified: Jinhao Lu
   Modified: Gareth Ellis
   Date Last Modified: July 8, 2017
*/

/*
 * UBC Snowbots - IARRC 2017
 * Firmware for Control of an RC car
 * 
 * This firmware will take in a message of the form:
 * `B<linear x><linear y><linear z><angular x><angular y><angular z>`
 * where each `<>` is a single byte. The degree to rotate the servo controlling
 * the front wheels is determined from `angular z`, and the speed of the car by `linear x`
 * (the rest of the values are discarded)
 */

#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>

// Uncomment this to enable debug logging - messages sent will be echoed back
//#define DEBUG

#define BAUD_RATE 9600

// size of character buffer being passed over serial connection
#define BUFFER_SIZE 7

// Robot speed will be received in a char buffer with each value between 0 and 255
// as a result, this assigns 128 as stop. Then:
// 0 < speed < 128 is reverse
// 128 < speed <= 255 is forward
// 255 is then full speed forward, 0 is full speed backward
#define UNMAPPED_STOP_SPEED 128
#define MAPPED_STRAIGHT_ANGLE 90

// Robot will keep executing the last command unless the period exceeds the INTERVAL
#define INTERVAL 5000

void serial_read();
void convert();
void drive(int, int);

// buffer inputs
int linear_x = UNMAPPED_STOP_SPEED;
int linear_y = UNMAPPED_STOP_SPEED;
int linear_z = UNMAPPED_STOP_SPEED;
int angular_x = MAPPED_STRAIGHT_ANGLE;
int angular_y = MAPPED_STRAIGHT_ANGLE;
int angular_z = MAPPED_STRAIGHT_ANGLE;

// motor pins
const int MOTOR_PIN = 10;
const int DIRECTION_PIN = 11;

// defines start of buffer
const char BUFFER_HEAD = 'B';

// max and min linear speeds and stopping condition
const int LINEAR_MAX = 255;
const int LINEAR_MIN = 80;
const int LINEAR_STOP = 0;

// max and min angular speeds and stopping condition
// the angle ranges from 33 to 147
const int ANGULAR_MAX = 145;
const int ANGULAR_MIN = 35;
const int ANGULAR_STOP = 90;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

Servo motor;
Servo direction_motor;

void setup() {

	Serial.begin(BAUD_RATE);
	motor.attach(MOTOR_PIN);
	direction_motor.attach(DIRECTION_PIN);

	// arm the motor
	motor.writeMicroseconds(1500);
 }

void loop() {
	serial_read(); 
	convert();
	drive(linear_x, angular_z);
}

void serial_read(){
  // BUFFER_HEAD identifies the start of the buffer
	if ( Serial.available() >= BUFFER_SIZE && Serial.read() == BUFFER_HEAD ) {
    Serial.println("Got message");
		linear_x = Serial.read();
		linear_y = Serial.read();
		linear_z = Serial.read();
		angular_x = Serial.read();
		angular_y = Serial.read();
		angular_z = Serial.read();
		previousMillis = currentMillis;
   
   #ifdef DEBUG
    Serial.println(linear_x);
    Serial.println(linear_y);
    Serial.println(linear_z);
    Serial.println(angular_x);
    Serial.println(angular_y);
    Serial.println(angular_z);
   #endif
	} else {
	  currentMillis = millis();
    if(currentMillis - previousMillis > INTERVAL){
	    linear_x = UNMAPPED_STOP_SPEED;
    }
	}
}

void convert() {
	// safety check if values are out of range
	if (linear_x > LINEAR_MAX)
		linear_x = LINEAR_MAX; 
	else if (linear_x < LINEAR_MIN)
		linear_x = LINEAR_MIN;

	if (angular_z > ANGULAR_MAX)
		angular_z = ANGULAR_MAX; 
	else if (angular_z < ANGULAR_MIN)
		angular_z = ANGULAR_MIN;
}

void drive(int linear_speed, int angular_speed){
	int velocity = 1500;
	int angle = 1500;

	velocity = map(linear_speed, 0, 255, 1000, 2000); 
	angle = map(angular_speed, 0, 180, 1000, 2000);

	motor.writeMicroseconds(velocity);
	direction_motor.writeMicroseconds(angle);
}

