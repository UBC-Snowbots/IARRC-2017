/* ESC Calibration for IARRC
   Author: Jinhao Lu, 
   Date: June 20 2017
*/

#include <Servo.h>

#define BAUD_RATE 9600

const int MOTOR_PIN = 10;
char buffer = 0;

Servo motor;

void setup() {

	Serial.begin(BAUD_RATE);
	Serial.println("Setting up");
	motor.attach(MOTOR_PIN);
	Serial.println();
    Serial.println("WARNING: For safety concerns, please disconnect one of the motor wires between the XL-5 and the motor");
    Serial.println();
    Serial.println("Simply power off the ESC and start again if you fail to follow the instruction");
	Serial.println();
	Serial.println("1. Power on the ESC");
	Serial.println("2. Enter '1' on the monitor to set throttle at neutral");
 }

void loop() {
	buffer = Serial.read();

	if(buffer == '1'){
		motor.writeMicroseconds(1500);
		Serial.println();
		Serial.println("3. Press and hold the EZ button till the LED first turns green and then red");
		Serial.println("4. Release the button");
		Serial.println("5. Wait till the LED blinks RED ONCE");
		Serial.println("6. Enter '2' on the monitor");
	}
	else if(buffer == '2'){
		motor.writeMicroseconds(2000);
    	Serial.println();
		Serial.println("7. Wait till the LED blinks RED TWICE");
		Serial.println("6. Then Enter '3' on the monitor");
    	Serial.println();
	}
	else if(buffer == '3'){
		motor.writeMicroseconds(1000);
		Serial.println("Wait till the LED blinks GREEN ONCE and then turns solid red or green (dependent on whether voltage check feature is enabled or not)");
		Serial.println();
		Serial.println("Calibration completed");
	}
}
