// MegaServoTest.ino
// BDK:ESE421:2018C
// Week 2 Lab Sketch -- Test Servo Steering

#include <Servo.h>

#define servoPin 7 // pin for servo signal

//
// global variables
// (yucky but needed to make i2c interrupts work later)
//
Servo steeringServo;
byte servoAngleDeg = 90;

void setup() {
    Serial.begin(115200);
    Serial.println("MegaServoTest");
//
//  connect the steering servo & send command
//
    steeringServo.attach(servoPin);
    steeringServo.write(servoAngleDeg);

}

//////////////////////////////////////////////////////////////////
void loop() {
//
//  set servo command
//  go 10 deg toward right every second then reset left
//  NEVER send a servo command without constraining servo motion!
//  -->large servo excursions could damage hardware or servo<--
//
    servoAngleDeg = servoAngleDeg + 10;
    if (servoAngleDeg > 120) {
      servoAngleDeg = 20;
    }
    steeringServo.write(constrain(servoAngleDeg,20,120));
    Serial.print(servoAngleDeg,DEC);
    Serial.print(" ");

//
//  pause 1 second
//
    Serial.println();
    delay(1000);
}
