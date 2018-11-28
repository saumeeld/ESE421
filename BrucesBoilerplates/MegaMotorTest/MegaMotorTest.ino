// MegaMotorTest.ino
// BDK:ESE421:2018C
// Week 2 Lab Sketch -- Test the motor PWM control
//

#define motorPin 8 // PWM for motor

//
// global variables
// (yucky but needed to make i2c interrupts work later)
//
byte motorPWM=150;
void setup() {
    Serial.begin(115200);       // for sending info to the terminal
    Serial.println("MegaMotorTest");
//
//  make the motors not spin
//
    pinMode(motorPin,OUTPUT);
    analogWrite(motorPin,0);
}

//////////////////////////////////////////////////////////////////
void loop() {
//
//  toggle motor command back and forth: high<-->low speed
//
    motorPWM = 400 - motorPWM;
    analogWrite(motorPin,motorPWM);
    Serial.print(motorPWM,DEC);
    Serial.print(" ");

//
//  pause 1 second
//
    Serial.println();
    delay(1000);
}
