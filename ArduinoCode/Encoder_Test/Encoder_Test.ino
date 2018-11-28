#include <TimerOne.h>
// library: https://code.google.com/archive/p/arduino-timerone/downloads
// https://www.youtube.com/watch?v=9848QETGd_k

// Using HC-020K Photoelectric encoder on the Dual Axis Reducer Motor 
// with 100nF capacitor on the encoder

#define motorPin 8 // PWM for motor
byte motorPWM=150;

int encoder_pin = 6; 
unsigned int counter=0;

void setup() {
  Serial.begin(115200);
  Serial.println("EncoderTest");

//  make the motors not spin
  pinMode(motorPin,OUTPUT);
  analogWrite(motorPin,0);

 // Intitialize encoder
  pinMode(encoder_pin, INPUT);
  Timer1.initialize(1000);  // set for 1 sec
  attachInterrupt(0,do_count,RISING);
  Timer1.attachInterrupt(timerIsr);
}

void loop() {
//  toggle motor command back and forth: high<-->low speed
//
    motorPWM = 400 - motorPWM;
    analogWrite(motorPin,motorPWM);
//    Serial.print(motorPWM,DEC);
//    Serial.print(" ");

// Pause 1 second
//    Serial.println();
//    delay(1000);

}

void do_count() {
  counter++;
}

void timerIsr() {
  Timer1.detachInterrupt();
  Serial.print("Motor Speed: ");
  int rotation = (counter/20)*60;
  Serial.print(rotation, DEC);
  Serial.println(" RPM");
  counter=0;
  Timer1.attachInterrupt(timerIsr);
}
