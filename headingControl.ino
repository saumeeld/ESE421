// MegaIMUTest.ino
// BDK:ESE421:2018C
// Week 4: Heading Control (w/ GPS + IMU)

// Libraries included:
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// global variables
// (yucky but needed to make i2c interrupts work later)
float gpsLat;
float gpsLon;
float psiGPS;
int gpsNSat;

// DEFINE PIN LAYOUT

// Define pins for IMU
// IMU uses SPI -- here are the pins on the Mega
// (Pins 49 & 47 are user selection)
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega
#define GYRO_Z_BIAS 1 //1 dps bias
static float psiEst = 0;

// Define pins for ping sensor
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)
#define motorPin 8 //PWM for motor
int delta_tms = 1000; //delay
float pingDistanceCM = 0.0;

// Tell sensor library which pins for accel & gyro data
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

#define STEERING_SERVO_PIN 7
Servo steeringServo;
byte motorPWM = 175;

// connect GPS to Hardware Serial1 (Serial0 is for USB)
// Serial1 is pins 18 & 19 on Mega
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);

void setup() {
  Serial.begin(115200);
  Serial.println("Heading Control");

  // Define pins for ping sensor
  pinMode(pingGrndPin, OUTPUT); digitalWrite(pingGrndPin, LOW);
  pinMode(pingTrigPin, OUTPUT);
  pinMode(pingEchoPin, INPUT);
  pinMode(motorPin, OUTPUT);
  analogWrite(motorPin, 0);

  // Define pins for steering servo
  steeringServo.attach(STEERING_SERVO_PIN);
  steeringServo.write(90);

  // IMU STUFF
  // Initialize gyro / mag / accel
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // Set ranges for sensor
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  //  GPS STUFF
  //  Activate Interrupt for GPS
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  //  Initialize comm with GPS at 9600 baud
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // minimum information only
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
}
//  This Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

//////////////////////////////////////////////////////////////////
void loop() {

  //  Get the gyro data
  lsm.read();  /* ask it to read in the data */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  // Read yaw rate from gyro
  Serial.print("\tZ: "); Serial.print(g.gyro.z); Serial.println(" dps");
  float gyroBias = 1;
  static float inertialRate = 0;
  inertialRate = g.gyro.z - gyroBias;
  Serial.print("inertialRate: " );
  Serial.println(g.gyro.z);

  //  Parse GPS when available (set by interrupt)
  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
    {
      Serial.println("GPS Parse Fail");
    }
    else
    {
      gpsLat = (GPS.latitudeDegrees - 40.0);
      gpsLon = (GPS.longitudeDegrees + 75.0);
      gpsNSat = GPS.satellites;
      psiGPS = GPS.angle;
      Serial.print("psiGPS: ");
      Serial.println(GPS.angle);
      Serial.print("GPS Satellites: ");
      Serial.println(GPS.satellites);
    }
  }

  // Emergency stop code
  getPingDistanceCM();
  if (pingDistanceCM < 35) {
    analogWrite(motorPin, 0);
  }
  else {
    analogWrite(motorPin, motorPWM);
  }

  // Correct vehicle heading if not straight
  estimateHeading(inertialRate, psiGPS, delta_tms, psiEst);
  fixHeading(psiEst, steeringServo);

  //  wait delta_tms milliseconds
  Serial.println();
  delay(delta_tms);
}

float constrainAngle(float angle) {
  angle = fmod(angle, 360);
  if (angle < 0)
    angle += 360;
  return angle;
}

// Complementarty filter implementation to estimate vehicle heading
void estimateHeading(float inertialRate, float psiGPS, int delta_tms, float psiEst) {
  float tau = 1;
  static float psiIMU = 0;
  static float psiX = 0;
  psiIMU = constrainAngle((.001 * delta_tms) * inertialRate);
  Serial.print("psiIMU: ");
  Serial.println(psiIMU);
  // Remember to add constraint to psiGPS - psiX to stop dramatic jumps
  psiX += (.001 * delta_tms) * ((1/tau) * (psiGPS - psiX));
  Serial.print("psiX: ");
  Serial.println(psiX);
  psiEst = psiIMU + psiX;
  Serial.print("psiEst: ");
  Serial.println(psiEst);
}

// Correct heading using proportional feedback
void fixHeading(float psiEst, Servo steeringServo) {
  float k_heading = 1.5;
  float servo_rigging_angle = 54;
  float servo_angle_deg = servo_rigging_angle - k_heading * psiEst;
  float servo_actual = constrain(servo_angle_deg, servo_rigging_angle - 50, servo_rigging_angle + 50);
  steeringServo.write(servo_actual);
  Serial.print("Servo angle " );
  Serial.println(servo_actual);
}

// Function to retrieve pin distance
void getPingDistanceCM()
{
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = 3000;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(pingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrigPin, LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(pingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
  {
    echo_time = timeout_us;
  }
  //
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  pingDistanceCM = constrain(0.017 * echo_time, 5.0, 50.0);
}
