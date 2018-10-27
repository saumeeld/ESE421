// Week 4: Heading Control (w/ GPS + IMU + Camera)

// Libraries included:
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Wire.h>

//constants
float MILLISECONDS_TO_SECONDS = .001;

// global variables
// (yucky but needed to make i2c interrupts work later)
float gpsLat;
float gpsLon;
static float psiGPS;
int gpsNSat;

byte piCommand;
float piE = 3.1416;
float sqrtN = 1.0;
word fiveK = 5000;
byte x = 1;
byte xsq = 1;
byte piData[2];

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

//Define comm. address with Raspberry Pi
#define SLAVE_ADDRESS 0x04


// Define pins for ping sensor
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)
#define motorPin 8 //PWM for motor
int updateTime = 1000; //delay
float pingDistanceCM = 0.0;
static float yawRate = 0;
static float psiEst = 0;


// Tell sensor library which pins for accel & gyro data
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

#define STEERING_SERVO_PIN 7
Servo steeringServo;
byte motorPWM = 175;
byte psiLF;

// connect GPS to Hardware Serial1 (Serial0 is for USB)
// Serial1 is pins 18 & 19 on Mega
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);

void setup() {
  Serial.begin(115200);
  Serial.println("Heading Control");

  //I2C stuff
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(getCameraData);
  Wire.onRequest(sendDataI2C);

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
  // Emergency stop code
  emergencyStopIfNecessary();

  yawRate = getIMUData();
  psiGPS = getGPSData();

  // Correct vehicle heading if not straight
  estimateHeading(yawRate, psiGPS, psiLF, updateTime, psiEst);
  fixHeading(psiEst, steeringServo);

  //  wait updateTime milliseconds
  delay(updateTime);
}

float constrainAngle(float angle) {
  angle = fmod(angle, 360);
  if (angle < 0)
    angle += 360;
  return angle;
}

// Complementarty filter implementation to estimate vehicle heading
float estimateHeading(float yawRate, float psiGPS, byte psiLF, int updateTime, float psiEst) {
  float tauPsi = 1;
  psiEst = complementaryFilter(psiLF, yawRate, tauPsi, updateTime, psiEst);
  Serial.print("psiEst: ");
  Serial.println(psiEst);
  return psiEst;
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


// Function to estimate a value a low frequency input and high frequency rate input
float complementaryFilter(float lowFrequencyInput, float highFrequencyInputRate, float tau, int updateTime, float estimate) {
  float deltaEstimate = 0;
  deltaEstimate = (1 / tau) * (lowFrequencyInput + tau * highFrequencyInputRate);
  estimate += deltaEstimate * updateTime * MILLISECONDS_TO_SECONDS;
  return estimate;
}

float getIMUData() {
  //  Get the IMU data
  lsm.read();  /* ask it to read in the data */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  // Read yaw rate from IMU
  Serial.print("\tZ: "); Serial.print(g.gyro.z); Serial.println(" dps");
  float gyroBias = 1;
  float yawRate;
  yawRate = g.gyro.z - gyroBias;
  Serial.print("yawRate: " );
  Serial.println(g.gyro.z);
  return yawRate;
}

float getGPSData() {
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
  return psiGPS;
}

// Function to manage collision distance
void emergencyStopIfNecessary() {
  getPingDistanceCM();
  if (pingDistanceCM < 35) {
    analogWrite(motorPin, 0);
  }
  else {
    analogWrite(motorPin, motorPWM);
  }
}

// Function to retrieve pin distance
void getPingDistanceCM() {
  // 3000 us timeout implies maximum distance is 51cm but in practice, actual max larger?
  const long timeout_us = 3000;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(pingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrigPin, LOW);
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  unsigned long echo_time;
  echo_time = pulseIn(pingEchoPin, HIGH, timeout_us);
  if (echo_time == 0) {
    echo_time = timeout_us;
  }
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0) = 0.017*echo_time
  pingDistanceCM = constrain(0.017 * echo_time, 5.0, 50.0);
}

void getCameraData(int nPoints) {
  piCommand = Wire.read();

  // if Pi is sending data, parse it into the two command variables
  if (piCommand == 255) {
    psiLF = Wire.read();
  }

  // now clear the buffer, just in case
  while (Wire.available()) {
    Wire.read();
  }
}

void sendDataI2C(void) {
  //  if (piCommand == 1) {
  //    float dataBuffer[2];
  //    dataBuffer[0] = piE;
  //    dataBuffer[1] = sqrtN;
  //    Wire.write((byte*) &dataBuffer[0], 2 * sizeof(float));
  //    Serial.println("sending floats");
  //  }
  //  else if (piCommand == 2) {
  //    byte dataBuffer[4];
  //    dataBuffer[0] = fiveK / 256;
  //    dataBuffer[1] = (fiveK - 256 * dataBuffer[0]);
  //    dataBuffer[2] = x;
  //    dataBuffer[3] = xsq;
  //    Wire.write(&dataBuffer[0], 4);
  //    Serial.println("sending bytes");
  //  }
}



