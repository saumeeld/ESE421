#include <Adafruit_GPS.h>

// MegaGPSTest.ino
// BDK:ESE421:2018C
// Week 2 Lab Sketch -- Test GPS
//
// Need the Adafruit Sensor Library
//

#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>

//
// global variables
// (yucky but needed to make i2c interrupts work later)
//
float gpsLat;
float gpsLon;
float gpsV;
float gpsPsi;
int gpsNSat;

//
// connect GPS to Hardware Serial1 (Serial0 is for USB)
// Serial1 is pins 18 & 19 on Mega
//
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);

void setup() {
  Serial.begin(115200);
  Serial.println("MegaGPSTest");

  //
  //  Activate Interrupt for GPS
  //
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  //
  //  initialize comm with GPS at 9600 baud
  //
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
  //
  //  parse GPS when available (set by interrupt)
  //
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
      gpsV = GPS.speed;
      gpsPsi = GPS.angle;
      gpsNSat = GPS.satellites;

      Serial.print((int)GPS.satellites); Serial.print(' ');
      Serial.print(GPS.hour, DEC); Serial.print(':');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      Serial.print(GPS.milliseconds); Serial.print(' ');

      Serial.print(gpsLat, 5); Serial.print(", "); Serial.print(gpsLon, 5);
    }
  }

  //
  //  pause 1 second
  //
  Serial.println();
  delay(1000);
}
