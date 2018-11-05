//
// some useful stuff
// http://gammon.com.au/i2c
//

#include <Wire.h>

#define SLAVE_ADDRESS 0x04
#define STRING_SIZE 4
//
// global variables
// (yucky but needed to make i2c interrupts work)
//
byte piCommand;
float piE = 3.1416;
float sqrtN = 1.0;
word fiveK = 5000;
byte x = 1;
byte xsq = 1;
byte piData[2];
void setup() {
//
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveDataI2C);
    Wire.onRequest(sendDataI2C);

    Serial.begin(115200);
}


//////////////////////////////////////////////////////////////////
void loop() {
  delay(500);  // update every half second
  if (piE > 3) {
    piE = 2.71828;
  }
  else {
    piE = 3.1416;
  }
  sqrtN = sqrt(sqrtN*sqrtN + 1.0);

  fiveK += 5;
  x += 1;
  xsq = x * x;

  Serial.print(piData[0]); Serial.print(" "); Serial.println(piData[1]);
}

void receiveDataI2C(int nPoints) {
      piCommand = Wire.read();
      //
      // if Pi is sending data, parse it into incoming data array
      //
      if (piCommand == 255) {
          piData[0] = Wire.read();
          piData[1] = Wire.read();
      }
      //
      // now clear the buffer, just in case
      //
      while (Wire.available()) {Wire.read();}
}

void sendDataI2C(void) {
  Serial.println("Entered send data");
  Serial.print("PiCommand "); Serial.println(piCommand);
    if (piCommand == 1) {
        //Serialize data and send over arduino
//      Serial.println("Got a request to send a serialized string");
//      byte stringBuffer[STRING_SIZE];
//      String serializedNumber = String(piE);
//      serializedNumber.toCharArray(stringBuffer, STRING_SIZE);
//      Serial.println(stringBuffer[0]);
//      Serial.println(stringBuffer[1]);
//      Serial.println(stringBuffer[2]);

        byte dataBuffer[1];
        dataBuffer[0] = fiveK / 256;
        
      Wire.write(&dataBuffer[0], 1); //character is 1 byte
    }
    else if (piCommand == 2) {
        byte dataBuffer[4];
        dataBuffer[0] = fiveK / 256;
        dataBuffer[1] = (fiveK - 256*dataBuffer[0]);
        dataBuffer[2] = x;
        dataBuffer[3] = xsq;
        Wire.write(&dataBuffer[0], 4);
        Serial.println("sending bytes");
    }
}
