#include "Arduino.h"
#include <Wire.h>
#include "i2c.h"


#define STRING_SIZE 10
#define SEND_STRING_COMMAND 1
#define RECEIVE_STRING_COMMAND 255

#define OFFSET_TAG 'O'
#define HEADING_TAG 'H'

//Data that comes from Pi
float offsetCamera;
float psiCamera;

extern float psiEst;
extern boolean psiEstInitialized;

I2C::I2C(int slaveAddress)
{
    _slaveAddress = slaveAddress;
    Wire.begin(_slaveAddress);
    Wire.onReceive(receiveData);
    Serial.println();
    //Wire.onRequest(sendData);
}

void I2C::receiveData(int bytesReceived)
{
    byte piCommand = Wire.read();

    //
    // if Pi is sending data, parse it into incoming data array
    //
    byte receivedBytes[STRING_SIZE];
    if (piCommand == RECEIVE_STRING_COMMAND) {
        for (int i = 0; i < STRING_SIZE; i++) {
            receivedBytes[i] = Wire.read();
        }
    }
    else {
        return;
    }

    Serial.println("NOTIFICATION: Received Data from Pi");

    //Last byte identifies what data is being received
    float floatReceived = I2C::convertByteArrayToFloat(receivedBytes); 

    for (int i = 0; i < STRING_SIZE; i++) {
      Serial.print(receivedBytes[i]); Serial.print(" ");
    }
    Serial.println();
    Serial.print("Last Byte is: " ); Serial.println(receivedBytes[STRING_SIZE-1]);
    switch(receivedBytes[STRING_SIZE-1]) {
        case 'O' : 
            offsetCamera = floatReceived;
            Serial.print("The received camera offset is: "); Serial.println(offsetCamera,3);
            break;
        case 'H' :
            psiCamera = floatReceived;
            Serial.print("The received camera heading is: "); Serial.println(psiCamera,3);
            if (!psiEstInitialized) {
              Serial.println("psiEst has been initalized");
              Serial.print("Initial psi is: ");Serial.println(psiCamera);
              psiEst = psiCamera;
              psiEstInitialized = true;
            }
            break; 
    }

    //
    // now clear the buffer, just in case
    //
    while (Wire.available()) {Wire.read();}
}

float I2C::convertByteArrayToFloat(byte serializedFloat[])
{
    String floatAsString = "";

    //First 9 bytes contain float.
    for (int i = 0; i < STRING_SIZE - 1; i++) {
       floatAsString += char(serializedFloat[i]);
    }
    Serial.print("Float as String is: "); Serial.println(floatAsString);
    return floatAsString.toFloat();
}
