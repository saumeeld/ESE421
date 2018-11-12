/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef I2C_LIB_H
#define I2C_LIB_H

#include "Arduino.h"

class I2C
{
  public:
    I2C(int slaveAddress);
    static void receiveData(int bytesReceived);
    //static void sendData();
  private:
    int _slaveAddress;
    static float convertByteArrayToFloat(byte serializedFloat[]);
};

#endif