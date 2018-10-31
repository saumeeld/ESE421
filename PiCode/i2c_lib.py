# adapted from code at https://gist.github.com/gileri/5a9285d6a1cfde142260
#
# check out this information about the "command" parameter (second argument in block read / write)
# https://raspberrypi.stackexchange.com/questions/8469/meaning-of-cmd-param-in-write-i2c-block-data
#
import struct
import smbus
import time



#
# this is the default Slave address of the Arduino (in case you forget)
#
# address = 0x04

#COMMAND BYTES
READ_INIT_CMD_BYTE = 1
WRITE_INIT_CMD_BYTE = 255


#ASSUME ALL NUMBERS ARE AT MOST 10 CHARACTERS
I2C_STRING_SIZE = 10

#
# 1 = command byte to request first data block from Arduino
# slaveAddress: slave address of the arduino
def getString(slaveAddress):
    try:
        data_received = bus.read_i2c_block_data(slaveAddress, READ_INIT_CMD_BYTE, I2C_STRING_SIZE)
        stringReceived = ''.join(data_received).strip()
    except:
        print("error reading string")
        stringReceived = "ERR"

    return stringReceived

#
# 255 = command byte to initiate writing to Arduino
# (arbitrary--must be different than read)
# slaveAddress: slave address of the arduino
def putByteList(byteList, slaveAddress):
    try:
        bus.write_i2c_block_data(slaveAddress, 255, byteList)
    except:
        print("error writing commands")
    return None

#
# crazy conversion of groups of 4 bytes in an array into a float
# simple code assumes floats are at beginning of the array
# "index" = float index, starting at 0
#
def bytes_2_float(data, index):
    bytes = data[4*index:(index+1)*4]
    return struct.unpack('f', "".join(map(chr, bytes)))[0]

#initialize the bus to begin i2c communications
def initialize_i2c():
    #
    # smbus implements i2c on the RPi
    #
    bus = smbus.SMBus(1)

def main():
    getString(0x04)

if __name__ == "__main__":
    main()