# adapted from code at https://gist.github.com/gileri/5a9285d6a1cfde142260
#
# check out this information about the "command" parameter (second argument in block read / write)
# https://raspberrypi.stackexchange.com/questions/8469/meaning-of-cmd-param-in-write-i2c-block-data
#
import struct
import smbus
import time

#
# 1 = command byte to request first data block from Arduino
# 8 = number of bytes (one 4-byte float + one 2-byte word)
#
def getByteList():
    try:
        data_received = bus.read_i2c_block_data(address, 1, 10)
        print("Data data_received from Arduino: %s", byteArrayToString(data_received)) 
        return float(byteArrayToString(data_received))
    except:
        print("error reading float data")
        
    return -1
    #return newFloats


#
# 255 = command byte to initiate writing to Arduino
# (arbitrary--must be different than read)
#
def putByteList(byteList):
    try:
        print(address)
        print(byteList)
        bus.write_i2c_block_data(address, 255, byteList)
    except:
        print("error writing commands")
    return None

def numToByteArray(num):
    byte_list = []
    serializedNum = str(num)
    
    for i in serializedNum:
        byte_list.append(ord(i))
    
    while len(byte_list) < 9:
        byte_list.append(0)
        
    return byte_list[:9]    
        
    

def byteArrayToString(byteArray):
    string = ""
    
    for byte in byteArray:
        if (byte == 0):
            return string
        string += chr(byte)
        
    return string

#
# crazy conversion of groups of 4 bytes in an array into a float
# simple code assumes floats are at beginning of the array
# "index" = float index, starting at 0
#
def bytes_2_float(data, index):
    bytes = data[4*index:(index+1)*4]
    return struct.unpack('f', "".join(map(chr, bytes)))[0]


##########
# main part of script starts here
##########

#
# smbus implements i2c on the RPi
#
bus = smbus.SMBus(1)

#
# this is the Slave address of the Arduino
#
address = 0x04




