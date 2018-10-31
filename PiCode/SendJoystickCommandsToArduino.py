# adapted from code at https://gist.github.com/gileri/5a9285d6a1cfde142260
#
# check out this information about the "command" parameter (second argument in block read / write)
# https://raspberrypi.stackexchange.com/questions/8469/meaning-of-cmd-param-in-write-i2c-block-data
#
import struct
import smbus
import time
import pygame
import time
import picamera

#
# 1 = command byte to request first data block from Arduino
# 8 = number of bytes (one 4-byte float + one 2-byte word)
#
def getFloatData(oldFloats):
    try:
        data_received = bus.read_i2c_block_data(address, 1, 8)
        newFloats = [bytes_2_float(data_received, 0)]
        newFloats.append(bytes_2_float(data_received, 1))
    except:
        print("error reading float data")
        newFloats = oldFloats;

    return newFloats

#
# 2 = command byte to request second data block from Arduino
# 4 = number of bytes (one 2-byte word + two bytes)
#
def getByteData(oldBytes):
    try:
        data_received = bus.read_i2c_block_data(address, 2, 4)
        newBytes = [data_received[0]*255 + data_received[1]]
        newBytes.append(data_received[2])
        newBytes.append(data_received[3])
    except:
        print("error reading byte data")
        newBytes = oldBytes;

    return newBytes

#
# 255 = command byte to initiate writing to Arduino
# (arbitrary--must be different than read)
#
def putByteList(byteList):
    try:
        bus.write_i2c_block_data(address, 255, byteList)
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

#
# initialize dummy value of output from Pi (bytes only)
#
byteListDummyFromPi = [150, 220]

#
# initialize dummy values of inputs to Pi
#
dummyToPiFloats = [-3.1416, 6.2832]
dummyToPiBytes = [2047, 50, 50]

def convertHatToNumber(hat_tuple):
    if (hat_tuple == (0,1)):
        return 0
    if (hat_tuple == (0,-1)):
        return 1
    if (hat_tuple == (-1,0)):
        return 2
    if (hat_tuple == (1,0)):
        return 3
    
    return 4 #unrecognized hat

def convertButtonToNumber(buttons):
    for i in range(J.get_numbuttons()):
        if buttons[i] == 1:
            return i
    return 20  

def takePicture():
    imgName = 'Picture' + str(picturesTaken) + '.jpg' 
    camera.capture(imgName)

#
# now loop thru reading from and writing to Arduino
#

pygame.init()
pygame.joystick.init()
J = pygame.joystick.Joystick(0)
J.init()
print(J.get_name())

#initialize cameras
camera = picamera.PiCamera()
photoHeight = 540
camera.resolution = (16*photoHeight/9, photoHeight)
picturesTaken = 0

while True:
    time.sleep(0.1)
#    dummyToPiFloats = getFloatData(dummyToPiFloats)
#    dummyToPiBytes = getByteData(dummyToPiBytes)
#    print(dummyToPiFloats, dummyToPiBytes)

#
#   send variable to Pi
#
    
    buttons = []
    for k in range(J.get_numbuttons()):
        pygame.event.pump()
        buttons.append(J.get_button(k))
    arduinoButtonRepresentation = [convertButtonToNumber(buttons)]
    if arduinoButtonRepresentation[0] == 7:
        take_picture() #When button is z
        picturesTaken++


    arduinoHatRepresentation = [] #should be only one number
    for k in range(J.get_numhats()):
        pygame.event.pump()
        arduinoHatRepresentation.append(convertHatToNumber(J.get_hat(k)))
        
##    print("The arduino hat representation is {}".format(arduinoHatRepresentation))
##    print("buttons array {}".format(buttons))
##    print("The arduino button representation is {}".format(arduinoButtonRepresentation))
    byteListToSend = [arduinoHatRepresentation[0], arduinoButtonRepresentation[0]]
    
##    if (arduinoButtonRepresentation == 7): #Z BUTTON
##        takePicture()
        
    putByteList(byteListToSend)



        








