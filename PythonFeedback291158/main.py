import math
import struct
import smbus
import time
from operator import itemgetter
import imageProcessing
import picamera
import mapper
import joyCommander
import sys


#COMS
#
# 1 = command byte to request first data block from Arduino
# 12 = number of bytes (three 4-byte floats)
#
def getFloatData(oldFloats):
    try:
        data_received = bus.read_i2c_block_data(address, 1, 16)
        newFloats = [bytes_2_float(data_received, 0)]
        newFloats.append(bytes_2_float(data_received, 1))
        newFloats.append(bytes_2_float(data_received, 2))
        newFloats.append(bytes_2_float(data_received, 3))
    except:
        print("error reading float data")
        newFloats = oldFloats;

    return newFloats

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
# 254 = command byte to initiate writing to Arduino using the joystick
#
def putJoystickByteList(byteList):
    try:
        bus.write_i2c_block_data(address, 254, byteList)
    except:
        print("error writing commands")
    return None

#
# 255 = command byte to initiate writing to Arduino (autonomous)
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

def float_2_bytes(data):
    ba = bytearray(struct.pack("f", data))
    return ba
    
def takeImage(cam, fileName):
    cam.capture(fileName)
    return None

def unwrapAngle(angle):
    if angle > 360:
        angle -= 360
    if angle < 0:
        angle += 360
    return angle
        
################################################SETUP###########################
#
# smbus implements i2c on the RPi
#
bus = smbus.SMBus(1)

#
# this is the Slave address of the Arduino
#
address = 0x04

#
# initialize dummy values of inputs to Pi
#
dummyToPiFloats = [0, 0, 0, 0]

#
#create mapper from .txt files
#
m = mapper.Mapper('mapPennParkNodes.txt', 'mapPennParkEdges.txt')

#
# set up the camera
#
camera = picamera.PiCamera()
photoHeight = 540
camera.resolution = (16*photoHeight/9, photoHeight)

takeImage(camera, 'inttest.jpg')

#
# set up joystick
#
try:
    jc = joyCommander.JoyCommander()
except:
    e = sys.exc_info()[0]
    
joystick = 0
prevCameraTime=time.time()
prevSwitchTime=time.time()
commands = [0,0,0,0,0]

targetHeading = 0
targetVelocity = 0

cameraRate = 0.5

w = 1.5 #width of road in cm
dD = 0 #desired offset from middle of road
kd = 10 #proportional feedback for d
vD = 50 #desired velocity
beta_offset = [0, w/2]
dE = 0
################################################MAIN LOOP#######################


while True:
    time.sleep(0.1)

    #get GPS data from Arduino
    dummyToPiFloats = getFloatData(dummyToPiFloats)
##
    #find current road
    m.findRoad(dummyToPiFloats[0], dummyToPiFloats[1], dummyToPiFloats[2])
    psiMap = m.getRoadHeading()
    vD = m.getSpeedLimit()
    
##    print m.getCurrentRoadName()
##    print "gpsLat:  %s\ngpsLat:  %s\npsiE:    %s\nVE:      %s"%(dummyToPiFloats[0], dummyToPiFloats[1], dummyToPiFloats[2], dummyToPiFloats[3])

    #
    #take picture
    #
    if time.time() >= cameraRate + prevCameraTime and not joystick:
        takeImage(camera, "image.jpg")
        prevCameraTime=time.time()

        try:
            beta_offset = imageProcessing.processImage("image.jpg")
        except:
            e = sys.exc_info()[0]
   
    
    beta = beta_offset[0]
    X_0 = beta_offset[1]
    
    dE = X_0 / abs(X_0) * w / 2 - X_0 * math.cos(beta * math.pi / 180)
    
    psiD = psiMap + (dD - dE) * kd
    psiE_cam = psiMap + beta

    print "beta:     %s"%beta
    print "X_0:      %s"%X_0
    print "dE:       %s"%dE
    print "vE:       %s"%dummyToPiFloats[3]
    #
    #Joystick commands
    #
    try:
        commands = jc.getCommands()
    except:
        commands = [0,0,0,0,0]
        e = sys.exc_info()[0]
    
    if commands[4] and time.time() >= 1 + prevSwitchTime:
        jc.setSpeed(0)
        commands[1]=0
        joystick = not joystick
        print "Joystick? %s"%(joystick)
        prevSwitchTime=time.time()
        #######
        psiD = commands[0]
        vD = commands[1]
        psiECam = 1.2    

        bl=list(float_2_bytes(psiD))+list(float_2_bytes(vD))+list(float_2_bytes(psiE_cam))
        putJoystickByteList(bl)
        
    if commands[2] and time.time() >= 1 + prevCameraTime and joystick:
        takeImage(camera, "controllerpic.jpg")
        prevCameraTime=time.time()
        
    if commands[3]:
        jc.setSpeed(0)
        commands[1]=0
        #######
        psiD = commands[0]
        vD = commands[1]
        psiE_cam = 1.2    

        bl=list(float_2_bytes(psiD))+list(float_2_bytes(vD))+list(float_2_bytes(psiE_cam))
        putJoystickByteList(bl)
    
    #send commands to Arduino
    if joystick:
        psiD = commands[0]
        vD = commands[1]

    if joystick:
        bl=list(float_2_bytes(psiD))+list(float_2_bytes(vD))+list(float_2_bytes(psiE_cam))
        putJoystickByteList(bl)
    else:
        bl=list(float_2_bytes(unwrapAngle(psiD)))+list(float_2_bytes(vD))+list(float_2_bytes(unwrapAngle(psiE_cam)))
        putByteList(bl)

