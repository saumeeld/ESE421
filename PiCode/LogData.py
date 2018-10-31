import i2c_lib

#Define Arduino slave pin address for i2c
I2C_ARDUINO_SLAVE_ADDRESS = 0x04

def getYawRateFromArduino():
	getFloatData(yawRate, I2C_ARDUINO_SLAVE_ADDRESS)

def getPsiEstimatedFromArduino():
	getFloatData(psiEstimated, I2C_ARDUINO_SLAVE_ADDRESS)

def main():
	#Initalize variables to read
	yawRate = [None]
	psiEstimated = [None]

	#TODO SEND DATA THAT IS GREATER THAN 255 OR LESS THAN 0
	getYawRateFromArduino()
	getPsiEstimatedFromArduino()
    
if __name__ == "__main__":
    main()


	
