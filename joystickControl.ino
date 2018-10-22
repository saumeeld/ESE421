// Script to receive joystick commands from Raspberry Pi and
// Control motors and servo accordingly

#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x04
#define motorPin 8 // PWM for motor
#define servoPin 7 // Servo 

// Joystick Hat Mappign
#define UP_HAT 0
#define DOWN_HAT 1
#define LEFT_HAT 2
#define RIGHT_HAT 3
#define UNUSED_HAT 4

// Joystick Button Mapping (CORRESPOND TO INDEX IN PYGAME BUTTON ARRAY)
#define CUP_BUTTON 0
#define CRIGHT_BUTTON 1
#define CDOWN_BUTTON 2
#define CLEFT_BUTTON 3
#define L_BUTTON 4
#define R_BUTTON 5
#define A_BUTTON 6
#define Z_BUTTON 7
#define B_BUTTON 8
#define START_BUTTON 9
#define servoRiggingAngle 54

Servo steeringServo;
byte servoAngleDeg = 90;
byte motorPWM = 150;
byte piCommand;
byte hat;
byte button;

void setup() {
  //
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveDataI2C);
  Wire.onRequest(sendDataI2C);

  Serial.begin(115200);

  //  make the motors not spin
  //
  pinMode(motorPin, OUTPUT);
  analogWrite(motorPin, 0);

  // Set up servo
  steeringServo.attach(servoPin);
  steeringServo.write(servoRiggingAngle);
}


//////////////////////////////////////////////////////////////////
void loop() {
  delay(100);  // update every half second
  //  Serial.print("Hat val is "); Serial.println(hat);
  //  Serial.print("Button val is "); Serial.println(button);
  printHatVal(hat);
  printButtonVal(button);
  steer(hat);
  motorCommand(button);
  Serial.println();
}

void printHatVal(byte hat) {
  switch (hat) {
    case UP_HAT:
      Serial.print("UP HAT ");
      break;
    case DOWN_HAT:
      Serial.print("DOWN HAT ");
      break;
    case LEFT_HAT:
      Serial.print("LEFT HAT ");
      break;
    case RIGHT_HAT:
      Serial.print("RIGHT HAT ");
      break;
    default:
      Serial.print("N/A HAT ");
  }
  Serial.println("PRESSED");
}

void printButtonVal(byte button) {
  switch (button) {
    case CUP_BUTTON:
      Serial.print("CUP BUTTON ");
      break;
    case CDOWN_BUTTON:
      Serial.print("CDOWN BUTTON ");
      break;
    case CLEFT_BUTTON:
      Serial.print("CLEFT BUTTON ");
      break;
    case CRIGHT_BUTTON:
      Serial.print("CRIGHT BUTTON ");
      break;
    case L_BUTTON:
      Serial.print("L BUTTON ");
      break;
    case R_BUTTON:
      Serial.print("R BUTTON ");
      break;
    case A_BUTTON:
      Serial.print("A BUTTON ");
      break;
    case Z_BUTTON:
      Serial.print("Z BUTTON ");
      break;
    case B_BUTTON:
      Serial.print("B BUTTON ");
      break;
    case START_BUTTON:
      Serial.print("START BUTTON ");
      break;
    default:
      Serial.print("N/A BUTTON ");
  }
  Serial.println("PRESSED");
}

void receiveDataI2C(int nPoints) {
  piCommand = Wire.read();

  // if Pi is sending data, parse it into the two command variables
  if (piCommand == 255) {
    hat = Wire.read();
    button = Wire.read();
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

void steer(byte hat) {
  switch (hat) {
    case LEFT_HAT:
      servoAngleDeg -= 3;
      steeringServo.write(constrain(servoAngleDeg, 20, 120));
      break;
    case RIGHT_HAT:
      servoAngleDeg += 3;
      steeringServo.write(constrain(servoAngleDeg, 20, 120));
      break;
    default:
      break;
  }
  Serial.print("Servo Angle: ");
  Serial.println(servoAngleDeg, DEC);
}

void motorCommand(byte button) {
  switch (button) {
    case START_BUTTON:
      motorPWM = 150;
      break;
    case B_BUTTON:
      motorPWM = 0;
      break;
    case CUP_BUTTON:
      motorPWM += 10;
      break;
    case CDOWN_BUTTON:
      motorPWM -= 10;
      break;
    default:
      break;
  }
  analogWrite(motorPin, constrain(motorPWM,0,250));
  Serial.print("Motor PWM: ");
  Serial.println(motorPWM, DEC);
}

