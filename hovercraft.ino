// April 13th, Jeffrey
// Changes:
// Added while loop for continuous stabilization
// If statement brace was not properly closed in previous codes
// Delay for 5 ms at the end of stabilize to settle servo motor
// angleZ() instead of angleY()
// 2:55 PM last changed

#include <MPU6050_light.h>
#include <Wire.h>
#include <Servo.h>


#define triggerPinLeft 11
#define echoPinLeft 2

#define triggerPinFront 13
#define echoPinFront 3

const int liftFanPin = 5;
const int thrustFanPin = 6;

const int servoPin = 9;
Servo myServo;

const int threshold_front = 20;
const int threshold_side = 40;

int value_front = 0;
int value_side = 0;
// MPU object
MPU6050 mpu(Wire);
// variables to store IMU data
float rollValue;   // don't need
float pitchValue;  // don't need
float yawValue;

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;


// variable to store the reference angle for stabilization
float refAngle = 0.0;

// Function prototypes
float getDistance(int triggerPin, int echoPin);
float getAngle();
void stabilize();
void shutDown();
void startUp();
void turnRight();
void turnLeft();


void setup() {

  Serial.begin(9600);

  pinMode(triggerPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);

  pinMode(triggerPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);

  digitalWrite(liftFanPin, LOW);
  digitalWrite(thrustFanPin, LOW);

  myServo.attach(servoPin);
  myServo.write(90);

  Wire.begin();
  byte status = mpu.begin();

  Wire.setWireTimeout(3000, true);
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);  // gyro and accelero
  Serial.println("Done!\n");
  refAngle = getAngle();  // Get the initial reference angle
  delay(2000);  // Wait for sensors to settle
}


void loop() {
  mpu.update();
  stabilize();  // Call the stabilize function ///// SHOULD BE IN A WHILE LOOP TO KEEP STABILIZING CAUSE DOING IT ONCE
  value_front = getDistance(triggerPinFront, echoPinFront);
  value_side = getDistance(triggerPinLeft, echoPinLeft);
  Serial.print("outside");
  Serial.print("Front = ");
  Serial.println(value_front);
   Serial.print("Side = ");
   Serial.println(value_side);

  if (value_front <= threshold_front) {
    shutDown();
    //mpu.update();
    delay(3000);
    value_side = getDistance(triggerPinLeft, echoPinLeft);
    // Obstacle detected, avoid it
    if (value_side > threshold_side) {  // turn left
    Serial.print("turn left");
      int YAW = getAngle();
      Serial.println(YAW);
      if (-45<= YAW && YAW < 45) {
        turnLeft();
        bool panic = false;
        long int stuckStartTime = millis();
        while (YAW <= 70 ) {
          if (isPanic()) {
              if (millis() - stuckStartTime > 2000) {
                break;
              }
            } else {
              stuckStartTime = millis();
            }
          YAW = getAngle();
        }
        shutDown();
        delay(3000);
      } else if (45 <= YAW && YAW < 135) {
        turnLeft();
        bool panic = false;
        long int stuckStartTime = millis();
        while (YAW <= 160 ) {
          if (isPanic()) {
              if (millis() - stuckStartTime > 2000) {
                break;
              }
            } else {
              stuckStartTime = millis();
            }
          YAW = getAngle();
        }
        shutDown();
        delay(3000);
      } else if (135 <= YAW || YAW < -135) {
        turnLeft();
        long int stuckStartTime = millis();
        bool panic = false;
        while (!(-105 <= YAW && YAW < 0) ) {
          if (isPanic()) {
              if (millis() - stuckStartTime > 2000) {
                break;
              }
            } else {
              stuckStartTime = millis();
            }
          YAW = getAngle();
        }
        shutDown();
        delay(3000);
      } else if (-45 > YAW && YAW >= -135) {
        turnLeft();
        long int stuckStartTime = millis();
        bool panic = false;
        while (YAW <= -15 ) {
          if (isPanic()) {
              if (millis() - stuckStartTime > 2000) {
                break;
              }
            } else {
              stuckStartTime = millis();
            }
          YAW = getAngle();
        }
        shutDown();
        delay(3000);
      } else {
        Serial.println("Nothing to apply");
      }

    } else {  // turn right
      Serial.print("turn right");
      int YAW = getAngle();
      Serial.println(YAW);
      if (-45 <= YAW && YAW <=45) {
        turnRight();
        long int stuckStartTime = millis();
        bool panic = false;
        while (YAW >= -70) {
           if (isPanic()) {
              if (millis() - stuckStartTime > 2000) {
                break;
              }
            } else {
              stuckStartTime = millis();
            }
          YAW = getAngle();
        }
        shutDown();
        delay(3000);
      } else if (45 < YAW && YAW <= 135) {
        turnRight();
        long int stuckStartTime = millis();
        bool panic = false;
        while (YAW >= 15) {
           if (isPanic()) {
              if (millis() - stuckStartTime > 2000) {
                break;
              }
            } else {
              stuckStartTime = millis();
            }
          YAW = getAngle();
        }
        shutDown();
        delay(3000);
      } else if (135 <= YAW || YAW < -135) {
        turnRight();
        long int stuckStartTime = millis();
        bool panic = false;
        while (!(YAW > 0 && YAW <= 105)) {
           if (isPanic()) {
              if (millis() - stuckStartTime > 2000) {
                break;
              }
            } else {
              stuckStartTime = millis();
            }
          YAW = getAngle();
        }
        shutDown();
        delay(3000);
      } else if (-45 > YAW && YAW >= -135) {
        turnRight();
        bool panic = false;
        long int stuckStartTime = millis();
          while (YAW <= -160) {
            if (isPanic()) {
              if (millis() - stuckStartTime > 2000) {
                break;
              }
            } else {
              stuckStartTime = millis();
            }
          YAW = getAngle();
        }
        shutDown();
        delay(3000);
      } else {
        Serial.println("Nothing to apply");
      }
    }
    
    value_front = getDistance(triggerPinFront, echoPinFront);
    Serial.print("Front = ");
    Serial.println(value_front);
    Serial.print("Side = ");
    Serial.println(value_side);
  }
  else{
      // go straight
      Go_Straight(value_front);
    }
}







// Function to calculate the distance from the ultrasonic sensor
float getDistance(int triggerPin, int echoPin) {
  long duration, distance;

  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.0343;
  return distance;
}

float getAngle() {
  mpu.update();
  return mpu.getAngleZ();
}


// Function to stabilize the hovercraft using the servo motor
void stabilize() {

  if (getAngle() >= -50 && getAngle() <= 50) {
    float angle = 89 + getAngle();
    //Serial.println(angle);
    myServo.write(angle);
    delay(10);
  } else if (getAngle() > 50 && getAngle() < 130){
    float angle = getAngle();
    myServo.write(angle);
    delay(10);
  }else if(getAngle() < -50 && getAngle() >-130){
    float angle = getAngle();
    myServo.write(180 + getAngle());
    delay(10);
  }else if (getAngle() >= 130) {
    float angle = 90 - (180 - getAngle());
    //Serial.println(angle);
    myServo.write(angle);
    delay(10);
  } else if (getAngle() <= -130) {
    float angle  = 90 + (180 + getAngle());
    myServo.write(angle);
    delay(10);  // delay by few milliseconds to avoid overwhelming the servo motor with too many commands too quicky
  }
}

void shutDown() {
  digitalWrite(liftFanPin, LOW);
  digitalWrite(thrustFanPin, LOW);
}

void startUp() {
  analogWrite(liftFanPin, 255);
  analogWrite(thrustFanPin, 230);
}

void turnRight() {
  analogWrite(liftFanPin, 255);
  analogWrite(thrustFanPin, 255);
  myServo.write(180);
}

void turnLeft() {
  analogWrite(liftFanPin, 255);
  analogWrite(thrustFanPin, 255);
  myServo.write(0);
}

void Go_Straight(float distance){
    
  if (60 <= distance && distance < 65){
    Serial.println("slowing down");
    analogWrite(liftFanPin, 0);
    delay(500);
  }
  else if (distance <= 65 && mpu.getAccAngleX() < 0.5){
    startUp();
  }
  else{
    // puts every thing on max settings
    startUp();
    }
}

boolean isPanic(){
  if(mpu.getAccAngleX()<0.02 && mpu.getAccAngleY() < 0.02){
    return true; 
  }
  return false;
}

