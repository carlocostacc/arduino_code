// April 15th, Amar
// Changes:
// Added while loop for continuous stabilization
// If statement brace was not properly closed in previous codes
// Delay for 5 ms at the end of stabilize to settle servo motor
// angleZ() instead of angleY()
// 5:10 PM last changed
// More preferable code that Simon wants to change 

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

const int threshold_front = 20 ;
const int threshold_side = 60;

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
int getAngle();
void stabilize();
void shutDown();
void startUp();
void rotateServoRight();
void rotateServoLeft();
void turnRight(float initialYAW);
void turnLeft(float initialYAW);
boolean isPanic(float previousyaw);
bool isRotating(float initialYAW, bool isRotatingRight);


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
  Serial.println(("Calculating offsets, do not move MPU6050"));
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
  print(value_front, value_side);
  if (value_front <= threshold_front) {
    shutDown();
    //mpu.update();
    delay(1000);
    value_side = getDistance(triggerPinLeft, echoPinLeft);
    // Obstacle detected, avoid it
    if (value_side > threshold_side) {  // turn left
      Serial.println("");
      Serial.print("turn left");
      int YAW = getAngle();
      Serial.println(YAW);
      Serial.println("");
      turnLeft(YAW);
    } else {  // turn right
      Serial.println("");
      Serial.print("turn right");
      int YAW = getAngle();
      Serial.println(YAW);
      Serial.println("");
      turnRight(YAW);
    }
    
    value_front = getDistance(triggerPinFront, echoPinFront);

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

int getAngle() {
  mpu.update();
  float angle = (mpu.getAngleZ());
  angle = (int)angle%360;
  if(angle < 0){
    angle = 360 + angle;
    }
  return (angle);
}


// Function to stabilize the hovercraft using the servo motor
void stabilize() {
  Serial.println(getAngle());
  if (getAngle() >= 300) {
    float angle = 90 -(360- getAngle());
    //angle = int(angle)%360;
    Serial.println("");
    Serial.println("Stabilize 0");
    Serial.println("");
    myServo.write(angle);
    delay(10);
  }else if (getAngle() <= 60) {
    float angle = 90 + getAngle();
    angle = int(angle)%360;
    Serial.println("");
    Serial.println("Stabilize 0");
    Serial.println("");
    myServo.write(angle);
    delay(10);
  } else if (getAngle() > 60 && getAngle() < 120){
    float angle = getAngle();
    angle = int(angle)%360;
    Serial.println("");
    Serial.println("Stabilize 90");
    Serial.println("");
    myServo.write(angle);
    delay(10);
  }else if(getAngle() < 300 && getAngle() >240){
    float angle = getAngle();
    angle = int(angle)%360;
    angle = 180 - (360 - angle);
    Serial.println("");
    Serial.println("Stabilize -90");
    Serial.println("");
    myServo.write(angle);
    delay(10);
  }else if (getAngle() <= 240 && getAngle() >= 120) {
    float angle = 90 - (180 - getAngle());
    angle = int(angle)%360;
    //Serial.println(angle);
    Serial.println("");
    Serial.println("Stabilize 180");
    Serial.println("");
    myServo.write(angle);
    delay(10);
    }

}

void shutDown() {
  Serial.println("SHUT DOWN!");
  digitalWrite(liftFanPin, LOW);
  digitalWrite(thrustFanPin, LOW);
}

void startUp() {
  analogWrite(liftFanPin, 255);
  analogWrite(thrustFanPin, 220);
}

void rotateServoRight() {
  analogWrite(liftFanPin, 255);
  analogWrite(thrustFanPin, 175);
  myServo.write(163);
}

void rotateServoLeft() {
  analogWrite(liftFanPin, 255);
  analogWrite(thrustFanPin, 175);
  myServo.write(17);
}

void Go_Straight(float distance){
    
  if (75 <= distance && distance < 83){
    analogWrite(liftFanPin, 0);
    delay(1000);
  }
  else{
    // puts every thing on max settings
    startUp();
    }
}

boolean isPanic(float previousyaw) {
  
  return abs(getAngle() - previousyaw) <= 1;
}

bool isRotating(float initialYAW, bool isRotatingRight) {
  float YAW = getAngle();
  if (315 <= initialYAW) {
    Serial.println("");
    Serial.println("315 <= initialYAW || initialYAW < 45");
    Serial.println("=======YAW=======");
    Serial.println(YAW);
    Serial.println("=================");
    Serial.println("");
    return isRotatingRight ? YAW >= 290 : YAW <= 70;
  } if (initialYAW < 45) {
    Serial.println("");
    Serial.println("315 <= initialYAW || initialYAW < 45");
    Serial.println("=======YAW=======");
    Serial.println(YAW);
    Serial.println("=================");
    Serial.println("");
    return isRotatingRight ? YAW >= 290 : YAW <= 70;
  }
  else if (45 <= initialYAW && initialYAW < 135) {
    Serial.println("");
    Serial.println("45 <= initialYAW && initialYAW < 135");
    Serial.println("=======YAW=======");
    Serial.println(YAW);
    Serial.println("=================");
    Serial.println("");
    return isRotatingRight ? YAW >= 15 : YAW <= 165;
  } else if (135 <= initialYAW && initialYAW < 225) {
    Serial.println("");
     Serial.println("135 <= initialYAW && initialYAW < 225");
     Serial.println("=======YAW=======");
    Serial.println(YAW);
    Serial.println("=================");
    Serial.println("");
    return isRotatingRight ? YAW >= 110 : YAW <= 250;
  } else if (315 > initialYAW && initialYAW >= 225) {
    Serial.println("");
    Serial.println("315 > initialYAW && initialYAW >= 225");
    Serial.println("=======YAW=======");
    Serial.println(YAW);
    Serial.println("=================");
    Serial.println("");
    return isRotatingRight ? YAW >= 195 : YAW <= 350;
  }
  return false;
}
void turnRight(float initialYAW) {
  Serial.println("");
  Serial.println("right");
  Serial.println("");
  float YAW = initialYAW;
  rotateServoRight();
  long int stuckStartTime = millis();
  while (isRotating(initialYAW, true)) {
      if (isPanic(YAW)) {
        if (millis() - stuckStartTime > 4000) {
          break;
        }
      } else {
        stuckStartTime = millis();
        YAW = getAngle();
      }
      
  }
    shutDown();
    delay(500);
    startUp();
    stabilize();
}

void turnLeft(float initialYAW) {
  Serial.println("");
  Serial.println("left");
  Serial.println("");
  float YAW = initialYAW;
  rotateServoLeft();
  long int stuckStartTime = millis();
  while (isRotating(initialYAW, false)) {
      if (isPanic(YAW)) {
        
        if (millis() - stuckStartTime > 3000) {
          
          break;
        }
      } else {
        stuckStartTime = millis();
        YAW = getAngle();
      }
      
  }
    shutDown();
    delay(500);
    startUp();
    stabilize();
  
}


void print(float front, float side){
  Serial.println("");
  Serial.println("=======DISTANCE_VALUES======");
  Serial.println("front and side values");
  Serial.print(front);
  Serial.print(", ");
  Serial.print(side);
  Serial.print(" ");
  Serial.println("");
  Serial.println("=======================");
  Serial.println("==========ANGLE===========");
  Serial.print("angle = ");
  Serial.print(getAngle());
  Serial.println("========================");
  Serial.println("");
  }


void Wiggle(){
  // we want to get unstuck if all else fails 
  }