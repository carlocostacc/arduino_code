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
const int threshold_side = 20;

int value_front = 0;
int value_side = 0;
// MPU object
MPU6050 mpu(Wire);
// variables to store IMU data
float rollValue; 
float pitchValue; 
float yawValue;

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
//gyroZ = gyroZ + 0.79; // GyroErrorZ ~ (-0.8)
//float gyroAngleY = gyroAngleY + GyroY * elapsedTime;
//float yaw =  yaw + GyroZ * elapsedTime;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

struct AccelData {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct GyroData {
  int16_t x;
  int16_t y;
  int16_t z;
};

// variable to store the reference angle for stabilization
float refAngle = 0.0;

// Function prototypes
float getDistance(int triggerPin, int echoPin);
float getAngle();
void stabilize();

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
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  refAngle = getAngle(); // Get the initial reference angle

  delay(3000); // Wait for sensors to settle 
}

void loop() {
  //setup();
  mpu.update();
  digitalWrite(liftFanPin, HIGH);
  Serial.print("Yaw = ");
  Serial.println(getAngle());
  analogWrite(thrustFanPin, 160); // Set the thrust fan speed to maximum
  stabilize(); // Call the stabilize function
  float Amar = getAngle();
  value_front = getDistance(triggerPinFront, echoPinFront);
  if (value_front < threshold_front) {
    digitalWrite(liftFanPin, LOW);
    delay(500);
    value_side = getDistance(triggerPinLeft, echoPinLeft);
    if (value_side > threshold_side) { //to turn left
      digitalWrite(liftFanPin, HIGH);
      while (Amar<75){
          myServo.write(30);
          Amar = getAngle();
          
      }
      myServo.write(90); // Stop the servo
    }
    else if (value_side < threshold_side){ //turn right
      digitalWrite(liftFanPin, HIGH);
      while ( Amar < -75){
          myServo.write(150);
          Amar = getAngle();
      }
      myServo.write(90); // Stop the servo
    }
    else {
      //myServo.write(90);
      stabilize();
    }
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

// Function to calculate the reference angle for stabilization
float getAngle() {
  // Read the accelerometer values
  return mpu.getAngleY();
}

// Function to stabilize the hovercraft using the servo motor
void stabilize() {
  float Gabrielle = 90 - mpu.getAngleY();
  Serial.println(Gabrielle);
  int Nadim = Gabrielle;
  myServo.write(Nadim);
}
