// C++ Arduino for the final project
//

#include <Wire.h>
#include <I2Cdev.h> // library for the mpu6050
#include <MPU6050.h> // library for the imu6050
#include <Servo.h> // library for the servo motor


#define triggerPinLeft 11 // P6 for left sensor
#define echoPinLeft 2

#define triggerPinFront 13 // P13 for front sensor
#define echoPinFront 3

// define pin numbers for the lift and thrust fans
const int liftFanPin = 5; // lift fan connected to P3
const int thrustFanPin = 6; // thrust fan connected to P4

// define pin numbers for the servo motor and a servo object 
const int servoPin = 9; // servo motor connected to P11
Servo myServo;

const int threshold_front = 10;
int value_front = 0; // distance read by the front sensor

const int threshold_side = 10; 
int value_side = 0; // distance read by the side sensor (LEFT)

float refAngle = 0.00;

// IMU object
MPU6050 mpu; 

void setup() {
  // put your setup code here, to run once:

  // Set pin modes for the left and front sensors 
  pinMode(triggerPinLeft, OUTPUT); // output a voltage
  pinMode(echoPinLeft, INPUT); // read input 

  pinMode(triggerPinFront, OUTPUT); // output a voltage
  pinMode(echoPinRight, INPUT); // read input
  Serial.begin(9600); // begin serial communication at 9600 baud 

  // INITIALIZE FANS
  digitalWrite(liftFanPin, LOW);
  digitalWrite(thrustFanPin, LOW);

  // INITIALIZE SERVO
  myServo.attach(servoPin);
  analogWrite(servoPin, 90); // initial position of the servo motor

  // INITIALIZE IMU
  Wire.begin();
  mpu.initialize();
  Serial.println("MPU-6050 Test");
  Serial.println("----------");
  
  // test is IMU connection is good 
  if (mpu.testConnection()) {
    Serial.println("MPU-6050 connection successful");
  }
  else {
    Serial.println("MPU-6050 connection failed");
    while(1);
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // maximum readable acceleration is +/- 2g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500); // maximum readable angular velocity is +/- 500 dps

  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:

  // *********************************************************************
  // control lift fan
  analogWrite(liftFanPin, 255); // set lift fan speed to maximum
  delay(5000); // wait 5s
  analogWrite(thrustFanPin, 0); // turn off lift fan 
  delay(5000); // wait 5s 

  // control thrust fan
  digitalWrite(thrustFanPin, HIGH); // turn on thrust fan
  delay(1000); // wait 1s
  digitalWrite(thrustFanPin, LOW); // turn off thrust fan
  delay(1000); // wait 1s 

  // control servo motor
  analogWrite(servoPin, 180); // turn servo one end
  delay(1000); // wait 1s
  analogWrite(servoPin, 0); // turn servo the other end
  // *********************************************************************

  // MOVE FORWARD (activate fans)
  digitalWrite(liftFanPin, HIGH);
  digitalWrite(thrustFanPin, HIGH);

  // CHECK DISTANCE AHEAD
  value_front = getDistance(triggerPinFront, echoPinFront);
  if (value_front < threshold_front) { // if (distance read by front sensor < threshold front)
    // Obstacle detected, avoid it
    value_side = getDistance(triggerPinLeft, echoPinLeft); // Check distance read by left sensor
    if (value_side > threshold_side) {
      // turn left
      analogWrite(servoPin, 180);
    }
    else {
      // turn right
      analogWrite(servoPin, 0);
    }
  }
  
  // No obstacle detected, keep moving forward
  else { 
    analogWrite(servoPin, 90);
  }

  // Helper function to calculate distance for a US sensor 
  float getDistance(int triggerPin, int echoPin) {
    long duration, distance;

    // Trigger the measurement by sending a 10 us pulse
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Read the duration of the echo pulse
    duration = pulseIn(echoPin, HIGH);

    // Calculate the distance based on the speed of sound
    distance = (duration / 2) * 0.0343;
    return distance;

  }


}