#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>

// Initialize IMU and Servo objects
Adafruit_BNO055 imu = Adafruit_BNO055();
Servo servo;

// Set servo motor pin and initial position
int servoPin = 9;
int servoPosition = 90;

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Initialize IMU and Servo
  if (!imu.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  servo.attach(servoPin);
  servo.write(servoPosition);

  // Set I2C communication speed
  Wire.begin();
  Wire.setClock(400000);
}

void loop() {
  // Read Euler angles from IMU
  sensors_event_t event;
  imu.getEvent(&event);
  float roll = event.orientation.x;
  float pitch = event.orientation.y;
  float yaw = event.orientation.z;

  // Print Euler angles to serial monitor
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("  Pitch: ");
  Serial.print(pitch);
  Serial.print("  Yaw: ");
  Serial.println(yaw);

  // Control servo motor based on pitch angle
  if (pitch > 0) {
    servoPosition = 180;
  } else {
    servoPosition = 0;
  }
  servo.write(servoPosition);

  // Wait for a short time to prevent overloading the I2C bus
  delay(10);
}
