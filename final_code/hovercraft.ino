#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 imu = Adafruit_BNO055();
Servo myservo;  // create servo object to control a servo
float ref_angle;
void setup() {
  // pin of the servo
  myservo.attach(13);
  // pin of the thrust fan  
  pinMode(12, OUTPUT);  
  // lift fan pin
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  // pin of the front sensor 
  pinMode(11, INPUT);  
  // PIN of the side sensor
  pinMode(10, INPUT);         
  // we need to figure out the pins that the fans and servos are connected to
  Serial.begin(9600);
    // imu setup and initialisation
  if (!imu.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  servo.attach(servoPin);
  servo.write(servoPosition);

   // Set I2C communication speed for the imu
  Wire.begin();
  Wire.setClock(400000);
  
}

void loop() {
  // main logic loop 
  int value_front;
  int value_side;
  int threshold_front;
  // the sensor is on the left side of the hovercraft
  int threshold_side;
  float angle;
  if(value_front > threshold_front){
    // go straight
    Straigth();
    }
  else if ((value_front < threshold_front)&&(value_side > threshold_side)) {

    // turn left 
    Turn_Left(angle);
    ref_angle += 90;
    }

  else if ((value_front < threshold_front)&&(value_side < threshold_side)) {

    // turn right 
    Turn_Right(angle);
    ref_angle -= 90;
    }
  Stabilize(angle);
  delay(1000);
}

// goes vroom
void Straigth()
{
  
  digitalWrite(12, 5);
  
  }

// takes the value of the angle of the hover carft and turns the servo at the required angle
void Turn_Right(float angle){
  // take the angle given and turn the servo 90 degrees and keep it at 
  //90 degrees until the imu reeds a value of 90 degrees bigger than the original angle 
  // store the original value of the angle
  float angle = get_Angle();
  while(get_Angle() < angle + 90){
    // turn servo 90 degrees in the right direction
    myservo.write(180);
    }
  // put the servo in the neutral position
    myservo.write(90);
  }


// takes the value of the angle of the hover carft and turns the servo at the required angle
void Turn_Left(float angle){
  
  // take the angle given and turn the servo 90 degrees and keep it at 
  //90 degrees until the imu reeds a value of 90 degrees bigger than the original angle 
  // store the original value of the angle
  float angle = get_Angle();
  while(get_Angle() > angle - 90){
    // turn servo 90 degrees in the right direction
    myservo.write(0);
    }
    // put the servo in the neutral position
    myservo.write(90);
  }


// takes in the angle from the imu and keeps the hover board straigth
void Stabilize(float angle){
    // if the hovercraft tilts to the left turn the fan to the right
    // calculate the value of the angle to give the servo using the yaw angle of the imu
    // use the side sensor to keep a constant distance with the wall 
    


 }
// return the angle of the hover craft 
float get_Angle(){
    sensors_event_t event;
    imu.getEvent(&event);
    float yaw = event.orientation.z;
  return yaw;
  }


  // TODO 
  // verify that the angle in the turn functions