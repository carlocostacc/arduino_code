#include <Servo.h>
Servo myservo;  // create servo object to control a servo
void setup() {
  // pin of the servo
  myservo.attach(13);
  // pin of the thrust fan  
  pinMode(12, OUTPUT);  
  // lift fan pin
  pinMode(9, OUTPUT); 
  // pin of the front sensor 
  pinMode(11, INPUT);  
  // PIN of the side sensor
  pinMode(10, INPUT);         
  // we need to figure out the pins that the fans and servos are connected to
  Serial.begin(9600);
  
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
    }

  else if ((value_front < threshold_front)&&(value_side < threshold_side)) {

    // turn right 
    Turn_Right(angle);
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
  float angle = getAngle();
  while(getAngle() < angle + 90){
    // turn servo 90 degrees in the right direction
    myservo.write(some value);
    }
  // put the servo in the neutral position
    myservo.write(neutral value);
  }


// takes the value of the angle of the hover carft and turns the servo at the required angle
void Turn_Left(float angle){
  
  // take the angle given and turn the servo 90 degrees and keep it at 
  //90 degrees until the imu reeds a value of 90 degrees bigger than the original angle 
  // store the original value of the angle
  float angle = getAngle();
  while(getAngle() > angle - 90){
    // turn servo 90 degrees in the right direction
    myservo.write(some value);
    }
    // put the servo in the neutral position
    myservo.write(neutral value);
  }


// takes in the angle from the imu and keeps the hover board straigth
void Stabilize(float angle){
  
 }
// return the angle of the hover craft 
float getAngle(){
  
  return 0.04
  }