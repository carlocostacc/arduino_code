#include <MPU6050_light.h>
#include <Wire.h>
#include <Servo.h>

#define triggerPinLeft 11
#define echoPinLeft 2

#define triggerPinFront 13
#define echoPinFront 3


//Gyro Variables
// time 
float elapsedTime, time, timePrev;  

//We use this variable to only calculate once the gyro data error
int gyro_error=0;          

//raw imu data
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;      

//gyro angles
float Gyro_angle_x, Gyro_angle_y, Gyro_angle_z;       

// gyro error
float Gyro_raw_error_x, Gyro_raw_error_y, Gyro_raw_error_y; 

//Acc Variables

// acceleration error 
int acc_error=0;          

// converter rad to degrees 
float rad_to_deg = 180/3.141592654;      

//raw acceleration values
float Acc_rawX, Acc_rawY, Acc_rawZ;    

//acceleration data
float Acc_angle_x, Acc_angle_y, Acc_angle_z;     

// acceleration angle error
// dont know what it does but here it is
float Acc_angle_error_x, Acc_angle_error_y,  Acc_angle_error_z; 

// final values for the angles
float Total_angle_x, Total_angle_y, Total_angle_z;


const int liftFanPin = 5;
const int thrustFanPin = 6;

const int servoPin = 9;
Servo myServo;

const int threshold_front = 20;
const int threshold_side = 20;

int value_front = 0;
int value_side = 0;

// Function prototypes
float getDistance(int triggerPin, int echoPin);
float getAngle();
void stabilize();

void setup() {

  
 
  pinMode(triggerPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);

  pinMode(triggerPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);

  digitalWrite(liftFanPin, LOW);
  digitalWrite(thrustFanPin, LOW);

  Wire.begin();                           
  
  // talking to register addr 68, this is the imu register
  Wire.beginTransmission(0x68);                         
  Wire.write(0x6B);                       
  Wire.write(0x00);
  Wire.endTransmission(true); 

  //Gyro config
  
  Wire.beginTransmission(0x68);           
  Wire.write(0x1B);                       
  Wire.write(0x10);                       
  Wire.endTransmission(true);             


  //Acc config
  Wire.beginTransmission(0x68);           
  Wire.write(0x1C);                       
  Wire.write(0x10);                       
  Wire.endTransmission(true); 

  // general config
  Serial.begin(9600);
  time = millis();


// calculating acceleration data befor the main loop
  if(acc_error==0)
  {
    for(int a=0; a<200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true); 
      
      Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ;
      Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
      Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;

      
      /*---X---*/
      Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg));
      /*---Y---*/
      Acc_angle_error_y = Acc_angle_error_y + ((atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg)); 
      /*---Z---*/
      // there might be an error here 
      Acc_angle_error_z = Acc_angle_error_z + ((atan(-1*(Acc_rawZ)/sqrt(pow((Acc_rawZ),2) + pow((Acc_rawX),2)))*rad_to_deg)); 
      

      // filtering for error drift of the imu
      if(a==199)
      {
        Acc_angle_error_x = Acc_angle_error_x/200;
        Acc_angle_error_y = Acc_angle_error_y/200;
        Acc_angle_error_z = Acc_angle_error_z/200;
        acc_error=1;
      }
    }
  } 

  if(gyro_error==0)
  {
    for(int i=0; i<200; i++)
    {
      Wire.beginTransmission(0x68); 
      Wire.write(0x43);                       
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true); 
         
      Gyr_rawX=Wire.read()<<8|Wire.read();   
      Gyr_rawY=Wire.read()<<8|Wire.read();
      Gyr_rawZ=Wire.read()<<8|Wire.read();
   
      /*---X---*/
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX/32.8); 
      /*---Y---*/
      Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY/32.8);
      /*---Z---*/
      Gyro_raw_error_z = Gyro_raw_error_z + (Gyr_rawZ/32.8);
      if(i==199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x/200;
        Gyro_raw_error_y = Gyro_raw_error_y/200;
        Gyro_raw_error_z = Gyro_raw_error_z/200;
        gyro_error=1;
      }
    }
  } 
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
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;

  //////////////////////////////////////Gyro read/////////////////////////////////////

    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);
        
    Gyr_rawX=Wire.read()<<8|Wire.read();
    Gyr_rawY=Wire.read()<<8|Wire.read();
    Gyr_rawZ=Wire.read()<<8|Wire.read();
    

    /*---X---*/
    Gyr_rawX = (Gyr_rawX/32.8) - Gyro_raw_error_x; 
    /*---Y---*/
    Gyr_rawY = (Gyr_rawY/32.8) - Gyro_raw_error_y;
    /*---Z---*/
    Gyr_rawZ = (Gyr_rawZ/32.8) - Gyro_raw_error_z;
    
 
    /*---X---*/
    Gyro_angle_x = Gyr_rawX*elapsedTime;
    /*---Y---*/
    Gyro_angle_y = Gyr_rawY*elapsedTime;
    /*---Z---*/
    Gyro_angle_z = Gyr_rawZ*elapsedTime;


        
    
    //////////////////////////////////////Acc read/////////////////////////////////////

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true); 
        
    Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ;
    Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
    Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ; 
    
    /*---X---*/
    Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_x;
    /*---Y---*/
    Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_y;    
    /*---Z---*/
    Acc_angle_z = (atan(-1*(Acc_rawY)/sqrt(pow((Acc_rawZ),2) + pow((Acc_rawX),2)))*rad_to_deg) - Acc_angle_error_z;    

    //////////////////////////////////////Total angle and filter/////////////////////////////////////
    /*---X axis angle---*/
    Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
    /*---Y axis angle---*/
    Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;
    /*---Z axis angle---*/
    Total_angle_z = 0.98 *(Total_angle_z + Gyro_angle_z) + 0.02*Acc_angle_z;

    
    
    
    Serial.print("Zº: ");
    Serial.print(Total_angle_z);
    Serial.println(" ");
    return Total_angle_z;
}

// Function to stabilize the hovercraft using the servo motor
void stabilize() {
  float Gabrielle = 90 - mpu.getAngleY();
  Serial.println(Gabrielle);
  int Nadim = Gabrielle;
  myServo.write(Nadim);
}