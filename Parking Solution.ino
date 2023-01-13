#include <Wire.h> //includes wire library and allows communication with i2c devices
#include <math.h> //includes maths library
#include <MPU6050_tockn.h> //includes MPU library
#include <Encoder.h> //includes encoder library
#include <Servo.h> //includes servo library

MPU6050 mpu6050(Wire);

#define I2C_SLAVE_ADDR 0x04 //device #4
#define trigPin 26 // esp32 pin 26 connected to hc-sr04 trigger pin
#define echoPin 25 //esp32 pin 25 connected to hc-sr04 echo pin

//initialising variables
int angle = 0;
int distance = 999;
long duration;
float distancecm;
int leftMotor_speed, rightMotor_speed, servoAngle;
int x = 0;

void setup() //initialising components
{
  Serial.begin(9600); //serial communication starts at baud rate 9600- transfers 9600 bits per second
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  mpu6050.begin(); //initialises mpu
  mpu6050.calcGyroOffsets(true); //calculates gyro offset
  pinMode(trigPin, OUTPUT); //sets trigPin as output
  pinMode(echoPin, INPUT); //sets echoPin as input
}

void loop()
{
  //Forward 1 second
  leftMotor_speed = 255;
  rightMotor_speed = 178;
  servoAngle = 70;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);

  //stopping car
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 70;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);

  //Rotate 180 degrees
  leftMotor_speed = 150;
  rightMotor_speed = 55;
  servoAngle = 100;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  while (angle > -180) {
    angle = accelerometer();
  }

  //stopping again
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 70;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);

  //reverse until 10cm away from wall
  leftMotor_speed = -100;
  rightMotor_speed = -70;
  servoAngle = 70
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  do {
    distance = ultrasonic();
    delay(1000);
  }
  while (distance >= 10);
  distance = 999;

  //stopping again
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 70;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);

  //Rotate 90 degrees
  leftMotor_speed = 50;
  rightMotor_speed = 150;
  servoAngle = 70;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  while (angle < -90) {
    angle = accelerometer();
  }

  //stop
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 70;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);

  //reverse until 10cm from wall
  leftMotor_speed = -100;
  rightMotor_speed = -70;
  servoAngle = 70;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  do {
    distance = ultrasonic();
    delay(1000);
  }
  while (distance >= 10);
  distance = 999;

  //stop
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 70;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(10000000000000000); //very long delay, likely to stop the program from running as there is no movement
}


void Transmit_to_arduino(int leftMotor_speed, intrightMotor_speed, int servoAngle)

{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));   // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));          // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));
  Wire.endTransmission();   // stop transmitting
}

float accelerometer() //reads and saves mpu angle
{
  mpu6050.update();
  return (mpu6050.getAngleZ());
}
 
float ultrasonic() //reads and saves measured distance
{
  float distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); //pauses for 2 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); //pauses for 10 microseconds
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return (duration * 0.034 / 2); //returns distance with duration*speed of sound
}