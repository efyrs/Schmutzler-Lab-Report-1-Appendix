long oldPosition  = -999; //store previous position of encoder
int count = 0; //encoder count initialised to 0
float distance;
 
void setup() {
  //initialise serial communication
  Serial.begin(9600);
 
 
  myservo.write(70);
  myservo.attach(servoPin);  //attach our servo object to pin D4
  //the Servo library takes care of defining the PinMode declaration (libraries/Servo/src/avr/Servo.cpp line 240)
 
  //configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);  
 
  motors(255, 178);
}
 
void loop() {
  goBackwards(); //moving backwards meant the car went straighter
  long newPosition = myEnc.read(); //myEnc.read reads the new position
 
  if (newPosition != oldPosition) //updating position
  {    
    oldPosition = newPosition;
 
    count += 1; //encoder count increases
