# Ball-and-Beam
Arduino is controlling a nonlinear motion control system, named ball and beam or Cart and beam


// note: for my beam to be horizontal, Servo Motor angle should be 50 degrees.
#include<Servo.h>
#include<PID_v1.h>

const int trigPin = 13;    // 13 Trig pin of the ultrasonic sensor (Yellow)
const int echoPin = 12;   // 12 Echo pin of the ultrasonic sensor (Orange)
const int servoPin = 11;  // Servo Pin
const int potpin = A0;   // pot pin

float Kp = 2.06;                                                   //Initial Proportional Gain 2.06
float Ki = 0.1;                                                      //Initial Integral Gain 0.1
float Kd = 0.75;                                                    //Intitial Derivative Gain 0.75
double Setpoint, Input, Output, ServoOutput;                                       

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.                                                                   
Servo myServo;                                                       //Initialize Servo.

void setup() {

  Serial.begin(9600);                                                //Begin Serial 
  myServo.attach(servoPin);                                          //Attach Servo
  resetServo();                                               // Reset servo to initial angle

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT); 
  pinMode(potpin, INPUT);                                    
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 

  myPID.SetOutputLimits(-80,60);                             // Set output limits to match servo range
}

void loop(){
  // Read setpoint from potentiometer
    Setpoint = map(analogRead(potpin), 0, 1023, 5, 38); // Map to beam range (5-38 cm)
    
    // Get ball position
    //input = getDistance();
  delay(100);
  //Setpoint = 10;
  Input = readPosition();                                            
 
  myPID.Compute();                                                   //computes Output in range of -80 to 60 degrees
  
  ServoOutput=50-Output;                                            // 130 degrees is my horizontal 
  myServo.write(ServoOutput);                                        //Writes value of Output to servo
  Serial.print("angle: ");
  Serial.println(ServoOutput);
  
}

void resetServo() {
  myServo.write(50);  // Set servo to 50 degrees for horizontal positioning
  delay(500);          // Delay to allow the servo to reach the desired position
}

float readPosition() {
  delay(40);                                                            //Don't set too low or echos will run into eachother.      
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2; // Calculate distance in cm

  if (distance > 38 || distance <= 2) {
    distance = 38; // Set maximum distance as 38 cm for the cart
  }
  Serial.print("Setpoint:");
  Serial.print(Setpoint);
  Serial.print("distance: ");
  Serial.println(distance);
  return distance;
}
