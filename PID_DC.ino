#include <Encoder.h> //Encoder library
#include <PID_v1.h> //PID library

//Assign Encoder pins
#define A 2
#define B 3

//Assign LEDs
#define LED_POWER 4
#define LED_PID 5
#define LED_POLE 6

//Assign switches
#define TOGGLE 7
#define ESTOP 8

//Assign DC Motor pins
#define D1 9
#define D2 10
#define PWM 11

//Assign Ultrasonic Sensor
#define TRIG 12
#define ECHO 13

//Assign DC Motor Max and Min Outputs
#define MAX_MOTOR 50

//create and set input, setpoint, output, and PID parameters
double PendAngle = 0.0;
double PendSet = 0.0;
double PosOut = 0.0;
double Kp = 25, Ki = 0, Kd = 2;

//Encoder and PID Control
Encoder Enc(A,B);
PID sysPID(&PendAngle, &PosOut, &PendSet, Kp, Ki, Kd, DIRECT);

void setup() {
//Start serial interface
Serial.begin(9600);

//Set Encoder Position and initialize DC Motor 
Enc.write(0);
pinMode(D1, OUTPUT);
pinMode(D2, OUTPUT);
pinMode(PWM, OUTPUT);

//Initialzie LEDs
pinMode(LED_POWER, OUTPUT);
pinMode(LED_PID, OUTPUT);
pinMode(LED_POLE, OUTPUT);

//Initialzie Switches
pinMode(TOGGLE, INPUT);
pinMode(ESTOP, INPUT);

//Initialzie Ultrasonic Sensor
pinMode(TRIG, OUTPUT);
pinMode(ECHO, INPUT);

//Initialize PID and set output limits
sysPID.SetMode(AUTOMATIC);
sysPID.SetOutputLimits(-5,5);

}

void loop() {
//Check ESTOP State
if(ESTOP == LOW){

  //Check TOGGLE State
  while(TOGGLE == HIGH){
//Measure cart postion using ultrasonic sensor
double distance = DistMeas();

if((distance <= 40 || distance >= 10) && (PendAngle <= -45 || PendAngle >= 45)){
//Turn on LEDs
digitalWrite(LED_POWER, HIGH);
digitalWrite(LED_PID, HIGH);

//Read Encoder value and convert to degrees
long count = Enc.read();
PendAngle = count * 0.15;

Serial.println(count);
Serial.println(PendAngle);

//PID Computation
sysPID.Compute();
double PWM_Out = map(abs(PosOut), 0, 5, 0, 50);
//Move motor to the right 
if (PosOut > 0) {
    digitalWrite(D1, HIGH);
    digitalWrite(D2, LOW);
    analogWrite(PWM, PWM_Out);
  } 

//Move motor to the left
else if (PosOut < 0) {
    digitalWrite(D1, LOW);
    digitalWrite(D2, HIGH);
    analogWrite(PWM, PWM_Out);
  } 

//Stop motor
else {
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    analogWrite(PWM, 0);
  }
delay(50);
}

else {
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  analogWrite(PWM, 0);
  return;
}
    }

}

else{
//Break from loop
return;
}
}

int DistMeas() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  int duration = pulseIn(ECHO, HIGH);
  int dist = duration * 0.034 / 2;
  return  dist;
}
