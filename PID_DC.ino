#include <Encoder.h> //Encoder library
#include <PID_v1.h> //PID library

//Assign Encoder pins
#define A 2
#define B 3

//Assign DC Motor pins
#define D1 8
#define D2 9
#define PWM 10

//Assign DC Motor Max and Min Outputs
#define MAX 10
#define MIN -10

//create and set input, setpoint, output, and PID parameters
double PendAngle;
double PendSet = 0;
double PosOut;
double Kp = 0, Ki = 0, Kd = 0;

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

//Initialize PID and set output limits
sysPID.SetMode(AUTOMATIC);
sysPID.SetOutputLimits(MIN,MAX);

}

void loop() {
//Read Encoder value and convert to degrees
long count = Enc.read();
PendAngle = count * 0.15;

//PID Computation
sysPID.Compute();

//Move motor to the right 
if (PosOut > 0) {
    digitalWrite(D1, HIGH);
    digitalWrite(D2, LOW);
    analogWrite(PWM, abs(PosOut));
  } 

//Move motor to the left
else if (PosOut < 0) {
    digitalWrite(D1, LOW);
    digitalWrite(D2, HIGH);
    analogWrite(PWM, abs(PosOut));
  } 

//Stop motor
else {
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    analogWrite(PWM, 0);
  }
delay(50);

}
