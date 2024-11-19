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

//Initialize PID and set output limits
sysPID.SetMode(AUTOMATIC);
sysPID.SetOutputLimits(-5,5);

}

void loop() {
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
