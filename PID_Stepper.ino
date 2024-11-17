#include <Encoder.h> //Encoder library
#include <Stepper.h> //Stepper library
#include <PID_v1.h> //PID library

//assign Encoder pins
#define A 2
#define B 3

//assign Stepper pins
#define p1 8
#define p2 9
#define p3 10
#define p4 11
#define STEPS 200

//Assign Stepper Max and Min Outputs
#define MAX 10
#define MIN -10

//create and set input, setpoint, output, and PID parameters
double PendAngle;
double PendSet = 0;
double PosOut;
double Kp = 0, Ki = 0, Kd = 0;

//Encoder, Stepper, and PID Control
Encoder Enc(A,B);
Stepper Step(200, p1, p2, p3, p4);
PID sysPID(&PendAngle, &PosOut, &PendSet, Kp, Ki, Kd, DIRECT);

void setup() {
//start serial interface
Serial.begin(9600);

//Set Encoder Position and Stepper speed
Enc.write(0);
Step.setSpeed(100);

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

//Convert PID output to steps for motor
int steps = (int)PosOut;

//Check if required steps is not 0
if(steps != 0){
  Step.step(steps);
}

delay(50);

}
