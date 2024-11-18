#include <Encoder.h> //Encoder library

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

//System Dynamics Matrix, Control Matrix, and Position Initialization
double x[4] = {0.0, 0.0, 0.0, 0.0};
double K[4] = {5.0, 5.0, 5.0, 5.0};
double PosOut = 0.0;

//Initialize Variables For Loop
unsigned long lastTime = 0.0;
double lastPos = 0.0;
double lastAngle = 0.0;

//Encoder
Encoder Enc(A,B);

void setup() {
//Start serial interface
Serial.begin(9600);

//Set Encoder Position and initialize DC Motor 
Enc.write(0);
pinMode(D1, OUTPUT);
pinMode(D2, OUTPUT);
pinMode(PWM, OUTPUT);
}

void loop() {
//Determine current time and time difference since last time recorded
unsigned long Time = millis();
double dt = (Time - lastTime)/1000;
lastTime = Time;

//Read Encoder and calculate state values based on Encoder count
long count = Enc.read();
x[0] = count * 0.15;
x[1] = (x[0]-lastAngle)/dt;
x[2] = count * (//gear ratio * steps);
x[3] = (x[2]-lastPos)/dt;

//Update temporary variables
lastAngle = x[0];
lastPos = x[2];

//Output Position Computation
for (int i = 0; i < 4; i++) {
    PosOut += x[i] * -K[i];
  }
PosOut= constrain(u, -255, 255);

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

}