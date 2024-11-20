#include <Encoder.h> //Encoder library

//Assign Encoder pins
#define A 2
#define B 3

//Assign DC Motor pins
#define D1 8
#define D2 9
#define PWM 10

//Assign DC Motor Max and Min Outputs
#define MAX 50

//System Dynamics Matrix, Control Matrix, and Position Initialization
double x1 = 0.0, x2 = 0.0, x3 = 0.0, x4 = 0.0;
double K[] = {1.0, 0.0, 0.0, 0.0};
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
double dt = (Time - lastTime)/1000.0;
lastTime = Time;

//Read Encoder and calculate state values based on Encoder count
long count = Enc.read();
x1 = count * 0.15;
Serial.println(x1);
x2 = (x1-lastAngle)/dt;
Serial.println(x2);
x3 = count * (10);
Serial.println(x3);
x4 = (x3-lastPos)/dt;
Serial.println(x4);


//Update temporary variables
lastAngle = x1;
lastPos = x3;

//Output Position Computation
PosOut = -1*(K[0]*x1 + K[1]*x2 +K[2]*x3 + K[3]*x4);
//Serial.println(PosOut);
PosOut = constrain(PosOut, -5, 5);
double PWM_Out = map(abs(PosOut), 0, 5, 0, 50);

//Move motor to the right 
if (PosOut > 0) {
    digitalWrite(D1, HIGH);
    digitalWrite(D2, LOW);
    analogWrite(PWM, abs(PWM_Out));
  } 

//Move motor to the left
else if (PosOut < 0) {
    digitalWrite(D1, LOW);
    digitalWrite(D2, HIGH);
    analogWrite(PWM, abs(PWM_Out));
  } 

//Stop motor
else {
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    analogWrite(PWM, 0);
  }

}