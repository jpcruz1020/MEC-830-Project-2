#include <Encoder.h> //Encoder library

//Assign Encoder pins
#define A 2
#define B 3

//Assign LEDs
#define LED_POWER 6
#define LED_PID 4
#define LED_POLE 5

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

//Initialize Toggle and EStop Status
int TOGGLE_STATUS = 1;
int ESTOP_STATUS = 1:

//System Dynamics Matrix, Control Matrix, and Position Initialization
double x1 = 0.0, x2 = 0.0, x3 = 0.0, x4 = 0.0;
double K[] = {1.0, 1.0, 1.0, 1.0};
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

//Initialzie LEDs
pinMode(LED_POWER, OUTPUT);
pinMode(LED_PID, OUTPUT);
pinMode(LED_POLE, OUTPUT);
digitalWrite(LED_POWER, LOW);
digitalWrite(LED_PID, LOW);
digitalWrite(LED_POLE, LOW);

//Initialzie Switches
pinMode(TOGGLE, INPUT_PULLUP);
pinMode(ESTOP, INPUT_PULLUP);

//Initialzie Ultrasonic Sensor
pinMode(TRIG, OUTPUT);
pinMode(ECHO, INPUT);

//Initialize PID and set output limits
sysPID.SetMode(AUTOMATIC);
sysPID.SetOutputLimits(-5,5);
}

bool isSystemStopped = false;
bool toggleWasOff = false;

void loop() {

//Check state of Toggle and EStop
TOGGLE_STATUS = digitalRead(TOGGLE);
ESTOP_STATUS = digitalRead(ESTOP);

// Check if ESTOP is pressed
  if (ESTOP_STATUS == LOW) {
    // Latch the system off
    isSystemStopped = true;

    // Turn off all system components
    digitalWrite(LED_POWER, LOW);
    digitalWrite(LED_PID, LOW);
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);

    Serial.println("System is OFF (ESTOP activated).");
  }

// If the system is latched as stopped, check for toggle reset
  if (isSystemStopped) {
    if (TOGGLE_STATUS == HIGH) {
      toggleWasOff = true;
    } else if (TOGGLE_STATUS == LOW && toggleWasOff) {
      isSystemStopped = false;
      toggleWasOff = false;
      Serial.println("System Reset via Toggle.");
    }
    return;
  }

//Check TOGGLE State
if(TOGGLE_STATUS == LOW){
//Measure cart postion using ultrasonic sensor
double distance = DistMeas();

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
