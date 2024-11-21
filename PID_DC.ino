#include <Encoder.h> //Encoder library
#include <PID_v1.h> //PID library

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

//Assign DC Motor Max and Min Outputs
#define MAX_MOTOR 50

//ESTOP State Tracking
bool ESTOP_STATE = false;

//create and set input, setpoint, output, and PID parameters
double PendAngle = 0.0;
double PendSet = 0.0;
double PosOut = 0.0;
double Kp = 5, Ki = 0.0, Kd = 2.0;

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

int TOGGLE_STATUS = digitalRead(TOGGLE);
int ESTOP_STATUS = digitalRead(ESTOP);

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

// if((distance <= 40 || distance >= 10) && (PendAngle <= -45 || PendAngle >= 45)){
//Turn on LEDs
digitalWrite(LED_POWER, HIGH);
digitalWrite(LED_PID, HIGH);

//Read Encoder value and convert to degrees
long count = Enc.read();
PendAngle = count * 0.15;

// Serial.println(PendAngle);

//PID Computation
sysPID.Compute();

//Serial.println(PosOut);
double PWM_Out = map(abs(PosOut), 0, 5, 0, 150);
//Move motor to the right 
if (PosOut > 0) {
    digitalWrite(D1, LOW);
    digitalWrite(D2, HIGH);
    analogWrite(PWM, PWM_Out);
  } 

//Move motor to the left
else if (PosOut < 0) {
    digitalWrite(D1, HIGH);
    digitalWrite(D2, LOW);
    analogWrite(PWM, PWM_Out);
  } 

//Stop motor
else {
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    analogWrite(PWM, 0);
  }
delay(100);
} else if (TOGGLE_STATUS == HIGH){
  digitalWrite(LED_POWER, LOW);
  digitalWrite(LED_PID, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  analogWrite(PWM, 0);
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
