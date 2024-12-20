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
#define MAX_MOTOR 250
#define PIDLIM 50
//Toggle State Tracking
int TOGGLE_STATUS = 1;
int ESTOP_STATUS = 1;

bool isSystemStopped = false;
bool toggleWasOff = false;

//create and set input, setpoint, output, and PID parameters
double PendAngle = 0.0;
double PendSet = 0.0;
double PosOut = 0.0;
double Kp = 9.0, Ki = 0.0, Kd = 1;

//Encoder and PID Control
Encoder Enc(A,B);
PID sysPID(&PendAngle, &PosOut, &PendSet, Kp, Ki, Kd, DIRECT);

void setup() {

Serial.begin(9600);

Enc.write(-1200); //Current Angle is reference

//Motor Pins
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
sysPID.SetOutputLimits(-PIDLIM,PIDLIM);


}



const double DEAD_ZONE = 2.0;

void loop() {

TOGGLE_STATUS = digitalRead(TOGGLE);
ESTOP_STATUS = digitalRead(ESTOP);

// Check if ESTOP is pressed
  if (ESTOP_STATUS == LOW) {

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

//-----Run Main Code Here----//
bool ENC_REF = false;
  if(TOGGLE_STATUS == LOW){

    digitalWrite(LED_POWER, HIGH);
    digitalWrite(LED_PID, HIGH);

    //Read Encoder value and convert to degrees
    long count = Enc.read();
    PendAngle = count * 0.15;

    Serial.println(PendAngle);
    //Serial.println(PosOut);

        //PID Computation
    sysPID.Compute();

    if (abs(PendAngle - PendSet) <= DEAD_ZONE && abs(PosOut) < 5.0) {
      stop_motor();
      return;
    }

    //Serial.println(PosOut);
    double PWM_Out = constrain(map(abs(PosOut), 0, PIDLIM, 0, MAX_MOTOR), 0, MAX_MOTOR);
    //Move motor to the right 
    if (PosOut > 0) {
        move_left(PWM_Out);
      } 

    //Move motor to the left
    else if (PosOut < 0) {
        move_right(PWM_Out);
      } 

    // else {
    //     stop_motor();
    // }

    delay(20);
  } else if (TOGGLE_STATUS == HIGH){
        //Turn OFF All Components
        digitalWrite(LED_POWER, LOW);
        digitalWrite(LED_PID, LOW);
        digitalWrite(D1, LOW);
        digitalWrite(D2, LOW);
        analogWrite(PWM, 0);
  }


 }

void move_right(int pwm_value) {
  digitalWrite(D1, LOW);
  digitalWrite(D2, HIGH);
  analogWrite(PWM, pwm_value);
}

void move_left(int pwm_value) {
  digitalWrite(D1, HIGH);
  digitalWrite(D2, LOW);
  analogWrite(PWM, pwm_value);
}

void stop_motor() {
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  analogWrite(PWM, 0);
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
