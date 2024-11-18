//Assign DC Motor pins
#define D1 2
#define D2 3
#define PWM 6

//Assign DC Motor Max and Min Outputs
#define MAX 255
#define MIN -255

void setup() {
//Start serial interface
Serial.begin(9600);
pinMode(D1, OUTPUT);
pinMode(D2, OUTPUT);
pinMode(PWM, OUTPUT);
}

void loop() {

//Move motor to the right 
digitalWrite(D1, HIGH);
digitalWrite(D2, LOW);
analogWrite(PWM, 50);
}
