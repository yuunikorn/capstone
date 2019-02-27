#include <PID_v1.h>

//Motor Init
#define MotorEnable 10 //Motor Enamble pin Runs on PWM signal
#define Out1  8  // Motor Forward pin
#define Out2  9 // Motor Reverse pin

//Motor Encoder Init
int encoderPin1 = 2; //Encoder Output 'A' must connected with interrupt pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with interrupt pin of arduino.

//Positional Pot Init
int potPosition1 = A3;           //this variable will store the position of the potentiometer

//PID Var Init
double Kp = 0.32;// you can set these constants however you like depending on trial & error
double Ki = 0.1;
double Kd = 0.3;
double input = 0;
double output = 0; 
double setpoint = 0;


void setup() {
  //Motor setup
  pinMode(MotorEnable, OUTPUT);
  pinMode(Out1, OUTPUT); 
  pinMode(Out2, OUTPUT); 
  Serial.begin(9600); //initialize serial comunication

  //Interrupt setup
  noInterrupts();  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 34286;            // preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

//  attachInterrupt(digitalPinToInterrupt(encoderPin1), ISR, mode); 
}


//Motor Methods

void motorForward(){
    digitalWrite(Out1, LOW);// Forward motion
    digitalWrite(Out2, HIGH);
}

void motorRev(){
    digitalWrite(Out1, HIGH);// Reverse motion
    digitalWrite(Out2, LOW);
}

void faststop(){
    digitalWrite(Out1, LOW);// Reverse motion
    digitalWrite(Out2, LOW);
}



//ISR (interrupt) Methods: Timer1 overflow interrupt example 
ISR(TIMER1_OVF_vect){        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
  TCNT1 = 34286;            // preload timer
  Serial.print("Hi?!");
  
  //digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
}



void loop() {
  faststop();
  delay(400);
  
  motorForward();
  delay(400);
  
  faststop();
  delay(400);
  
  motorRev();
  delay(400);

}


/////////////////////////////////////////////
