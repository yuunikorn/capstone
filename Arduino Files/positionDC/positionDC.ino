//CANON CAPSTONE CODE || YU CHANG OU
#include <PID_v1.h>
  
//Motor Init
int MotorEnable = 10;//Motor Enamble pin Runs on PWM signal
int Out1 = 8;  // Motor Forward pin
int Out2 = 9; // Motor Reverse pin

//Motor Encoder Init
int encoderPin1 = 2; //Encoder Output 'A' must connected with interrupt pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with interrupt pin of arduino.

int potPin = A0;

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

  
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT); 
  pinMode(potPin, INPUT);
  
  Serial.begin(9600); //initialize serial comunication

  //Interrupt setup
  noInterrupts();  
  
  //TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise

  //TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TCCR2B = TCCR2B & 0b11111000;
  
  TCNT1 = 34286;            // preload timer 65536-16MHz/256/2Hz
  
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

  
}






//ISR (interrupt) Methods: Timer1 overflow interrupt example 
ISR(TIMER1_OVF_vect){      // interrupt service routine that wraps a user defined function supplied by attachInterrupt
  TCNT1 = 44286;   //34286;            // preload timer


  int position = analogRead(potPin);
  if ((position > 400) && (position < 600)){
    faststop();
  }
  else if(position <= 400){
    motorForward();
  }
  else{
    motorRev();
  }
  
}


ISR(TIMER2_OVF_vect){
  //Serial.println("TWO");
}



void loop() {
  analogWrite(MotorEnable, 223);
  
  
 
}


/////////////////////////////////////////////

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
