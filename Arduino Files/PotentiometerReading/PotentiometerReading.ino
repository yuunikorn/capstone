//YU CHANG OU
#include <PID_v1.h>

//IO ports
int potPinA0 = A0;
int enableM1 = 10; //Motor Enable pin Runs on PWM signal
int Encoder1 = 2;
int Encoder2 = 3;
int motor1A = 9;
int motor1B = 8;

int potPinA1 = A1;
int enableM2 = 11; //Motor Enable pin Runs on PWM signal
int Encoder1M2 = 21;
int Encoder2M2 = 20;
int motor2A = 13;
int motor2B = 12;

//Other Vars
int PPR = 260;  // Encoder Pulse per revolution.
int posMax = PPR;
int posMin = 0;
int potentMax = 1023;
int potentMin = 0;

//int angle; //current angle of movement

//Encoderinfo  int count = 0;
volatile int encoderPos = 0;
int lastPos = LOW;
int n = LOW;

volatile int M2encoders = 0;
int M2lastposition = LOW;
int m = LOW;


//PID Init
double kp = 5 , ki = 0 , kd = 0;      //initialize by setting to 0 //5 , ki = 1 , kd = .3;        
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  
double error;

//PID Init
double kpM2 = 5 , kiM2 = 0 , kdM2 = 0;      //initialize by setting to 0 //5 , ki = 1 , kd = .3;        
double inputM2 = 0, outputM2 = 0, setpointM2 = 0;
PID myPIDM2(&inputM2, &outputM2, &setpointM2, kpM2, kiM2, kdM2, DIRECT);  
double errorM2;


void setup() {
  // put your setup code here, to run once:
  pinMode(potPinA0, INPUT);
  pinMode(Encoder1, INPUT_PULLUP);
  pinMode(Encoder2, INPUT_PULLUP);

  pinMode(potPinA1, INPUT);
  pinMode(Encoder1M2, INPUT_PULLUP);
  pinMode(Encoder2M2, INPUT_PULLUP);
  
  attachInterrupt(0, ISRoutine, CHANGE); //pin2 interrupt on encoder
  attachInterrupt(1, ISRoutine, CHANGE); //pin3 interrupt

  attachInterrupt(2, ISRoutineM2, CHANGE); //pin2 interrupt on encoder
  attachInterrupt(3, ISRoutineM2, CHANGE); //pin3 interrupt

  TCCR1B = TCCR1B & 0b11111000 | 0x01;  // set 31KHz PWM to prevent motor noise
  //TCCR2B = TCCR2B & 0b11111000 | 0x01; // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode`
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-250, 250); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
  myPIDM2.SetMode(AUTOMATIC);   //set PID in Auto mode`
  myPIDM2.SetSampleTime(1);  // refresh rate of PID controller
  myPIDM2.SetOutputLimits(-250, 250); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  
  pinMode(enableM1, OUTPUT);
  pinMode(motor1A, OUTPUT); 
  pinMode(motor1B, OUTPUT); 
    
  pinMode(enableM2, OUTPUT);
  pinMode(motor2A, OUTPUT); 
  pinMode(motor2B, OUTPUT); 
  Serial.begin(9600);
}


//MOTOR METHOD
void forwardM1(){
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
}
void backwardM1(){
    digitalWrite(motor1A, HIGH);
    digitalWrite(motor1B, LOW);
}
void arrivedM1(){
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, LOW);
}
void forwardM2(){
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
}
void backwardM2(){
    digitalWrite(motor2A, HIGH);
    digitalWrite(motor2B, LOW);
}
void arrivedM2(){
    digitalWrite(motor2A, LOW);
    digitalWrite(motor2B, LOW);
}


void encoderinfoM1(){
      n = digitalRead(Encoder1);
  
  if ((lastPos == LOW) && (n == HIGH)) {
    if (digitalRead(Encoder2) == LOW) {
      encoderPos++;
    } else {
      encoderPos--;
    }
  }
  lastPos = n;
  error = setpoint - input;
}

void encoderinfoM2(){
      m = digitalRead(Encoder1M2);
  
  if ((M2lastposition == LOW) && (m == HIGH)) {
    if (digitalRead(Encoder2M2) == LOW) {
      M2encoders ++;
    } else {
      M2encoders --;
    }
  }
  M2lastposition = m;
  errorM2 = setpointM2 - inputM2;
}


//ISRROUTNINE
void ISRoutine(){
  encoderinfoM1();
}

void ISRoutineM2(){
  encoderinfoM2();
}


void directionDecisionM1(int out){
 //Serial.println(encoderPos);
  if(encoderPos < out){
    forwardM1();
    forwardM2();
    analogWrite(enableM1, out);
    analogWrite(enableM2, out);
  }
  else{
    backwardM1();
    backwardM2();
    analogWrite(enableM1, abs(out));
    analogWrite(enableM2, abs(out));
  }
}

void directionDecisionM2(int out){
 //Serial.println(encoderPos);
  if(encoderPos < out){
    forwardM1();
    forwardM2();
    analogWrite(enableM1, out);
    analogWrite(enableM2, out);
  }
  else{
    backwardM1();
    backwardM2();
    analogWrite(enableM1, abs(out));
    analogWrite(enableM2, abs(out));
  }
}


void loop() {

  int angle = map (analogRead(potPinA0), potentMin, potentMax, posMin, posMax);
  int angleM2 = map (analogRead(potPinA1), potentMin, potentMax, posMin, posMax);
  setpoint = angle;
  setpointM2 = angleM2; 
  //setpoint = 45;   //angle;                    //PID while work to achive this value consider as SET value
  input = encoderPos;           // data from encoder consider as a Process value
  inputM2 = M2encoders;
  myPID.Compute();                 // calculate new output
  myPIDM2.Compute(); 
  directionDecisionM1(output);  
  directionDecisionM2(outputM2); 


  //Serial.println(encoderPos);
  
  
}
