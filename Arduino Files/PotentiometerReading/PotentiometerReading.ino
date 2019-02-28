//YU CHANG OU
#include <PID_v1.h>

//IO ports
int potPin = A0;
int enableM1 = 10; //Motor Enable pin Runs on PWM signal
int Encoder1 = 2;
int Encoder2 = 3;
int motor1A = 9;
int motor1B = 8;

//Other Vars
int PPR = 341;  // Encoder Pulse per revolution.

int angle; //current angle of movement

//Encoderinfo  int count = 0;
volatile int encoderPos = 0;
int lastPos = LOW;
int n = LOW;


//PID Init
double kp = 4 , ki = 1 , kd = .1;      //initialize by setting to 0        
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  

void setup() {
  // put your setup code here, to run once:
  pinMode(potPin, INPUT);
  pinMode(Encoder1, INPUT_PULLUP);
  pinMode(Encoder2, INPUT_PULLUP);
  
  attachInterrupt(0, ISRoutine, CHANGE); //pin2 interrupt on encoder
  attachInterrupt(1, ISRoutine, CHANGE); //pin3 interrupt

  TCCR1B = TCCR1B & 0b11111000 | 0x01;  // set 31KHz PWM to prevent motor noise
  //TCCR2B = TCCR2B & 0b11111000 | 0x01; // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-200, 200); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  
  pinMode(enableM1, OUTPUT);
  pinMode(motor1A, OUTPUT); 
  pinMode(motor1B, OUTPUT); 
  Serial.begin(9600);
}


//MOTOR METHOD
void forward(){
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
}
void backward(){
    digitalWrite(motor1A, HIGH);
    digitalWrite(motor1B, LOW);
}
void arrived(){
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, LOW);
}

void encoderinfo(){
      n = digitalRead(Encoder1);
  
  if ((lastPos == LOW) && (n == HIGH)) {
    if (digitalRead(Encoder2) == LOW) {
      encoderPos++;
    } else {
      encoderPos--;
    }
  }
  lastPos = n;
  //Serial.print("Encoder: ");
  //Serial.println(encoderPos);
}


//ISRROUTNINE
void ISRoutine(){
  encoderinfo();
}


void directionDecision(int out){
    
  //Serial.println(angle);  
  
  if(encoderPos < out){
    forward();
    analogWrite(enableM1, out);
  }
  else{
    backward();
    analogWrite(enableM1, abs(out));
  }
}


void loop() {

  angle = map (analogRead(potPin), 0, 1023, 0, 250);
  
  //Serial.print("Angle: ");
  Serial.println(angle);
  Serial.println(analogRead(potPin));
  
  setpoint = angle;                    //PID while work to achive this value consider as SET value
  input = encoderPos ;           // data from encoder consider as a Process value
  myPID.Compute();                 // calculate new output
  directionDecision(output);  

}
