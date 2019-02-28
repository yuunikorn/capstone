int potPin = A0;

int enableM1 = 10; //Motor Enamble pin Runs on PWM signal

int Encoder1 = 2;
int Encoder2 = 3;
int motor1A = 9;
int motor1B = 8;

void setup() {
  // put your setup code here, to run once:
  pinMode(potPin, INPUT);
  pinMode(Encoder1, INPUT);
  pinMode(Encoder2, INPUT);
  
  pinMode(enableM1, OUTPUT);
  pinMode(motor1A, OUTPUT); 
  pinMode(motor1B, OUTPUT); 
  Serial.begin(115200);
}

void loop() {
  int position = analogRead(potPin);
  int motorspeed1 = 0;
  
  if (position < 512){
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, HIGH);
    motorspeed1 = map(analogRead(potPin), 0, 512, 255, 5);
    analogWrite(enableM1, motorspeed1);
  }
  else{
    digitalWrite(motor1A, HIGH);
    digitalWrite(motor1B, LOW);
    motorspeed1 = map(analogRead(potPin), 513, 1023, 5, 255);
    analogWrite(enableM1, motorspeed1);
  }


}
