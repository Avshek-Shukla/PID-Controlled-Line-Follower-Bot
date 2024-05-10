/* -- sensor pin define from left to right -- */
const int S[] = { 3, 4, 5, 6, 7};
char val;
/*   
 * A is left and B is right 
 * error is negative in right side and vise versa
*/
/* -- motor pin define -- */
// left motor definition
#define INA 8
#define INB 9

// left side enable
#define ENA 10

// right side enable
#define ENB 11

// right motor definition
#define INC 12
#define IND 13

// function prototype
int errorRead(); // read errors
int PID(int error); //controls speed by receiving error
void forward(float pidValue); //sets speed of motors

// variable declarations
float Kp = 52, Kd = 6, Ki = 0;  // hit and trial is done here
int P = 0, I = 0, D = 0, prevError, baseSpeed = 115; // PID section for globalization

void setup() {
  Serial.begin(9600);
  // sensor input setup
  for (int i=0; i<=4; i++) {
    pinMode( S[i], INPUT);
  }
  // motor output setup
  // Left motor
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

  //right motor
  pinMode(INC, OUTPUT);
  pinMode(IND, OUTPUT); 
}



void loop() {
  int error =  errorRead();
   if (Serial.available() > 0)
  {
  val = Serial.read();
  Serial.println(val);
  if (val == 'F'){
    Kp = Kp + 10;
  }
   else if (val == 'B'){
    Kp = Kp - 10;
  }
  else if (val == 'R'){
    Kd = Kd + 1;
  }
  else if (val == 'L'){
    Kd = Kd - 1;
  }
  Serial.println(Kp);
  Serial.println(Kd);
  
}  
  if (error != 10 && error != 9){
    int pidValue = PID(error);
    forward(pidValue);
  }
}

/* -- error reading mechanism -- */
int errorRead() {
  int rS[5], error,  state;
  for (int i=0; i<5; i++) {
    rS[i] = digitalRead(S[i]);   
  }

  state = rS[0]<<4 | rS[1]<<3 | rS[2]<<2 | rS[3]<<1 | rS[4]<<0;

  int errorBox[] = {10, 4, 2, 3, 0, 9, 1, 5, -2, 9, 9, 9, -1, 9, 9, 9, -4, 9, 9, 9, 9, 9, 9, 9, -3, 9, 9, 9, -5, 9, 9, 9};
  return errorBox[state];
}

/* -- PID control mechanism -- */
int PID(int error) {
  P = error;
  I = I + error;
  D = error - prevError;
  prevError = error;
  return Kp*P + Ki*I + Kd * D;  // returns PID_VALUE
}


/* -- this is for robot movement -- */
void forward(int pidValue) {
  static int flag = 0;
  if (pidValue != 0) {
  int A = constrain(baseSpeed + pidValue, 0, 255);
  int B = constrain(baseSpeed - pidValue, 0, 255);
  //Serial printing
  //Serial.print("(");Serial.print(A);Serial.print(",");Serial.print(B);Serial.println(")");
    if (flag == 0 && millis() < 7000) {
      analogWrite(ENA, 255); //left motor 
      analogWrite(ENB, 255); //right motor
      if (pidValue > 0) {
        //right turn
        digitalWrite(INA, LOW);
        digitalWrite(INB, HIGH);
        digitalWrite(INC, HIGH);
        digitalWrite(IND, LOW);
      } else {
        //left turn
        digitalWrite(INA, HIGH);
        digitalWrite(INB, LOW);
        digitalWrite(INC, LOW);
        digitalWrite(IND, HIGH);
      }
      delay(60);
      //changed to original one
      digitalWrite(INA, LOW);
      digitalWrite(INB, HIGH);
      digitalWrite(INC, LOW);
      digitalWrite(IND, HIGH);
      flag = 1;
    }
    analogWrite(ENA, A); //left motor 
    analogWrite(ENB, B); //right motor
  } else {
    //Serial.print("(");Serial.print(200);Serial.print(",");Serial.print(200);Serial.println(")");
    flag = 0;
    analogWrite(ENA, 238); //left motor 
    analogWrite(ENB, 238); //right motor
  }
  // for left motor
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);

  // for right motor
  digitalWrite(INC, LOW);
  digitalWrite(IND, HIGH);
}
