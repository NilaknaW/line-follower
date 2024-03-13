//Right motor driver pins
int in1 = 8;
int in2 = 7;
int enA = 9;

// Left motor driver pins 
int in3 = 5;
int in4 = 4;
int enB = 3;

//IR sensor array pins
#define ir0 A0  //D2
#define ir1 A1  //D3
#define ir2 A2  //D4
#define ir3 A3  //D5
#define ir4 A4  //D6
#define ir5 A5  //D7

//ultrasonic sensor pins
int ustrig = 11;
int usecho = 12;

//speed variables
int rightSpeedpwm = 0;
int leftSpeedpwm = 0;
int speedAdjust = 0;
int baseSpeed = 60;
int maxSpeed = 100;
int irThresh = 900;  // add code to calibrate the thresh
int irThreshMargin = 50; // put a good threshold

// IR readings
int irVal[6] = {0, 0, 0, 0, 0, 0};
int irWeight[6] = {-3, -2, -1, 1, 2, 3};
int irValAnalog[6] = {0, 0, 0, 0, 0, 0};

// PID control variables
float error = 0;
float prevError = 0;
float P, I, D;

float kp = 5;   //change these values
float ki = 0.001;   //change these values
float kd = 2;   //change these values

// function declaration
void lineFollow();
void setSpeed();
void forward();
void reverse();
void stop();
void readIR();
void PIDcontrol();
void printIRvals();
void calibrateSensors();
void checkMotorDriver();

//main program starts
void setup() {
  // put your setup code here, to run once:
  pinMode(ir0, INPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  pinMode(usecho, INPUT);
  pinMode(ustrig, OUTPUT);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);  

  Serial.begin(9600);

  calibrateSensors();
  // Serial.println(irThresh);
  // delay(2000);

  forward();
  delay(1000);
}

// main program
void loop() {
  lineFollow();
}


////////////////////////////////////////////////////////////////////////
// function definition

void lineFollow() {
  readIR();
  // printIRvals();
  PIDcontrol();
  setSpeed();
  // checkMotorDriver(); //check motor driver code
}

//set moving speed
void setSpeed() {
  analogWrite(enA, rightSpeedpwm);
  analogWrite(enB, leftSpeedpwm); 
}

//set moving direction
void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void reverse() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

// read input from IR
void readIR() {
  // direct digital read
  // irVal[0] = digitalRead(ir0);
  // irVal[1] = digitalRead(ir1);
  // irVal[2] = digitalRead(ir2);
  // irVal[3] = digitalRead(ir3);
  // irVal[4] = digitalRead(ir4);
  // irVal[5] = digitalRead(ir5);

  // analog read and thresholding
  irValAnalog[0] = analogRead(ir0);
  irValAnalog[1] = analogRead(ir1);
  irValAnalog[2] = analogRead(ir2);
  irValAnalog[3] = analogRead(ir3);
  irValAnalog[4] = analogRead(ir4);
  irValAnalog[5] = analogRead(ir5);

  for (int i=0; i<6; i++) {
    if (irValAnalog[i] > irThresh) {
      irVal[i]=1;
    } else {
      irVal[i]=0;
    }
  }
}

// PID controller code
void PIDcontrol() {
  // if meets white line stop immediately
  if (irVal[0]+irVal[1]+irVal[2]+irVal[3]+irVal[4]+irVal[5]==0) {
    stop();
  }

  // calculate error
  error = 0;
  for (int i=0; i<6; i++) {
    error += irWeight[i]*irVal[i];
  }

  P = error;
  I = I + error;
  D = error - prevError;
  prevError = error;

  // adjust speed values of left and right wheels
  speedAdjust = kp*P + ki*I + kd*D;
  rightSpeedpwm = baseSpeed + speedAdjust;      // plus minus depend on IR input: if white is 0 then this is correct
  leftSpeedpwm = baseSpeed - speedAdjust;

  if (rightSpeedpwm < 0) {
    rightSpeedpwm = 0;
  }
  if (leftSpeedpwm < 0) {
    leftSpeedpwm = 0;
  }
  if (rightSpeedpwm > maxSpeed) {
    rightSpeedpwm = maxSpeed;
  }
  if (leftSpeedpwm > maxSpeed) {
    leftSpeedpwm = maxSpeed;
  }
}

void calibrateSensors() {
  readIR();
  int irsum = 0;
  for (int i = 0; i < 6; i++) {
    // Read sensor values and calculate average
    irsum += irValAnalog[i];
  }
  irThresh = irsum / 6 + irThreshMargin;
}

void printIRvals() {
  // print analog readings
  for(int i = 0; i < 6; i++) {
    Serial.print(irValAnalog[i]);
    Serial.print("-");
  }
  Serial.println("");

  // print thresholded ir values
  for(int i = 0; i < 6; i++) {
    Serial.print(irVal[i]);
    Serial.print("-");
  }
  Serial.println("");

  // keep one line space for convenience
  Serial.println("");
  delay(2000);
}

void checkMotorDriver() {
  rightSpeedpwm = 200;
  leftSpeedpwm = 100;
  setSpeed();
  forward();
  delay(1000);
  reverse();
  delay(1000);
  stop();
  delay(2000);
}

