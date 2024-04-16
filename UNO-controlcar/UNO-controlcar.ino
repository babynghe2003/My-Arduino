#include <Servo.h>
// #include <SoftwareSerial.h>

#define MOTOR_L_1 7
#define MOTOR_L_2 6
#define MOTOR_R_1 5
#define MOTOR_R_2 4

#define arm1_servo A2
#define arm2_servo A3
#define grip_servo A1
#define center_servo A4
#define flag_servo A0


// SoftwareSerial mySerial(2,3); // RX, TX

Servo arm1;
Servo arm2;
Servo grip;
Servo center;
Servo flag;

float arm1_angle = 90;
float arm2_angle = 90;

bool is_grip = false;
float grip_angle = 95;
float ungrip_angle = 160;

float center_angle = 90;
float center_d = 3;

bool is_flag = false;
float flag_angle = 180;
float unflag_angle = 90;


bool isArm1In = false;
bool isArm1De = false;

bool isArm2In = false;
bool isArm2De = false;

bool isArmControl = false;

void setup() {
  arm1.attach(arm1_servo);
  arm2.attach(arm2_servo);
  grip.attach(grip_servo);
  center.attach(center_servo);
  flag.attach(flag_servo);


  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  Serial.begin(9600);
  // mySerial.begin(9600);

  arm1.write(90);
  arm2.write(90);
  grip.write(ungrip_angle);
  center.write(center_angle);

}
// R-HIGH
int maxspeed = 255;
char code;
String message = "";

void loop() {

  if (Serial.available()) {
    message = Serial.readStringUntil('\n'); // read from serial or bluetooth
    // Serial.println(message);
  if (message == "F") {  // di thang
    Serial.println("Forward");
    forward();
  } else if (message == "S") {
    Serial.println("Stop");
    stop();
  } else if (message == "G") {
    backward();
  } else if (message == "Q") {
    forwardLeft();
  } else if (message == "E") {
    forwardRight();
  } else if (message == "L") {
    left();
  } else if (message == "R") {
    right();
  } else if (message == "Z") {
    backwardLeft();
  } else if (message == "C") {
    backwardRight();
  } else if (message == "M") {
    if (is_grip){
      grip.write(ungrip_angle);
    }else{
      grip.write(grip_angle);
    }
    is_grip = !is_grip;
  } else if (message == "N") {
    if (is_flag){
      flag.write(unflag_angle);
    }else{
      flag.write(flag_angle);
    }
    is_flag = !is_flag;
  } else if (message == "X") {
    center_angle = constrain(center_angle + center_d, 0, 180);
    center.write(center_angle);
  } else if (message == "Y") {
    center_angle = constrain(center_angle - center_d, 0, 180);
    center.write(center_angle);
  } else if (message[0] == 'J'){
    arm1_angle = message.substring(1).toInt();
    arm1.write(arm1_angle);
  } else if (message[0] == 'K'){
    arm2_angle = message.substring(1).toInt();
    arm2.write(arm2_angle);
  } 
  }
  

}

// ###################################################### CONTROL MOTOR CAR ==============================================
void stop() {
  setmotor(0, 0);
}

void forward() {
  setmotor(maxspeed, maxspeed);
}

void forwardLeft() {
  setmotor(0, maxspeed);
}

void forwardRight() {
  setmotor(maxspeed, 0);
}

void backward() {
  setmotor(-maxspeed, -maxspeed);
}

void backwardLeft() {
  setmotor(0, -maxspeed);
}

void backwardRight() {
  setmotor(-maxspeed, 0);
}

void left() {
  setmotor(-maxspeed, maxspeed);
}
void right() {
  setmotor(maxspeed, -maxspeed);
}

void setmotor(int speedA, int speedB) {
  if (speedA > 0){
    speedA = constrain(speedA, 0, maxspeed);
    digitalWrite(MOTOR_R_2, LOW);
    analogWrite(MOTOR_R_1, speedA);
  } else{
    speedA = constrain(speedA, -maxspeed, 0);
    digitalWrite(MOTOR_R_2, HIGH);
    analogWrite(MOTOR_R_1,255 + speedA);
  }
  speedB = -speedB;
  if (speedB > 0){
    speedB = constrain(speedB, 0, maxspeed);
    digitalWrite(MOTOR_L_1, LOW);
    analogWrite(MOTOR_L_2, speedB);
  } else{
    speedB = constrain(speedB, -maxspeed, 0);
    digitalWrite(MOTOR_L_1, HIGH);
    analogWrite(MOTOR_L_2, 255 + speedB);
  }
}


