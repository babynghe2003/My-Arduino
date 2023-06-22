#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(A0, A1); // RX, TX
#define MOTOR_L_1 7
#define MOTOR_L_2 6
#define MOTOR_R_1 4
#define MOTOR_R_2 5

Servo arm1;
Servo arm2;
Servo grip;
#define arm1_servo 9
#define arm2_servo 11
#define grip_servo 10


float arm1_angle = 90;
float arm2_angle = 90;
float grip_angle = 90;

float roll_speed = 0.01;


bool isArm1In = false;
bool isArm1De = false;

bool isArm2In = false;
bool isArm2De = false;

bool isArmControl = false;

void setup() {
  arm1.attach(arm1_servo);
  arm2.attach(arm2_servo);
  grip.attach(grip_servo);

  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  Serial.begin(115200);
  mySerial.begin(9600);

}
// R-HIGH
int maxspeed = 100;
char code;

void loop() {

  if (mySerial.available()) {
    code = mySerial.read(); // read from serial or bluetooth
    Serial.println((char)code);
  }
  if (code == 'V') {
    isArmControl = true;
  } else if (code == 'v') {
    isArmControl = false;
  }
  if (isArmControl) {
    if (code == 'F') {
      stop_arm();
      isArm1De = true;  // di thang
    } else if (code == 'S' || code == 'w' || code == 'u') {
      stop_arm();
    } else if (code == 'B') {
      stop_arm();
      isArm1In = true;
    } else if (code == 'G') {
      stop_arm();
      isArm1In = true;
      isArm2In = true;
    } else if (code == 'I') {
      stop_arm();
      isArm1In = true;
      isArm2De = true;
    } else if (code == 'L') {
      stop_arm();
      isArm2In = true;
    } else if (code == 'R') {
      stop_arm();
      isArm2De = true;
    } else if (code == 'H') {
      stop_arm();
      isArm1De = true;
      isArm2In = true;
    } else if (code == 'J') {
      stop_arm();
      isArm1De = true;
      isArm2De = true;
    } else if (code == 'X') {
      grip.write(170);
    } else if (code == 'x') {
      grip.write(80);
    } else if (code == 'W') {
      stop();
    } else if (code == 'U') {
      stop();
    } else if (code <= '9' && code >= '0'){
      roll_speed = 0.01 + 0.0015*(code - 48);
    } else if (code == 'q'){
      roll_speed = 0.025;
    }
    servo_control();

  } else {
    if (code == 'F') {  // di thang
      forward();
    } else if (code == 'S') {
      stop();
    } else if (code == 'B') {
      backward();
    } else if (code == 'G') {
      forwardLeft();
    } else if (code == 'I') {
      forwardRight();
    } else if (code == 'L') {
      left();
    } else if (code == 'R') {
      right();
    } else if (code == 'H') {
      backwardLeft();
    } else if (code == 'J') {
      backwardRight();
    } else if (code <= '9' && code >= '0'){
      maxspeed = 100 + 12*(code - 48);
    } else if (code == 'q'){
      maxspeed = 250;
    }
  }
}

// ###################################################### CONTROL MOTOR CAR ==============================================
void stop() {
  motor_l_tien(0);
  motor_r_tien(0);
}

void forward() {
  motor_l_tien(maxspeed);
  motor_r_tien(maxspeed);
}

void forwardLeft() {
  motor_l_tien(0);
  motor_r_tien(maxspeed);
}

void forwardRight() {
  motor_l_tien(maxspeed);
  motor_r_tien(0);
}

void backward() {
  motor_r_lui(maxspeed*3/4);
  motor_l_lui(maxspeed*3/4);
}

void backwardLeft() {
  motor_l_lui(maxspeed);
  motor_r_tien(0);
}

void backwardRight() {
  motor_l_tien(0);
  motor_r_lui(maxspeed);
}

void left() {
  motor_l_lui(maxspeed*3/4);
  motor_r_tien(maxspeed*3/4);
}
void right() {
  motor_r_lui(maxspeed*3/4);
  motor_l_tien(maxspeed*3/4);
}

void motor_l_tien(int speed){
  digitalWrite(MOTOR_L_1, LOW);
  analogWrite(MOTOR_L_2, constrain(speed, 0, 255));  
}
void motor_l_lui(int speed){
  digitalWrite(MOTOR_L_1, HIGH);
  analogWrite(MOTOR_L_2, constrain(255 - speed, 0, 255));
  Serial.print("HIGH");
}
void motor_r_tien(int speed){
  digitalWrite(MOTOR_R_1, LOW);
  analogWrite(MOTOR_R_2, constrain(speed, 0, 255));  
}
void motor_r_lui(int speed){
  digitalWrite(MOTOR_R_1, HIGH);
  analogWrite(MOTOR_R_2, constrain(255-speed, 0, 255));
  Serial.print("HIGH");
}

// <=============================================CONTROL ARM======================================================================

void stop_arm() {
  isArm1In = false;
  isArm1De = false;
  isArm2In = false;
  isArm2De = false;
}

void servo_control() {
  if (isArm1In && arm1_angle + roll_speed < 180) {
    arm1_angle += roll_speed;
  }
  if (isArm1De && arm1_angle - roll_speed > 0) {
    arm1_angle -= roll_speed;
  }

  if (isArm2In && arm2_angle + roll_speed < 180) {
    arm2_angle += roll_speed;
  }
  if (isArm2De && arm1_angle - roll_speed > 0) {
    arm2_angle -= roll_speed;
  }

  arm1.write(arm1_angle);
  arm2.write(arm2_angle);
}
