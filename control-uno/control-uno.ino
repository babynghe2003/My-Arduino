#include <Servo.h>
#include <SoftwareSerial.h>

#define MOTOR_L_1 7
#define MOTOR_L_2 6
#define MOTOR_R_1 4
#define MOTOR_R_2 5

#define arm1_servo A0
#define arm2_servo A1
#define grip_servo A2
#define flag_servo 3


SoftwareSerial mySerial(A4,A5); // RX, TX

Servo arm1;
Servo arm2;
Servo grip;
Servo flag;

float arm1_angle = 90;
float arm2_angle = 90;
float grip_angle = 90;


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

void set_motor(int speedA, int speedB) {
  if (speedA > 0){
    speedA = constrain(speedA, minspeeda, maxspeeda);
    digitalWrite(MOTOR_R_2, LOW);
    analogWrite(MOTOR_R_1, speedA);
  } else{
    speedA = constrain(speedA, -maxspeeda, -minspeeda);
    digitalWrite(MOTOR_R_2, HIGH);
    analogWrite(MOTOR_R_1,255 + speedA);
  }

  if (speedB > 0){
    speedB = constrain(speedB, minspeedb, maxspeedb);
    digitalWrite(MOTOR_L_1, LOW);
    analogWrite(MOTOR_L_2, speedB);
  } else{
    speedB = constrain(speedB, -maxspeedb, -minspeedb);
    digitalWrite(MOTOR_L_1, HIGH);
    analogWrite(MOTOR_L_2, 255 + speedB);
  }
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
