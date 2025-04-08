#include <ServoTimer2.h>
#include <AltSoftSerial.h>

#define MOTOR_L_1 7
#define MOTOR_L_2 6
#define MOTOR_R_1 5
#define MOTOR_R_2 4

#define arm1_servo A2
#define arm2_servo A1
#define grip_servo A0
#define center_servo A4

AltSoftSerial altSerial;  // 8,9 : RX,TX

ServoTimer2 arm1;
ServoTimer2 arm2;
ServoTimer2 grip;
ServoTimer2 center;

float arm1_angle = 90;
float arm2_angle = 90;

bool is_grip = false;
float grip_angle = 140;
float ungrip_angle = 70;

float center_angle = 90;
float center_d = 3;


bool isArm1In = false;
bool isArm1De = false;

bool isArm2In = false;
bool isArm2De = false;

bool isArmControl = false;

void setup() {
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  Serial.begin(9600);
  // Initialize Bluetooth communication
  altSerial.begin(9600);

  // Clear any pending data
  while (altSerial.available()) {
    altSerial.read();
  }

  arm1.attach(arm1_servo);
  arm2.attach(arm2_servo);
  grip.attach(grip_servo);
  center.attach(center_servo);

  arm1.write(angleToPulse(arm1_angle));
  arm2.write(angleToPulse(arm2_angle));
  grip.write(angleToPulse(grip_angle));
  center.write(angleToPulse(center_angle));
}
// R-HIGH
int maxspeed = 255;
char code;
String message = "";
bool has_code = false;

void loop() {
  while (altSerial.available()) {
    char c = altSerial.read();
    if ((c >= 32 && c <= 126) || c == '\n' || c == '\r') {
      if (c == '\n') {
        processCommand();
        message = "";
      } else if (c != '\r') {
        message += c;
      }
    }
  }
  delay(50);
}
void processCommand() {
  // Serial.println(message);
  if (message == "F") {  // di thang
    // Serial.println("Forward");
    forward();
  } else if (message == "S") {
    // Serial.println("Stop");
    stop();
  } else if (message == "B") {
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
    grip.write(grip_angle);
  } else if (message == "m") {
    grip.write(ungrip_angle);
  } else if (message == "X") {
    center_angle = constrain(center_angle + center_d, 0, 180);
    center.write(center_angle);
  } else if (message == "Y") {
    center_angle = constrain(center_angle - center_d, 0, 180);
    center.write(center_angle);
  } else if (message[0] == 'J') {  // "J108" -> "108" -> 108
    arm1_angle = message.substring(1).toInt();
    arm1.write(arm1_angle);
  } else if (message[0] == 'K') {
    arm2_angle = message.substring(1).toInt();
    arm2.write(arm2_angle);
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
  setmotor(-maxspeed * 0.6, maxspeed * 0.6);
}

void forwardRight() {
  setmotor(maxspeed * 0.6, -maxspeed * 0.6);
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
  setmotor(-maxspeed * 0.6, maxspeed * 0.6);
}
void right() {
  setmotor(maxspeed * 0.6, -maxspeed * 0.6);
}

void setmotor(int speedA, int speedB) {

  // speedA = -speedA;
  if (speedA > 0) {
    digitalWrite(MOTOR_R_2, LOW);
    digitalWrite(MOTOR_R_1, HIGH);
  } else if (speedA < 0) {
    digitalWrite(MOTOR_R_2, HIGH);
    digitalWrite(MOTOR_R_1, LOW);
  } else {
    digitalWrite(MOTOR_R_2, LOW);
    digitalWrite(MOTOR_R_1, LOW);
  }

  speedB = -speedB;
  if (speedB > 0) {
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, HIGH);
  } else if (speedB < 0) {
    digitalWrite(MOTOR_L_1, HIGH);
    digitalWrite(MOTOR_L_2, LOW);
  } else {
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, LOW);
  }
}
