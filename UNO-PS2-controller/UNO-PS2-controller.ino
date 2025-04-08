#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>
// #define PS2X_DEBUG

#define PS2_DAT 10  //MISO  19
#define PS2_CMD 11  //MOSI  23
#define PS2_SEL 12  //SS     5
#define PS2_CLK 13  //SLK   18

#define MOTOR_L_1 7
#define MOTOR_L_2 6
#define MOTOR_R_1 5
#define MOTOR_R_2 4

#define arm1_servo A2
#define arm2_servo A3
#define grip_servo A1
#define center_servo A4

#define pressures false
#define rumble false

PS2X ps2x;  // create PS2 Controller Class

Servo centerServo;
Servo gripServo;
Servo armServo1;
Servo armServo2;

int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;

float acen = 0.25;
float center = 75;
int grip = 120;
float arm1Servo = 180;
float arm2Servo = 65;

bool is_grip = false;
float grip_angle = 95;
float ungrip_angle = 160;

float center_angle = 90;
float center_d = 3;

bool isArm1In = false, isArm2In = false, isArm1De = false, isArm2De = false, isCenterIn = false, isCenterDe = false, isFlagIn = false, isFlagDe = false;

bool isForWard = false, isBackWard = false, isLeft = false, isRight = false, isHalfRight = false, isHalfLeft = false;
float maxSpeedMotor = 255;


void setup() {
  Serial.begin(115200);

  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  while (error != 0) {
    delay(1000);
    error = ps2x.config_gamepad(13, 11, 12, 10, false, false);  //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    Serial.print("#try config ");
    Serial.println(tryNum);
    tryNum++;
  }
  Serial.println(ps2x.Analog(1), HEX);

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      Serial.println("GuitarHero Controller Found");
      break;
  }

  armServo1.attach(arm1_servo);
  armServo2.attach(arm2_servo);
  gripServo.attach(grip_servo);
  centerServo.attach(center_servo);


  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);

  armServo1.write(90);
  armServo2.write(90);
  gripServo.write(ungrip_angle);
  centerServo.write(center_angle);
}

void loop() {
  if (error == 1)  //skip loop if no controller found
    return;
  ps2x.read_gamepad(false, vibrate);
  read_gamepad();

  arm_control();

  delay(50);
}

void read_gamepad() {
  if (ps2x.ButtonPressed(PSB_PAD_UP)) {  //will be TRUE as long as button is pressed
    isForWard = true;
  }
  if (ps2x.ButtonReleased(PSB_PAD_UP)) {  //will be TRUE as long as button is pressed
    isForWard = false;
    set_motor(0, 0);
  }
  if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
    isBackWard = true;
  }
  if (ps2x.ButtonReleased(PSB_PAD_DOWN)) {
    isBackWard = false;
    set_motor(0, 0);
  }

  if (ps2x.Button(PSB_CIRCLE)) {
    isRight = true;
  } else {
    isRight = false;
  }
  if (ps2x.Button(PSB_SQUARE)) {
    isLeft = true;
  } else {
    isLeft = false;
  }
  if (ps2x.ButtonReleased(PSB_CIRCLE)) {
    isRight = false;
  }
  if (ps2x.ButtonReleased(PSB_SQUARE)) {
    isLeft = false;
  }

  // speed control
  if (ps2x.ButtonPressed(PSB_PAD_RIGHT))  //will be TRUE if button was JUST pressed OR released
  {
    maxSpeedMotor = constrain(maxSpeedMotor + 10, 150, 255);
  }
  if (ps2x.ButtonPressed(PSB_PAD_LEFT))  //will be TRUE if button was JUST pressed OR released
  {
    maxSpeedMotor = constrain(maxSpeedMotor - 10, 150, 255);
  }

  motor_control();
  if (ps2x.Button(PSB_TRIANGLE)) {
    gripServo.write(160);
  } else {
    gripServo.write(60);
  }

  if (ps2x.Analog(PSS_LX) == 0) {
    isCenterIn = true;
  } else isCenterIn = false;

  if (ps2x.Analog(PSS_LX) == 255) {
    isCenterDe = true;
  } else isCenterDe = false;

  // Arm1 servo
  if (ps2x.Analog(PSS_LY) == 255) {
    isArm1In = true;
  } else isArm1In = false;

  if (ps2x.Analog(PSS_LY) == 0) {
    isArm1De = true;
  } else isArm1De = false;

  // Arm2 servo
  if (ps2x.Analog(PSS_RY) == 255) {
    isArm2In = true;
  } else isArm2In = false;

  if (ps2x.Analog(PSS_RY) == 0) {
    isArm2De = true;
  } else isArm2De = false;
}

void motor_control() {
  if (isForWard && isRight)  // forward right
  {
    set_motor(maxSpeedMotor, 0);
  } else if (isForWard && isLeft)  // forward left
  {
    set_motor(0, maxSpeedMotor);
  } else if (isBackWard && isRight)  // BackWard right
  {
    set_motor(-maxSpeedMotor, 0);
  } else if (isBackWard && isLeft)  // BackWard left
  {
    set_motor(0, -maxSpeedMotor);
  } else if (isForWard) {
    set_motor(maxSpeedMotor, maxSpeedMotor);
  } else if (isBackWard) {
    set_motor(-maxSpeedMotor * 3 / 4, -maxSpeedMotor * 3 / 4);
  } else if (isRight) {
    set_motor(maxSpeedMotor, -maxSpeedMotor);
  } else if (isLeft) {
    set_motor(-maxSpeedMotor, maxSpeedMotor);
  } else {
    set_motor(0, 0);
  }
}

void arm_control() {
  if (isArm1In) arm1Servo = constrain(arm1Servo + 4, 0, 180);
  if (isArm2In) arm2Servo = constrain(arm2Servo + 4, 0, 180);
  if (isArm1De) arm1Servo = constrain(arm1Servo - 4, 0, 180);
  if (isArm2De) arm2Servo = constrain(arm2Servo - 4, 0, 180);

  if (isCenterIn) center = constrain(center + 2.3, 0, 180);
  if (isCenterDe) center = constrain(center - 2.3, 0, 180);

  centerServo.write((int)center);
  armServo1.write((int)arm1Servo);
  armServo2.write((int)arm2Servo);
}


void set_motor(int speedB, int speedA) {
  speedA = -speedA;
  if (speedA > 0) {
    speedA = constrain(speedA, 0, 255);
    digitalWrite(MOTOR_R_2, LOW);
    analogWrite(MOTOR_R_1, speedA);
  } else {
    speedA = constrain(speedA, -255, -0);
    digitalWrite(MOTOR_R_2, HIGH);
    analogWrite(MOTOR_R_1, 255 + speedA);
  }
  speedB = -speedB;
  if (speedB > 0) {
    speedB = constrain(speedB, 0, 255);
    digitalWrite(MOTOR_L_1, LOW);
    analogWrite(MOTOR_L_2, speedB);
  } else {
    speedB = constrain(speedB, -255, -0);
    digitalWrite(MOTOR_L_1, HIGH);
    analogWrite(MOTOR_L_2, 255 + speedB);
  }
}
