#include <PS2X_lib.h>  //for v1.6
#include <ESP32Servo.h>
#define PS2_DAT 23  //MISO  19
#define PS2_CMD 22  //MOSI  23
#define PS2_SEL 21  //SS     5
#define PS2_CLK 19  //SLK   18

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures true
#define rumble false

PS2X ps2x;  // create PS2 Controller Class


int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;

#define MOTOR_L_1 27
#define MOTOR_L_2 14
#define MOTOR_R_1 25
#define MOTOR_R_2 26

Servo centerServo;
Servo gripServo;
Servo armServo1;
Servo armServo2;
VL53L0X sensor;
const int freq = 30000;
const int ENAChanel = 6;
const int ENBChanel = 7;
const int resolution = 8;
int maxspeed = 255;

float acen = 0.25;
float center = 75;
int grip = 120;
float arm1Servo = 180;
float arm2Servo = 65;

bool isArm1In = false, isArm2In = false, isArm1De = false, isArm2De = false, isCenterIn = false, isCenterDe = false;

bool isForWard = false, isBackWard = false, isLeft = false, isRight = false, isHalfRight = false, isHalfLeft = false;
float maxSpeedMotor = 255;

void setup() {
  // start Serial
  Serial.begin(115200);

  // config motor
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);

  digitalWrite(MOTOR_L_1, HIGH);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, HIGH);
  digitalWrite(MOTOR_R_2, LOW);

  ledcSetup(ENAChanel, freq, resolution);
  ledcSetup(ENBChanel, freq, resolution);
  ledcAttachPin(MOTOR_L_2, ENAChanel);
  ledcAttachPin(MOTOR_R_2, ENBChanel);

  // config servo
  centerServo.attach(16);
  gripServo.attach(5);
  armServo1.attach(17);
  armServo2.attach(18);

  armServo1.write(arm1Servo);
  armServo2.write(arm2Servo);
  gripServo.write(grip);
  centerServo.write(center);

  // connect ps2 controller
  while (error != 0) {
    delay(1000);  // 1 second wait
    //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print("#try config ");
    Serial.println(tryNum);
    tryNum++;
  }

  Serial.println(ps2x.Analog(1), HEX);

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println(" Unknown Controller type found ");
      break;
    case 1:
      Serial.println(" DualShock Controller found ");
      break;
    case 2:
      Serial.println(" GuitarHero Controller found ");
      break;
    case 3:
      Serial.println(" Wireless Sony DualShock Controller found ");
      break;
  }
}
/*
ButtonPressed
ButtonReleased
Button
NewButtonState
*/
void loop() {

  if (type == 1) {                      //DualShock Controller
    ps2x.read_gamepad(false, vibrate);  //read controller and set large motor to spin at 'vibrate' speed


    //########################################## Car ##################################################

    if (ps2x.ButtonPressed(PSB_PAD_UP)) {
      isForWard = true;
    }
    if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
      isBackWard = true;
    }
    if (ps2x.ButtonReleased(PSB_PAD_UP)) {
      isForWard = false;
      motorLeft(-maxSpeedMotor);
      motorRight(-maxSpeedMotor);
      delay(40);
    }
    if (ps2x.ButtonReleased(PSB_PAD_DOWN)) {
      isBackWard = false;
      motorLeft(maxSpeedMotor);
      motorRight(maxSpeedMotor);
      delay(40);
    }

    if (ps2x.Analog(PSS_RX) == 255) {
      isHalfRight = true;
    } else {
      isHalfRight = false;
    }
    if (ps2x.Analog(PSS_RX) == 0) {
      isHalfLeft = true;
    } else {
      isHalfLeft = false;
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
    motorControl();

    // speed control
    if (ps2x.ButtonPressed(PSB_PAD_RIGHT))  //will be TRUE if button was JUST pressed OR released
    {
      maxSpeedMotor = constrain(maxSpeedMotor + 10, 150, 255);
      maxSpeedMotor = constrain(maxSpeedMotor + 10, 150, 255);
    }
    if (ps2x.ButtonPressed(PSB_PAD_LEFT))  //will be TRUE if button was JUST pressed OR released
    {
      maxSpeedMotor = constrain(maxSpeedMotor - 10, 150, 255);
      maxSpeedMotor = constrain(maxSpeedMotor - 10, 150, 255);
    }

    //########################################## Arm ##################################################
    if (ps2x.Button(PSB_TRIANGLE)) {
      gripServo.write(135);
    } else {
      gripServo.write(80);
    }
    // if (ps2x.ButtonReleased(PSB_TRIANGLE)){
    //     // if (grip == 50) grip = 120;
    //     // else grip = 50;
    //   gripServo.write(120);
    // }
    if (ps2x.ButtonPressed(PSB_CROSS))  //will be TRUE if button was JUST pressed OR released
    {
      gripServo.write(135);
      delay(85);
      gripServo.write(80);
    }
    // Center servo
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

    control_arm();
  }

  delay(50);
}

void control_arm() {
  if (isArm1In) arm1Servo = constrain(arm1Servo + 4, 0, 180);
  if (isArm2In) arm2Servo = constrain(arm2Servo + 4, 0, 180);
  if (isArm1De) arm1Servo = constrain(arm1Servo - 4, 0, 180);
  if (isArm2De) arm2Servo = constrain(arm2Servo - 4, 0, 180);

  if (isCenterIn) center = constrain(center + 2.3, 0, 180);
  if (isCenterDe) center = constrain(center - 2.3, 0, 180);

  centerServo.write((int)center);
  armServo1.write((int)arm1Servo);
  armServo2.write((int)arm2Servo);

  Serial.print(ps2x.Analog(PSS_LX));
  Serial.print(' ');
  Serial.print(isCenterIn);
  Serial.print(' ');
  Serial.print(isCenterDe);
  Serial.print(' ');
  Serial.print("Center: ");
  Serial.print(center);
  Serial.print(", Arm1: ");
  Serial.print(arm1Servo);
  Serial.print(", Arm2: ");
  Serial.println(arm2Servo);
}

void motorControl() {
  if (isForWard && isHalfRight)  // forward right
  {
    motorLeft(maxSpeedMotor);
    motorRight(map(maxSpeedMotor, 150, 255, 10, 80));
  } else if (isForWard && isHalfLeft)  // forward left
  {
    motorLeft(map(maxSpeedMotor, 150, 255, 10, 80));
    motorRight(maxSpeedMotor);
  } else if (isForWard && isRight)  // forward right
  {
    motorLeft(map(maxSpeedMotor, 150, 255, 150, 220));
    motorRight(-map(maxSpeedMotor, 150, 255, 100, 130));
  } else if (isForWard && isLeft)  // forward left
  {
    motorLeft(-map(maxSpeedMotor, 150, 255, 100, 130));
    motorRight(map(maxSpeedMotor, 150, 255, 150, 220));
  } else if (isBackWard && isRight)  // BackWard right
  {
    motorLeft(map(maxSpeedMotor, 150, 255, 130, 150));
    motorRight(-maxSpeedMotor);
  } else if (isBackWard && isLeft)  // BackWard left
  {
    motorLeft(-maxSpeedMotor);
    motorRight(map(maxSpeedMotor, 150, 255, 130, 150));
  } else if (isForWard) {
    motorLeft(maxSpeedMotor);
    motorRight(maxSpeedMotor);
  } else if (isBackWard) {
    motorLeft(-map(maxSpeedMotor, 150, 255, 150, 200));
    motorRight(-map(maxSpeedMotor, 150, 255, 150, 200));
  } else if (isRight) {
    motorLeft(map(maxSpeedMotor, 150, 255, 150, 190));
    motorRight(-map(maxSpeedMotor, 150, 255, 150, 190));
  } else if (isLeft) {
    motorLeft(-map(maxSpeedMotor, 150, 255, 150, 190));
    motorRight(map(maxSpeedMotor, 150, 255, 150, 190));
  }
}

void motorLeft(int speed) {
  if (speed > 0) {
    digitalWrite(MOTOR_L_1, HIGH);
    ledcWrite(MOTOR_L_2, constrain(255 - abs(speed), 0, 255));
  } else {
    digitalWrite(MOTOR_L_1, LOW);
    ledcWrite(MOTOR_L_2, constrain(abs(speed), 0, 255));
  }
}

void motorRight(int speed) {
  if (speed > 0) {
    digitalWrite(MOTOR_R_1, HIGH);
    ledcWrite(MOTOR_R_2, constrain(255 - abs(speed), 0, 255));
  } else {
    digitalWrite(MOTOR_R_1, LOW);
    ledcWrite(MOTOR_R_2, constrain(abs(speed), 0, 255));
  }
}