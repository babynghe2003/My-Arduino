#include "BluetoothSerial.h"
#include <ESP32Servo.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#define ENA 14
#define MOTOR_L_1 26
#define MOTOR_L_2 27
#define MOTOR_R_1 33
#define MOTOR_R_2 25
#define ENB 32

Servo centerServo;
Servo gripServo;
Servo armServo1;
Servo armServo2;

const int freq = 30000;
const int ENAChanel = 6;
const int ENBChanel = 7;
const int resolution = 8;
int maxspeed = 255;

float acen = 0.015;
float center = 90;
float target_center = 90;
int grip = 50;
float arm1Servo = 120, target_arm1 = 120;
float arm2Servo = 60, target_arm2 = 60;

int hori = 90;
int verti = 90;

bool isArm1In = false, isArm2In = false, isArm1De = false, isArm2De = false, isCenterIn = false, isCenterDe = false;

void setup() {
  
  Serial.begin(115200);
  SerialBT.begin("UchihahahCar1"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
    pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
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
  ledcAttachPin(ENA, ENAChanel);
  ledcAttachPin(ENB, ENBChanel);

  centerServo.attach(16);
  gripServo.attach(5);
  armServo1.attach(17);
  armServo2.attach(18);

  armServo1.write(arm1Servo);
  armServo2.write(arm2Servo);
  gripServo.write(grip);
  centerServo.write(center);      

}
String message = "";


void loop() {

  if (SerialBT.available()) {
    char letter = SerialBT.read();
    Serial.println(letter);
    if (letter == 'F')
      forward();
    else if (letter == 'G')
      backward();
    else if (letter == 'L')
      left();
    else if (letter == 'R')
      right();
    else if (letter == 'Q')
      forwardLeft();
    else if (letter == 'E')
      forwardRight();
    else if (letter == 'Z')
      backwardLeft();
    else if (letter == 'C')
      backwardRight();
    else if (letter == 'S')
      stop();
    else if (letter == 'X'){
      isCenterIn = true;
    }
    else if (letter == 'Y'){
      isCenterDe = true;
    }
    else if (letter == 'l'){
      isArm1In = true;
    }
    else if (letter == 'o'){
      isArm1De = true;
    }
    else if (letter == 'p'){
      isArm2In = true;
    }
    else if (letter == 'k'){
      isArm2De = true;
    }
    else if (letter == 'M'){
      if (grip == 50) grip = 150;
      else grip = 50; 
      gripServo.write(grip);      
    }
  }

  control_arm();
  
  // Serial.print(center);
  // Serial.print(' ');
  // Serial.println(target_center);

}

void control_arm(){
  if (isArm1In) target_arm1 = constrain(target_arm1 + 0.003, 0, 180); 
  if (isArm2In) target_arm2 = constrain(target_arm2 + 0.003, 0, 180); 
  if (isArm1De) target_arm1 = constrain(target_arm1 - 0.003, 0, 180); 
  if (isArm2De) target_arm2 = constrain(target_arm2 - 0.003, 0, 180); 

  if (isCenterIn) target_center = constrain(target_center + 0.003, 0, 180); 
  if (isCenterDe) target_center = constrain(target_center - 0.003, 0, 180); 

  center += (target_center - center)*acen;
  centerServo.write((int)center);

  arm1Servo += (target_arm1 - arm1Servo) *acen;
  armServo1.write((int)arm1Servo);

  arm2Servo += (target_arm2 - arm2Servo) *acen;
  armServo2.write((int)arm2Servo);
}

void setmotor(int speeda, int speedb){
  ledcWrite(ENAChanel, speeda);
  ledcWrite(ENBChanel, speedb);
}

void stop(){
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, LOW);  
  isArm1In = false;
  isArm2In = false;
  isArm1De = false;
  isArm2De = false;
  isCenterIn = false;
  isCenterDe = false;
}

void forward(){
  Serial.println(" is foward");

  digitalWrite(MOTOR_L_1, HIGH);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, HIGH);
  digitalWrite(MOTOR_R_2, LOW);  
  setmotor(maxspeed,maxspeed - 7);
}

void forwardLeft(){
  Serial.println(" is foward left");

  digitalWrite(MOTOR_L_1, HIGH);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, HIGH);
  digitalWrite(MOTOR_R_2, LOW);  
  setmotor(0,maxspeed-40);
}

void forwardRight(){
  digitalWrite(MOTOR_L_1, HIGH);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, HIGH);
  digitalWrite(MOTOR_R_2, LOW);  
  setmotor(maxspeed-40,0);
}

void backward(){
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, HIGH);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, HIGH);  
  setmotor(maxspeed,maxspeed);
}

void backwardLeft(){
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, HIGH);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, HIGH);  
  setmotor(maxspeed - 40,0);
}

void backwardRight(){
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, HIGH);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, HIGH);  
  setmotor(0,maxspeed - 40);
}

void left(){
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, HIGH);
  digitalWrite(MOTOR_R_1, HIGH);
  digitalWrite(MOTOR_R_2, LOW);
  setmotor(maxspeed - 55,maxspeed - 55);
}
void right(){
  digitalWrite(MOTOR_L_1, HIGH);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, HIGH);
  setmotor(maxspeed - 55,maxspeed - 55);
}

