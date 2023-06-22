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

float center = 90;
float target_center = 90;
float acenter = 0.1;
int grip = 50;
int arm1Servo = 120;
int arm2Servo = 60;

int hori = 90;
int verti = 90;

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
    if (letter != '\n'){
      message += letter;
    }else{
      if (message == "F")
        forward();
      else if (message == "G")
        backward();
      else if (message == "L")
        left();
      else if (message == "R")
        right();
      else if (message == "Q")
        forwardLeft();
      else if (message == "E")
        forwardRight();
      else if (message == "Z")
        backwardLeft();
      else if (message == "C")
        backwardRight();
      else if (message == "S")
        stop();
      else if (message == "X"){
        target_center = constrain(target_center + 8, 0, 180);
      }
      else if (message == "Y"){
        target_center = constrain(target_center - 8, 0, 180);
      }
      else if (message == "M"){

        if (grip == 50) grip = 150;
        else grip = 50; 
        gripServo.write(grip);      
      }
      else if (message[0] == 'J'){
        armServo1.write(message.substring(1).toInt());
        // int hori = message.substring(1).toInt() - 90;
        // if (arm1Servo + hori < 180 && arm1Servo + hori > 0 && arm2Servo + hori < 180 && arm2Servo + hori > 0){
        //   arm1Servo += hori;
        //   arm2Servo += hori;

        //   // armServo1.write(arm1Servo);
        //   // armServo2.write(arm2Servo);
        // }
      
      }
      else if (message[0] == 'K'){
        armServo2.write(message.substring(1).toInt());
        // int verti = message.substring(1).toInt() - 90;
        // if (arm1Servo + verti < 180 && arm1Servo + verti > 0 && arm2Servo - verti < 180 && arm2Servo - verti > 0){
        //   arm1Servo += verti;
        //   arm2Servo -= verti;
        //   // armServo1.write(arm1Servo);
        //   // armServo2.write(arm2Servo);
        // }

        
      }


    }
    Serial.println(message);
  }
  center += (target_center - center)*acenter;
  centerServo.write((int)center);
  // Serial.print(center);
  // Serial.print(' ');
  // Serial.println(target_center);

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