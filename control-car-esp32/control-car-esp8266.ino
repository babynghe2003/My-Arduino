#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
#define MOTOR_L_1 33
#define MOTOR_L_2 25
#define MOTOR_R_1 26
#define MOTOR_R_2 27

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("Program begun!");
  SerialBT.begin("CampusCarMP1");  //Bluetooth device name

  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  Serial.begin(115200);

}
// R-HIGH
int maxspeed = 250;
char code;

void loop() {

  if (SerialBT.available()) {
    code = SerialBT.read(); // read from serial or bluetooth
    Serial.println(code);

    if (code == 'F') {  // di thang
      forward();
    } else if (code == 'S') {
      stop();
    } else if (code == 'G') {
      backward();
    } else if (code == 'Q') {
      forwardLeft();
    } else if (code == 'E') {
      forwardRight();
    } else if (code == 'L') {
      left();
    } else if (code == 'R') {
      right();
    } else if (code == 'Z') {
      backwardLeft();
    } else if (code == 'C') {
      backwardRight();
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
}
void motor_r_tien(int speed){
  digitalWrite(MOTOR_R_1, LOW);
  analogWrite(MOTOR_R_2, constrain(speed, 0, 255));  
}
void motor_r_lui(int speed){
  digitalWrite(MOTOR_R_1, HIGH);
  analogWrite(MOTOR_R_2, constrain(255-speed, 0, 255));
}


