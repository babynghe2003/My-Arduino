#include <SoftwareSerial.h>

#define MOTOR_L_1 3
#define MOTOR_L_2 5
#define MOTOR_R_1 9
#define MOTOR_R_2 6

#define SV_1 13
#define SV_2 12
#define SV_3 11


SoftwareSerial mySerial(1, 0); // RX, TX
byte speed_Coeff = 3;
char data;
byte armVal, maxspeed = 0;
int sv1Val = 2240, sv2Val = 750, sv3Val = 1795;

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  Serial.println("BLT OK");
  pinMode(LED_BUILTIN, OUTPUT);
  mySerial.begin(115200);
}
void loop(){
  forward();
  delay(3000);
  backward();
  delay(3000);
  right();
  delay(3000);
  left();
  delay(3000);
}
void motor_control() {
  if (mySerial.available()) {
    data = mySerial.read();
    Serial.println(data);
    if (data == 'F'){
       forward();
       digitalWrite(LED_BUILTIN, HIGH); 
    } 
    if (data == 'B')  backward();
    if (data == 'L')  left();
    if (data == 'R')  right();
    if (data == 'G')  forwardLeft();
    if (data == 'I')  forwardRight();
    if (data == 'H')  backwardLeft();
    if (data == 'J')  backwardRight();
    if (data == 'S') { 
      stop(); 
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
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
  digitalWrite(MOTOR_L_2, HIGH);  
}
void motor_l_lui(int speed){
  digitalWrite(MOTOR_L_1, HIGH);
  digitalWrite(MOTOR_L_2, LOW);
}
void motor_r_tien(int speed){
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, HIGH);  
}
void motor_r_lui(int speed){
  digitalWrite(MOTOR_R_1, HIGH);
  digitalWrite(MOTOR_R_2, LOW);
  Serial.print("HIGH");
}