// #include <Servo.h>
// #include <SoftwareSerial.h>

// SoftwareSerial mySerial(7, 8); // RX, TX
#define MOTOR_L_1 0
#define MOTOR_L_2 2
#define MOTOR_R_1 14
#define MOTOR_R_2 12


void setup() {
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  Serial.begin(9600);
  // mySerial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

}
// R-HIGH
int maxspeed = 100;
char code;

void loop() {

  if (Serial.available()){
    code = Serial.read();
    Serial.print(code);
  }
  
    if (code == 'F') {  // di thang
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      forward();
    } else if (code == 'S') {
      // digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(MOTOR_L_1, LOW);
        digitalWrite(MOTOR_L_2, LOW);
                digitalWrite(MOTOR_R_1, LOW);
        digitalWrite(MOTOR_R_2, LOW);
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
  digitalWrite(MOTOR_L_2, HIGH);  
}
void motor_l_lui(int speed){
  digitalWrite(MOTOR_L_1, HIGH);
  digitalWrite(MOTOR_L_2, LOW);
  // Serial.print("HIGH");
}
void motor_r_tien(int speed){
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, HIGH);  
}
void motor_r_lui(int speed){
  digitalWrite(MOTOR_R_1, HIGH);
  digitalWrite(MOTOR_R_2, LOW);
  // Serial.print("HIGH");
}

