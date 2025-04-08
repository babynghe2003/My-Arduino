#define MOTOR_L_1 4
#define MOTOR_L_2 5
#define MOTOR_R_1 7
#define MOTOR_R_2 6

void setup() {

  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  Serial.begin(9600);
}
// R-HIGH
int maxspeed = 200;
char code;

void loop() {
  motor_l_tien(200);
  delay(3000);
  motor_l_lui(100);
  delay(3000);
  motor_l_lui(0);
  motor_r_tien(200);
  delay(3000);
  motor_r_lui(100);
  delay(3000);
  motor_r_tien(0);
}


void motor_l_tien(int speed){
  // analogWrite(MOTOR_L_1, 0);
  // analogWrite(MOTOR_L_2, constrain(speed, 0, 255));  
  digitalWrite(MOTOR_L_1, HIGH);
  digitalWrite(MOTOR_L_2, LOW);
  Serial.print("L tien");
}
void motor_l_lui(int speed){
  // analogWrite(MOTOR_L_1, 255);
  // analogWrite(MOTOR_L_2, constrain(255 - speed, 0, 255));
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, HIGH);
}
void motor_r_tien(int speed){
  // analogWrite(MOTOR_R_1, 0);
  // analogWrite(MOTOR_R_2, constrain(speed, 0, 255));  
  digitalWrite(MOTOR_R_1, HIGH);
  digitalWrite(MOTOR_R_2, LOW);
}
void motor_r_lui(int speed){
  // analogWrite(MOTOR_R_1, 255);
  // analogWrite(MOTOR_R_2, constrain(255-speed, 0, 255));
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, HIGH);
}

