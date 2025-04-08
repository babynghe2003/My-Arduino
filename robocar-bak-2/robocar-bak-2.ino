
#define MOTOR_L_1 3
#define MOTOR_L_2 5
#define MOTOR_R_1 9
#define MOTOR_R_2 6

uint8_t sensor_pins[4] = {A3, A2, A1, A0};
int error = 0;
bool isRunning = true;
int maxspeed = 250;
void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  Serial.println("BLT OK");
  for (auto pin : sensor_pins){
    pinMode(pin, INPUT);
  }
  pinMode(LED_BUILTIN, OUTPUT);
}
void loop(){
  while (isRunning){
    read_sensor();
    control_motor();
  }
  motor_r_tien(0);
  motor_l_tien(0);
}

void read_sensor(){
  String sensor = "";
  for (auto pin: sensor_pins){
    sensor += char(digitalRead(pin) + 48);
  }
  Serial.print(sensor);

  if (sensor == "1000"){
   error = 3;
 }else if (sensor == "1110"){
   error = 2;
 } else if (sensor == "0100"){
   error = 1;
 } else if (sensor == "0110"){
   error = 0;
 }else if (sensor == "0010"){
   error = -1;
 }else if (sensor == "0111"){
   error = -2;
 }else if (sensor == "0001"){
   error = -3;
 }else if (sensor == "0000") {
    if (error < 0)
      error = -4;
    else 
      error = 4;
  }else if (sensor == "1111"){
    isRunning = false;
  }
  Serial.print(error);
}

void control_motor(){
    int speedA = maxspeed - error*40;
  int speedB = maxspeed + error*40;
  if (error >= 0){
    speedB = maxspeed + error * 40;
    speedA = maxspeed - error*40;
    if (error ==  4){
      motor_l_lui( 200);
    }else{
      motor_l_tien(speedA);
    }
    motor_r_tien(speedB);
    Serial.println("Left");
  }else {
    speedB = maxspeed + error * 40;
    speedA = maxspeed - error*40;
    if (error == -4){
      motor_r_lui( 200);
    }else{
      motor_r_tien(speedB);
    }
    motor_l_tien(speedA);
    Serial.println("Right");
  }

}

void motor_l_tien(int speed){
  analogWrite(MOTOR_L_1, 0);
  analogWrite(MOTOR_L_2, constrain(speed, 0, 255));  
}
void motor_l_lui(int speed){
  analogWrite(MOTOR_L_1, constrain(speed, 0, 255));
  analogWrite(MOTOR_L_2, 0);
}
void motor_r_tien(int speed){
  analogWrite(MOTOR_R_1, 0);
  analogWrite(MOTOR_R_2, constrain(speed, 0, 255));  
}
void motor_r_lui(int speed){
  analogWrite(MOTOR_R_1, constrain(speed, 0, 255));
  analogWrite(MOTOR_R_2, 0);
  Serial.print("HIGH");
}