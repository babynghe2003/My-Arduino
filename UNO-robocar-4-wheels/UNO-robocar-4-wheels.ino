#define MOTOR_L_1 7
#define MOTOR_L_2 6
#define MOTOR_R_1 5
#define MOTOR_R_2 4

const uint8_t SENSORS_PIN[] = { A0, A1, A2, A3, A4 };
uint16_t sensor_max[] = {500,500,500,500,500};
uint16_t sensor_min[] = {50,50,50,50,50};

float Kp = 100;
float Ki = 0.0002;
float Kd = 0;
int P;
int I;
int D;
int PIDValue = 0;
float error = 0;
int lastError = 0;
int isRunning = 1;
bool isNoLine = false;

const uint8_t maxspeeda = 255;
const uint8_t maxspeedb = 255;
const uint8_t minspeeda = 10;
const uint8_t minspeedb = 10;

void setup() {
  // setup motor
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  pinMode(A5, INPUT);

  // setup sensors
  for (auto sensorPin : SENSORS_PIN) {
    pinMode(sensorPin, INPUT);
  }
  Serial.begin(115200);
  calibrate();
  delay(100);
  // for (auto mx : sensor_max){
  //   Serial.print(mx);
  //   Serial.print(' ');
  // }
  // Serial.println(' ');
  // for (auto mx : sensor_min){
  //   Serial.print(mx);
  //   Serial.print(' ');
  // }
  // Serial.println(' ');
  // set_motor(0,0);
  // while(1){
  //   read_sensors();
  //   for (auto mx : SENSORS_PIN){
  //     Serial.print(analogRead(mx));
  //     Serial.print(' ');
  //   }
  //   Serial.println();
  //   delay(50);
  // };
}

bool stoped = 0;
long last = -2000; 
long last_state = 1;
void loop() {
  // set_motor(200, 200);
  // delay(1000);
  // set_motor(-200, 200);
  // while(1);
  // set_motor(50,50);
  while (isRunning) {
    if (last_state == 1 && digitalRead(A5) == 0) last = millis();
    last_state = digitalRead(A5);

   
    if (millis() - last < 2000){
      int speed = millis() - last;
      speed = constrain(speed, 0, 500);
      speed = map(speed, 0, 500, 50, 255);
      set_motor(speed,speed);
    } else {
      last_state = 1;
      read_sensors();
      caculate_pid();
      motor_control();
    }
    delay(10);
  }
  if (stoped == 0){
    set_motor(-255, -255);
    delay(100);
    stoped = 1;
    set_motor(0, 0);
    while (1){
      
    }
  }
  

}

void calibrate(){
  set_motor(0,0);
  for (int i = 0; i < 1000; i++){
    delay(1);
    for (int i = 0; i < 5; i++){
      int value = analogRead(SENSORS_PIN[i]);
      sensor_max[i] = max(sensor_max[i], value);
      sensor_min[i] = min(sensor_min[i], value);
    }
  }
}

void read_sensors() {
  String sensorArray = "";
  int sumSensor = 0;
  for (int i = 0; i < 5; i++) {
    int value = analogRead(SENSORS_PIN[i]);
    char cval = value > (sensor_max[i] - sensor_min[i])/5*1 + sensor_min[i] ? '1' : '0';
    sensorArray += cval;
  }
  if (sumSensor > 0){
    isNoLine = false;
  }else{
    isNoLine = true;
  }
  Serial.println(sensorArray);

  if (sensorArray == "00001") error = 5;
  else if (sensorArray == "00111") error = 4;
  else if (sensorArray == "00011") error = 3;
  else if (sensorArray == "00010") error = 2;
  else if (sensorArray == "00110") error = 1;
  else if (sensorArray == "00100") error = 0;
  else if (sensorArray == "01110") error = 0;
  else if (sensorArray == "01100") error = -1;
  else if (sensorArray == "01000") error = -2;
  else if (sensorArray == "11000") error = -3;
  else if (sensorArray == "11100") error = -4;
  else if (sensorArray == "10000") error = -5;
  else if (sensorArray == "00000"){
    if (error > 0) error = 6;
    else error = -6;
  }
   else if (sensorArray == "11111" || sensorArray == "10101" ) {
    isRunning = 0;
  }
  // error = -error;
}

void caculate_pid() {
  P = error;
  if (abs(error) < 6)
  I = I + Ki*error;
  D = error - lastError;
  PIDValue = Kp * P + I + Kd * D;
  lastError = error;
  // Serial.println(PIDValue);

}


void set_motor(int speedA, int speedB) {
  speedA = -speedA;
  if (speedA > 0){
    speedA = constrain(speedA, minspeeda, 255);
    digitalWrite(MOTOR_R_2, LOW);
    analogWrite(MOTOR_R_1, speedA);
  } else{
    speedA = constrain(speedA, -255, -minspeeda);
    digitalWrite(MOTOR_R_2, HIGH);
    analogWrite(MOTOR_R_1,255 + speedA);
  }
  speedB = -speedB;
  if (speedB > 0){
    speedB = constrain(speedB, minspeedb, 255);
    digitalWrite(MOTOR_L_1, LOW);
    analogWrite(MOTOR_L_2, speedB);
  } else{
    speedB = constrain(speedB, -255, -minspeedb);
    digitalWrite(MOTOR_L_1, HIGH);
    analogWrite(MOTOR_L_2, 255 + speedB);
  }
}

void motor_control() {
  if (error < 0)
    set_motor(maxspeeda , maxspeedb + PIDValue);
  else 
    set_motor(maxspeeda - PIDValue, maxspeedb );
}

