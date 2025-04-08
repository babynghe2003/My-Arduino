#include <Arduino.h>
#define MOTOR_L_1 1
#define MOTOR_L_2 2
#define MOTOR_R_1 3
#define MOTOR_R_2 4

const uint8_t SENSORS_PIN[] = { 21, 20, 10, 8, 7, 6, 5 };

float Kp = 20;
float Ki = 0.01;
float Kd = 10;
float P;
float I;
float D;
float PIDValue = 0;
float error = 0;
int lastError = 0;
int isRunning = 1;

uint8_t maxspeed = 100;
const uint8_t maxspeedt = 255;
void read_sensors();
void caculate_pid();
void motor_control();
void set_motor(int, int);

uint8_t lastDir = 1;

void setup() {
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);

  for (auto sensorPin : SENSORS_PIN) {
    pinMode(sensorPin, INPUT);
  }
  Serial.begin(115200);
  delay(100);
}

void loop() {
  for (int i = 100; i < maxspeed; i++){
    set_motor(i,i);
    delay(5);
  }


  while (isRunning) {
    maxspeed = constrain(maxspeed + 3, 0,  maxspeedt);
    read_sensors();
    caculate_pid();
    motor_control();
    delay(5);
  }

  set_motor(-200, -200);
  delay(100);
  set_motor(0, 0);
  while (1) {
    delay(1000);
  }
}

void read_sensors() {
  String sensorArray = "";
  int sumSensor = 0;
  for (auto sensorPin : SENSORS_PIN) {
    sensorArray += (char)(digitalRead(sensorPin) + 48);
    sumSensor += digitalRead(sensorPin);
  }
  Serial.println(sensorArray);

  if (sensorArray[0] == '1'){
    lastDir = 0;
  } else if (sensorArray[6] == '1'){
    lastDir = 2;
  }

  if (sensorArray == "0000001") error = 8;
  else if (sensorArray == "0000011") error = 7;
  else if (sensorArray == "0000010") error = 5;
  else if (sensorArray == "0000110") error = 3;
  else if (sensorArray == "0000100") error = 2;
  else if (sensorArray == "0001100") error = 1;
  else if (sensorArray == "0001000") error = 0;
  else if (sensorArray == "0011100") error = 0;
  else if (sensorArray == "0011000") error = -1;
  else if (sensorArray == "0010000") error = -2;
  else if (sensorArray == "0110000") error = -3;
  else if (sensorArray == "0100000") error = -5;
  else if (sensorArray == "1100000") error = -7;
  
  else if (sensorArray == "1000000") error = -8;
  else if (sensorArray == "0111000" || sensorArray == "0111100" || sensorArray == "1111100") error = -9;
  else if (sensorArray == "0001110" || sensorArray == "0011110" || sensorArray == "0011111") error = 9;
  else if (sensorArray == "0000000") {
    if (lastDir == 0) error = -12;
    else if (lastDir == 2) error = 12;
  } else if (sensorArray == "1111111") {
    isRunning = 0;
  }
}

void caculate_pid() {
  P = error;
  if (abs(error) < 6)
    I = I + error;
  else
    I = 0;
  D = error - lastError;
  PIDValue = Kp * P + Ki * I + Kd * D;
  Serial.println(PIDValue);
  lastError = error;
}

void motor_control() {
  if (error < 0)
    set_motor(maxspeed + PIDValue, maxspeed);
  else
    set_motor(maxspeed, maxspeed - PIDValue);
}

void set_motor(int speedLeft, int speedRight) {
  // uncomment line above if any side run reverse
  speedRight = constrain(speedRight, -maxspeed, maxspeed);
  speedLeft = constrain(speedLeft, -maxspeed, maxspeed);
  speedLeft = -speedLeft;
  speedRight = -speedRight;

  if (speedRight > 0) {
    digitalWrite(MOTOR_R_2, LOW);
    analogWrite(MOTOR_R_1, speedRight);
  } else {
    digitalWrite(MOTOR_R_2, HIGH);
    analogWrite(MOTOR_R_1, 255 + speedRight);
  }
  if (speedLeft > 0) {
    digitalWrite(MOTOR_L_1, LOW);
    analogWrite(MOTOR_L_2, speedLeft);
  } else {
    digitalWrite(MOTOR_L_1, HIGH);
    analogWrite(MOTOR_L_2, 255 + speedLeft);
  }
}
