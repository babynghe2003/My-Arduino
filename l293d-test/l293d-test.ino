#include <AFMotor.h>

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor4(2, MOTOR12_1KHZ);

#define MOTOR_R_1 16
#define MOTOR_R_2 17
#define MOTOR_L_1 5
#define MOTOR_L_2 18
const uint8_t SENSORS_PIN[] = { A0, A1, A2, A3 };
int error = 0;
bool isRunning = true;
void setup() {
  motor1.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(200);
  motor4.setSpeed(200);
  Serial.begin(9600);
}

void loop() {
  if (!isRunning){
    motor1.setSpeed(0);
    motor4.setSpeed(0);
    return;
  }
  read_sensors();
  control_motor();
}

void read_sensors() {
  String sensorArray = "";
  for (auto sensorPin : SENSORS_PIN) {
    sensorArray += (char)(digitalRead(sensorPin) + 48);
  }

  Serial.println(sensorArray);

  if (sensorArray == "1000") {
    error = 3;
  } else if (sensorArray == "1110") {
    error = 2;
  } else if (sensorArray == "0100") {
    error = 1;
  } else if (sensorArray == "0110") {
    error = 0;
  } else if (sensorArray == "0010") {
    error = -1;
  } else if (sensorArray == "0111") {
    error = -2;
  } else if (sensorArray == "0001") {
    error = -3;
  } else if (sensorArray == "0000") {
    if (error < 0) {
      error = -4;
    } else if (error > 0) {
      error = 4;
    }
  }else if (sensorArray == "1111"){
    isRunning = false;
  }
}

void control_motor() {
  int speedA, speedB;
  if (error > 0) {
    if (error == 4){
      speedA = 150;
      motor1.run(BACKWARD);
    } 
    else{
      motor1.run(FORWARD);
      speedA = 255 - error * 30;

    } 
    motor4.run(FORWARD);
    speedB = 255;
  } else {
    if (error == -4) {
      motor4.run(BACKWARD);
      speedB = 150;
    } else {
      speedB = 255 + error * 30;
      motor4.run(FORWARD);
    }
    motor1.run(FORWARD);
    speedA = 255;
  }
  motor1.setSpeed(speedA );
  motor4.setSpeed(speedB );
}
