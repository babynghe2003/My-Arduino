#define ENA 32
#define MOTOR_L_1 26
#define MOTOR_L_2 27
#define MOTOR_R_1 33
#define MOTOR_R_2 25
#define ENB 14

const uint8_t SENSORS_PIN[] = { 18, 5, 17, 16, 4, 2, 15 };

float Kp = 8;
float Ki = 0.0008;
float Kd = 12;
int P;
int I;
int D;
int PIDValue = 0;
float error = 0;
int lastError = 0;
int isRunning = 1;
bool isNoLine = false;

const uint8_t maxspeeda = 240;
const uint8_t maxspeedb = 245;
const uint8_t minspeeda = 138;
const uint8_t minspeedb = 135;

const int freq = 30000;
const int ENAChanel = 0;
const int ENBChanel = 1;
const int resolution = 8;

void setup() {
  // setup motor
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

  // setup sensors
  for (auto sensorPin : SENSORS_PIN) {
    pinMode(sensorPin, INPUT);
  }
  Serial.begin(115200);
}

void loop() {
  while (isRunning) {
    read_sensors();
    caculate_pid();
    motor_control();
  }
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, LOW);
}

void read_sensors() {
  String sensorArray = "";
  int sumSensor = 0;
  for (auto sensorPin : SENSORS_PIN) {
    sensorArray += (char)(digitalRead(sensorPin) + 48);
    sumSensor += digitalRead(sensorPin);
  }
  if (sumSensor > 0){
    isNoLine = false;
  }else{
    isNoLine = true;
  }
  // Serial.println(sensorArray);

 if (sensorArray == "0000001") error = 6;
  else if (sensorArray == "0000011") error = 5;
  else if (sensorArray == "0000010") error = 4;
  else if (sensorArray == "0000110") error = 3;
  else if (sensorArray == "0000100") error = 2;
  else if (sensorArray == "0001100") error = 1;
  else if (sensorArray == "0001000") error = 0;
  else if (sensorArray == "0011000") error = -1;
  else if (sensorArray == "0010000") error = -2;
  else if (sensorArray == "0110000") error = -3;
  else if (sensorArray == "0100000") error = -4;
  else if (sensorArray == "1100000") error = -5;
  else if (sensorArray == "1000000") error = -6;
  else if (sensorArray == "0111000" || sensorArray == "0111100") error = -6;
  else if (sensorArray == "0001110" || sensorArray == "0011110") error = 6;
  else if (sensorArray == "0000000") {
    if (error < -3) error = -6;
    else if (error > 3) error = 6;
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

}


void set_motor(int speedA, int speedB) {
  int a = 0, b = 0;
  if (speedA < minspeeda) {
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_R_2, HIGH);
    if (abs(error) == 5)
      speedA = 80;
    else
      speedA = minspeeda + 10 ;
    b = 15;
  } else {
    digitalWrite(MOTOR_R_1, HIGH);
    digitalWrite(MOTOR_R_2, LOW);
  }

  if (speedB < minspeedb) {
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, HIGH);
    if (abs(error) == 5)
      speedB = 80;
    else
      speedB = minspeedb + 10;
    a = 15;
  } else {
    digitalWrite(MOTOR_L_1, HIGH);
    digitalWrite(MOTOR_L_2, LOW);
  }
  ledcWrite(ENAChanel, constrain(speedA - a, 0,maxspeeda));
  ledcWrite(ENBChanel, constrain(speedB - b, 0,maxspeedb));
}

void motor_control() {
  if (error < 0)
    set_motor(maxspeeda + PIDValue, maxspeedb);
  else 
    set_motor(maxspeeda, maxspeedb - PIDValue);
}


