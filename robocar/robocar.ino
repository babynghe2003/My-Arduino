#define ENA 32
#define MOTOR_L_1 26
#define MOTOR_L_2 27
#define MOTOR_R_1 33
#define MOTOR_R_2 25
#define ENB 14

const uint8_t SENSORS_PIN[] = { 18, 5, 17, 16, 4, 2, 15 };

float Kp = 5;
float Ki = 0.0001;
float Kd = 12;
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
const uint8_t minspeeda = 137;
const uint8_t minspeedb = 137;

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

bool stoped = 0;

void loop() {
  while (isRunning) {
    read_sensors();
    caculate_pid();
    motor_control();
  }
  if (stoped == 0){
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, HIGH);
    ledcWrite(ENAChanel, 200);
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_R_2, HIGH);
    ledcWrite(ENBChanel, 200);
    delay(100);
    stoped = 1;
  }
  ledcWrite(ENAChanel, 0);
  ledcWrite(ENBChanel, 0);
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

 if (sensorArray == "0000001"){
  error = 6;
 } 
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
  else if (sensorArray == "1000000"){
     error = -6;
  }
  else if (sensorArray == "0000000") {
    if (error < 0) error = -7;
    else if (error > 0) error = 7;
  } else if (sensorArray == "1111111" || sensorArray == "1001001" || sensorArray == "1011001" 
  || sensorArray == "1001101"|| sensorArray == "1011101"|| sensorArray == "1010001"|| sensorArray == "1000101" ) {
    isRunning = 0;
  }
}

void caculate_pid() {
  P = error;
  if (abs(error) < 7)
  I = I + Ki*error;
  D = error - lastError;
  PIDValue = Kp * P + I + Kd * D;
  Serial.println(PIDValue);

}


void set_motor(int speedA, int speedB) {
  int a = 0, b = 0;
  if (error == -7) {
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_R_2, HIGH);
    speedA = 200;
    // b = 50;
  } else {
    digitalWrite(MOTOR_R_1, HIGH);
    digitalWrite(MOTOR_R_2, LOW);
  }

  if (error == 7) {
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, HIGH);
    speedB = 200;
    // a = 50;
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


