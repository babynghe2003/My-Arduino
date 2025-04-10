
#define MOTOR_L_1 32
#define MOTOR_L_2 33
#define MOTOR_R_1 25
#define MOTOR_R_2 26


const uint8_t SENSORS_PIN[] = {4, 16, 17, 5, 19};

float Kp = 13;
float Ki = 0.0002;
float Kd = 12;
int P;
int I;
int D;
int PIDValue = 0;
float error = 0;
int lastError = 0;
int isRunning = 1;
bool isNoLine = false;

const uint8_t maxspeeda = 185;
const uint8_t maxspeedb = 185;
const uint8_t minspeeda = 10;
const uint8_t minspeedb = 10;

const int freq = 3000;
const int ENAChanel = 5;
const int ENBChanel = 6;
const int resolution = 8;

void setup() {
  // setup motor
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);

  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_R_1, LOW);

  ledcSetup(ENAChanel, freq, resolution);
  ledcSetup(ENBChanel, freq, resolution);
  ledcAttachPin(MOTOR_L_2, ENAChanel);
  ledcAttachPin(MOTOR_R_1, ENBChanel);

  // setup sensors
  for (auto sensorPin : SENSORS_PIN) {
    pinMode(sensorPin, INPUT);
  }
  Serial.begin(115200);
  delay(500);
}

bool stoped = 0;

void loop() {
  // set_motor(50,50);
  while (isRunning) {
    read_sensors();
    caculate_pid();
    motor_control();
    delay(20);
  }
  if (stoped == 0){
    digitalWrite(MOTOR_R_2, LOW);
    digitalWrite(MOTOR_R_1, HIGH);
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, HIGH);
    delay(100);
    stoped = 1;
    digitalWrite(MOTOR_L_1, LOW);
    ledcWrite(ENBChanel, 0);
    digitalWrite(MOTOR_R_1, LOW);
    ledcWrite(ENAChanel, 0);
    while (1){
      
    }
  }
  

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
   else if (sensorArray == "11111" ) {
    isRunning = 0;
  }
}

void caculate_pid() {
  P = error;
  if (abs(error) < 6)
  I = I + Ki*error;
  D = error - lastError;
  PIDValue = Kp * P + I + Kd * D;
  // Serial.println(PIDValue);

}


void set_motor(int speedA, int speedB) {
  speedA = -speedA;
  if (speedA > 0){
    speedA = constrain(speedA, minspeeda, maxspeeda);
    digitalWrite(MOTOR_R_2, LOW);
    ledcWrite(ENAChanel, speedA);
  } else{
    speedA = constrain(speedA, -maxspeeda, -minspeeda);
    digitalWrite(MOTOR_R_2, HIGH);
    ledcWrite(ENAChanel,255 + speedA);
  }
  speedB = - speedB;
  if (speedB > 0){
    speedB = constrain(speedB, minspeedb, maxspeedb);
    digitalWrite(MOTOR_L_1, LOW);
    ledcWrite(ENBChanel, speedB);
  } else{
    speedB = constrain(speedB, -maxspeedb, -minspeedb);
    digitalWrite(MOTOR_L_1, HIGH);
    ledcWrite(ENBChanel, 255 + speedB);
  }
}

void motor_control() {
  if (error < 0)
    set_motor(maxspeeda , maxspeedb + PIDValue);
  else 
    set_motor(maxspeeda - PIDValue, maxspeedb );
}


