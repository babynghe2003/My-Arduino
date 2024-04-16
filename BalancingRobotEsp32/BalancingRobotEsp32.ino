#include <ESP32Servo.h>

#include <MPU6050_tockn.h>
#include <Wire.h>

#define LEN 19
#define LDIR 5
#define LSTEP 18
#define REN 4
#define RDIR 15
#define RSTEP 2
#define BTN_STOP_ALARM    0

#define LSERVO 12
#define RSERVO 14

#define KI 0
#define KP 100.0
#define KD 100.0

#define MAX_ACCEL 20
#define ZERO_SPEED 0xffffff


hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;

portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;


// volatile uint32_t counter1 = 0, counter2 = 0;
volatile uint32_t dir_M1 = 1, dir_M2 = 1;
volatile int32_t steps1 = 0;
volatile int32_t steps2 = 0;

int16_t speed_M1 = 0, speed_M2 = 0;

Servo LServo;
Servo RServo;

double P = 0, I = 0, D = 0, PID = 0;
double error = 0.0, lastError;


MPU6050 mpu6050(Wire);
long timermilis = 0;

bool mode1 = true;
bool mode2 = true;


void IRAM_ATTR timer1ISR(){
  portENTER_CRITICAL_ISR(&timerMux1);
  if (dir_M1 != 0){
    if (mode1)
    digitalWrite(LSTEP, HIGH);
    else
    digitalWrite(LSTEP, LOW);
    mode1 = !mode1;
    if (dir_M1 > 0)
			steps1--;
		else
			steps1++;
  }
	portEXIT_CRITICAL_ISR(&timerMux1);
}

void IRAM_ATTR timer2ISR(){
  portENTER_CRITICAL_ISR(&timerMux2);
  if (dir_M2 != 0){
    if (mode2)
    digitalWrite(RSTEP, HIGH);
    else
    digitalWrite(RSTEP, LOW);
    mode2 = !mode2;

    if (dir_M2 > 0)
			steps2--;
		else
			steps2++;

  }
	portEXIT_CRITICAL_ISR(&timerMux2);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEN, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LDIR, OUTPUT);
  pinMode(RDIR, OUTPUT);
  pinMode(LSTEP, OUTPUT);
  pinMode(RSTEP, OUTPUT);
  digitalWrite(LEN, LOW);
  digitalWrite(REN, LOW);

  LServo.attach(LSERVO);
  RServo.attach(RSERVO);

  LServo.write(180-30);
  RServo.write(30);

  pinMode(BTN_STOP_ALARM, INPUT);

  timer1 = timerBegin(0, 40, true);
  timer2 = timerBegin(0, 40, true);

  timerAttachInterrupt(timer1, &timer1ISR, true);
  timerAlarmWrite(timer1, ZERO_SPEED, true);

  timerAttachInterrupt(timer2, &timer2ISR, true);
  timerAlarmWrite(timer2, ZERO_SPEED, true);

  timerAlarmEnable(timer1);
  timerAlarmEnable(timer2);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  setSpeedM1(100);
  setSpeedM2(200);
}

void loop() {
  // If button is pressed
  // if (digitalRead(BTN_STOP_ALARM) == LOW) {
  //   // If timer is still running
  //   if (timer1) {
  //     // Stop and free timer
  //     timerEnd(timer1);
  //     timer1 = NULL;
  //   }
  //   if (timer2) {
  //     // Stop and free timer
  //     timerEnd(timer2);
  //     timer2 = NULL;
  //   }
  // }

  // mpu6050.update();
  // lastError = error;
  // error = mpu6050.getAccY()*10;
  // P = error * KP;
  // I += error * KI;
  // D = (error - lastError)*KD;
  // PID = P + I + D;

  // // PID = constrain(PID,-100,100);

  // setSpeed(PID, PID);

  // if (dir1 == 1){
  //   digitalWrite(LDIR, LOW);
  // }else{
  //   digitalWrite(LDIR, HIGH);
  // }

  // if (dir2 == 1){
  //   digitalWrite(RDIR, HIGH);
  // }else{
  //   digitalWrite(RDIR, LOW);
  // }
  
  Serial.print(steps1);
  Serial.print(" ");
  Serial.print(steps2);
  Serial.print(" ");
  Serial.print(dir_M1);
  Serial.print(" ");
  Serial.println(dir_M2);
  delay(1000);




}

void setSpeedM1(int tspeed){ // speed: 28 -> 1000, true speed: 200 - 5\

  long timer_period;
  int16_t speed;
  Serial.println("set M1");
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;
  
  speed = speed_M2 * 50;

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M1 = 1;
    digitalWrite(LDIR, LOW);
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M1 = -1;
    digitalWrite(LDIR, HIGH);
  }
  if (timer_period > ZERO_SPEED)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  timerAlarmWrite(timer1, timer_period, true);

}

void setSpeedM2(int tspeed){ // speed: 28 -> 1000, true speed: 200 - 5
  long timer_period;
  int16_t speed;
  Serial.println("set M2");


  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;
  
  speed = speed_M2 * 50;

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M2 = 1;
    digitalWrite(RDIR, LOW);
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M2 = -1;
    digitalWrite(RDIR, HIGH);
  }
  if (timer_period > ZERO_SPEED)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  timerAlarmWrite(timer2, timer_period, true);


}
