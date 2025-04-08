#include <pwmWrite.h>

Pwm pwm = Pwm();

const int servoPin4 = 4;
const int servoPin5 = 5;

void setup() {
}

void loop() {
  int position;  // position in degrees
  for (position = 0; position <= 180; position += 1) {
    pwm.writeServo(servoPin4, position);
    pwm.writeServo(servoPin5, 180 - position);
    delay(15);
  }
  pwm.writeServo(servoPin5, 90);
  delay(2000);
}