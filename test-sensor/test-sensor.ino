#include <Servo.h>

Servo myServo;

void setup(){

  myServo.attach(A0);
}

void loop(){

  myServo.write(90);
}