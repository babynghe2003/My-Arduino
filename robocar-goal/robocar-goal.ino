#include<Servo.h>

Servo goal;
int sensor = A5;
int ledPin = 13;
long lastTime = -5001;

void setup()
{
    pinMode(sensor, INPUT);
    pinMode(ledPin, OUTPUT);
    goal.attach(3);
    digitalWrite(ledPin, HIGH); // tắt đèn led
    goal.write(135);
    Serial.begin(9600);
}

void loop()
{
  Serial.print(millis());
  Serial.print(' ');
  Serial.println(analogRead(A0));
  if (analogRead(A0) < 35){
    lastTime = millis();
  }
  if (millis() - lastTime < 5000){
    goal.write(50);
    digitalWrite(ledPin, HIGH);
    // delay(1000000);
    // goal.write(105);
  }else{
    digitalWrite(ledPin, LOW);
    goal.write(135);
  }
}
