 
#include <SoftwareSerial.h>

#define MOTOR_L_1 7
#define MOTOR_L_2 6
#define MOTOR_R_1 5
#define MOTOR_R_2 4

SoftwareSerial BTserial(10, 11); // RX | TX
 
const long baudRate = 9600; 
char c=' ';
boolean NL = true;
 

uint16_t maxspeed = 255;
String message = "";


void setup() 
{

  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);

  digitalWrite(MOTOR_L_1, HIGH);
  digitalWrite(MOTOR_L_2, HIGH);
  digitalWrite(MOTOR_R_1, HIGH);
  digitalWrite(MOTOR_R_2, HIGH);

    Serial.begin(9600);
    Serial.print("Sketch:   ");   Serial.println(__FILE__);
    Serial.print("Uploaded: ");   Serial.println(__DATE__);
    Serial.println(" ");
 
    BTserial.begin(baudRate);  
    Serial.print("BTserial started at "); Serial.println(baudRate);
    Serial.println(" ");
}
 
void loop()
{
set_motor(255,255);

 
}
void set_motor(int speedB, int speedA) {
  // speedA = -speedA;
  if (speedA > 0){
    speedA = constrain(speedA, 0, maxspeed);
    digitalWrite(MOTOR_R_2, LOW);
    analogWrite(MOTOR_R_1, speedA);
  } else{
    speedA = constrain(speedA, -maxspeed, 0);
    digitalWrite(MOTOR_R_2, HIGH);
    analogWrite(MOTOR_R_1,255 + speedA);
  }
  // speedB = -speedB;
  if (speedB > 0){
    speedB = constrain(speedB, 0, maxspeed);
    digitalWrite(MOTOR_L_1, LOW);
    analogWrite(MOTOR_L_2, speedB);
  } else{
    speedB = constrain(speedB, -maxspeed, -0);
    digitalWrite(MOTOR_L_1, HIGH);
    analogWrite(MOTOR_L_2, 255 + speedB);
  }
}