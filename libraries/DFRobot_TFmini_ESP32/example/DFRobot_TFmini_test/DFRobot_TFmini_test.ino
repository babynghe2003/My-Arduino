 /*
  * @File  : DFRobot_TFmini_test.ino
  * @Brief : This example use TFmini to measure distance
  *         With initialization completed, we can get distance value,signal strength and temprature
  * Author: BabyNghe2003
  * Link repo: https://github.com/babynghe2003/DFRobot_TFmini_ESP32.git
  * @version  V1.0
  */
  
#include <DFRobot_TFmini.h>

DFRobot_TFmini  TFmini;
uint16_t distance,strength, temprature;
// Default Serial2 using GPIO 16 and GPIO 17 to comunitcate UART2

void setup(){
    Serial.begin(115200);
    TFmini.begin();
}

void loop(){
    if(TFmini.measure2()){                     
        distance = TFmini.getDistance();       
        strength = TFmini.getStrength();    
        temprature = TFmini.getTemprature();
        Serial.print("Distance = ");
        Serial.print(distance);
        Serial.print("cm  ,");
        Serial.print("Strength = ");
        Serial.print(strength);
        Serial.print("  ,Temprature = ");
        Serial.println(temprature);
        delay(500);
    }
}
