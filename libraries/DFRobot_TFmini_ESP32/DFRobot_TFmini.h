#ifndef __DFRobot_TFmini_H__
#define __DFRobot_TFmini_H__

#include<Arduino.h>
#include<HardwareSerial.h>

class DFRobot_TFmini
{
  public:
    DFRobot_TFmini();
    void begin();
    bool measure(void);
    bool measure2(void);
    int getDistance(void);
    int getStrength(void);
    int getTemprature(void);
  private:
    HardwareSerial Serial2;
    int distance = 0;
    int strength = 0;
    int temprature = 0;
};

#endif
