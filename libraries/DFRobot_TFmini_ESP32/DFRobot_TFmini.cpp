#include <DFRobot_TFmini.h>

DFRobot_TFmini::DFRobot_TFmini()
  : Serial2(2)  // Initialize Serial2 using constructor initializer list
{
}
void  DFRobot_TFmini::begin()
{

  Serial2.begin(115200);
}

bool  DFRobot_TFmini::measure(void)
{
  int TFbuff[9] = {0};
  long checksum  = 0 ;
  while(Serial2.available()){
    TFbuff[0] = Serial2.read();
    checksum += TFbuff[0];
    if(TFbuff[0] == 'Y'){
      TFbuff[1] = Serial2.read();
      checksum += TFbuff[1];
      if(TFbuff[1] == 'Y'){
        for(int i = 2;i < 8;i++){
          TFbuff[i] = Serial2.read();
          checksum += TFbuff[i];
        }
        TFbuff[8] = Serial2.read();
        checksum &= 0xff;
        if(checksum == TFbuff[8]){
          distance = TFbuff[2]+TFbuff[3]*256;
          strength = TFbuff[4]+TFbuff[5]*256;
          return true;
        }else{
          checksum  = 0;
        }
      }else{
        checksum  = 0;
      }
    }else{
      checksum  = 0;
    }
  }
  return false;
}

bool DFRobot_TFmini::measure2(void){
  unsigned char uart[9];
  char check;
  int rec_debug_state = 0x01;
  while(true){
    if (Serial2.available()) //check if serial port has data input
    {
      if (rec_debug_state == 0x01)
      { //the first byte
        uart[0] = Serial2.read();
        if (uart[0] == 0x59)
        {
          check = uart[0];
          rec_debug_state = 0x02;
        }
      }
      else if (rec_debug_state == 0x02)
      { //the second byte
        uart[1] = Serial2.read();
        if (uart[1] == 0x59)
        {
          check += uart[1];
          rec_debug_state = 0x03;
        }
        else {
          rec_debug_state = 0x01;
        }
      }
      else if (rec_debug_state == 0x03)
      {
        uart[2] = Serial2.read();
        check += uart[2];
        rec_debug_state = 0x04;
      }
      else if (rec_debug_state == 0x04)
      {
        uart[3] = Serial2.read();
        check += uart[3];
        rec_debug_state = 0x05;
      }
      else if (rec_debug_state == 0x05)
      {
        uart[4] = Serial2.read();
        check += uart[4];
        rec_debug_state = 0x06;
      }
      else if (rec_debug_state == 0x06)
      {
        uart[5] = Serial2.read();
        check += uart[5];
        rec_debug_state = 0x07;
      }
      else if (rec_debug_state == 0x07)
      {
        uart[6] = Serial2.read();
        check += uart[6];
        rec_debug_state = 0x08;
      }
      else if (rec_debug_state == 0x08)
      {
        uart[7] = Serial2.read();
        check += uart[7];
        rec_debug_state = 0x09;
      }
      else if (rec_debug_state == 0x09)
      {
        uart[8] = Serial2.read();
        if (uart[8] == check)
        {
          distance = uart[2] + uart[3] * 256; //the distance
          strength = uart[4] + uart[5] * 256; //the strength
          temprature = uart[6] + uart[7] * 256; //calculate chip temprature
          temprature = temprature / 8 - 256;
          while (Serial2.available()) {
            Serial2.read(); // This part is added becuase some previous packets are
          }
          return true;
        }
        rec_debug_state = 0x01;
      }
    }  
  }
  return false;
}

int DFRobot_TFmini::getDistance(void)
{
  return distance;
}

int DFRobot_TFmini::getStrength(void)
{
  return strength;
}

int DFRobot_TFmini::getTemprature(void){
  return temprature;
}
