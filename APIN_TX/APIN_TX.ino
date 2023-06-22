#include<nRF24L01.h>
#include<RF24.h>
#include <DFRobot_TFmini.h>

RF24 radio(4,5);
DFRobot_TFmini  TFmini;

struct sensorData {
  String id;
  int distance, strength, temprature;
};

sensorData data;
int i;

void setup(){

  delay(1000);
  Serial.begin(115200);
  Serial.println("Setup");
  TFmini.begin();
  radio.begin();
  radio.setChannel(5);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(0x1234567890LL);
  data.id = "1";
  data.distance = 0;
  data.strength = 0;
  data.temprature = 0;
}

void loop(){
  if(TFmini.measure2()){
    data.distance = TFmini.getDistance();       
    data.strength = TFmini.getStrength();    
    data.temprature = TFmini.getTemprature();
    radio.write(&data, sizeof(sensorData));
    delay(500);
  }
}