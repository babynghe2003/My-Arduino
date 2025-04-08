#include<nRF24L01.h>
#include<RF24.h>
#include <ArduinoJson.h>

RF24 radio(4,5);

struct sensorData {
  String id;
  int distance, strength, temprature;
};

sensorData data;


void setup(){
  delay(1000);
  Serial.begin(115200);
  Serial.println("Setup");
  radio.begin();
  radio.setChannel(5);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1, 0x1234567890LL);
  radio.startListening();
}

void loop(){
  if (radio.available()){
    radio.read(&data, sizeof(sensorData));
    Serial.println(data.distance);
    delay(200);
  }
}