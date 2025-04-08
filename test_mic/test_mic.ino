#include <SPI.h>
#include <SD.h>


#define MIC_PIN 14           // Analog input pin for MAX4466
#define SD_CS_PIN 5          // CS pin for SD card reader
#define RECORDING_TIME_SEC 10 // Recording time in seconds
const int SAMPLE_RATE =  44100;    // Sample rate in Hz

File audioFile;
bool recording = false;

unsigned long currentTime = 0;
unsigned long lastTime = 0;
unsigned long step = 1000000/SAMPLE_RATE;
unsigned long size = 0;

void setup() {
  Serial.begin(2000000);
  while (!Serial) {
    delay(10);
  }

  if (!SD.begin()) {
    Serial.println("SD card initialization failed.");
    return;
  }
  audioFile = SD.open("/test_wav.txt", FILE_WRITE);
  Serial.println("SD card initialized.");

  
}

void loop() {
  // currentTime = micros();
  // if (currentTime - lastTime > step){
  //   Serial.println(size);
  //   int bytesRead = analogRead(14);
  //   int16_t scaledSample = map(bytesRead, 0, 4096,-32767,32767);
  //   audioFile.print(scaledSample);
  //   if (++size > 127782){
  //     Serial.print("Recording complete");
  //     audioFile.close();
  //     while(1);
  //   }
  // }
  Serial.println(++size);
}

