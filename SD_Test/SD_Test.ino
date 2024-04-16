/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO
 *    D1       -
 */
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <ESP32Servo.h>

File audioFile;
File logFile;
Servo myservo;

const char * path = "/dataset/audio4.wav";
const char * logpath = "/log/log.txt";

int microphonePin = 14;
int servoPin = 27;

const int sampleRate = 44100;  // Sample rate in Hz (CD quality)
const int bitsPerSample = 16;  // Bits per sample (16-bit)
const int numChannels = 1;     // Mono audio

unsigned long millisCurrent = 0;
unsigned long millisElapsed = 0;
unsigned long millisLast = 0;

int pos = 130;

struct wavStruct {
    const char chunkID[4] = {'R', 'I', 'F', 'F'};
    uint32_t chunkSize = 36;                     //Size of (entire file in bytes - 8 bytes) or (data size + 36)
    const char format[4] = {'W', 'A', 'V', 'E'};
    const char subchunkID[4] = {'f', 'm', 't', ' '};
    const uint32_t subchunkSize = 16;
    const uint16_t audioFormat = 1;              //PCM == 1
    uint16_t numChannels = 1;                    //1=Mono, 2=Stereo
    uint32_t sampleRate = 11000;
    uint32_t byteRate = 11000;                   //== SampleRate * NumChannels * BitsPerSample/8
    uint16_t blockAlign = 1;                     //== NumChannels * BitsPerSample/8
    uint16_t bitsPerSample = 8;                  //8,16,32...
    const char subChunk2ID[4] = {'d', 'a', 't', 'a'};
    uint32_t subChunk2Size = 0;                  //== NumSamples * NumChannels * BitsPerSample/8
  //Data                                       //The audio data
};


void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    digitalWrite(LED_BUILTIN, HIGH);
    myservo.attach(servoPin);
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        while(1);
        return;
    }

    if (!SD.exists("/dataset")) {
        SD.mkdir("/dataset");
    }

    if (!SD.exists("/log")) {
        SD.mkdir("/log");
    }

    // Check if /mydir/hello.txt exists
    if (!SD.exists(path)) {
        audioFile = SD.open(path, FILE_WRITE);
        writeWavHeader();
    }else {
      audioFile = SD.open(path, FILE_APPEND);
    }

    if (!SD.exists(logpath)) {
        logFile = SD.open(logpath, FILE_WRITE);
    }else {
      logFile = SD.open(logpath, FILE_APPEND);
    }
    // audioFile.print("AAAAAAAA\n");
    digitalWrite(LED_BUILTIN, LOW);
    for (pos = 120; pos >= 60; pos -= 1) { 
      myservo.write(pos);    
      delay(15);             
    }
    delay(1000);
    millisLast = millis();
}

void loop(){
  millisCurrent = millis();

  int audioSample = analogRead(14) - 2048;
  writeAudioSample(audioSample);
  if (millisCurrent - millisLast > 400) myservo.write(120);  
  if (millisCurrent - millisLast > 2000){
    audioFile.close();
    logFile.close();

    while (1);
  }
  
}

void writeWavHeader(){
  wavStruct wavHeader;
  audioFile.write((byte*)&wavHeader, 44);
}

void writeAudioSample(int sample) {
  byte lowByte = sample & 0xFF;
  byte highByte = (sample >> 8) & 0xFF;

  audioFile.write(lowByte);
  audioFile.write(highByte);
}
