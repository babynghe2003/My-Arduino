w/*
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
#include <driver/adc.h>

File audioFile;
File logFile;
Servo myservo;

const char * path = "/dataset/audio5.wav";
const char * logpath = "/log/log.txt";

int microphonePin = 14;
int servoPin = 27;

const int sampleRate = 44100;  // Sample rate in Hz (CD quality)
const int bitsPerSample = 16;  // Bits per sample (16-bit)
const int numChannels = 1;     // Mono audio
const int bufferSize = 1024;   // Audio buffer size (adjust as needed)

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
    // myservo.attach(servoPin);
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        while(1);
        return;
    }

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);

    if (!SD.exists("/dataset")) {
        SD.mkdir("/dataset");
    }

    if (!SD.exists("/log")) {
        SD.mkdir("/log");
    }

    // Check if /mydir/hello.txt exists
    audioFile = SD.open(path, FILE_WRITE);
    writeWavHeader();
   
    if (!SD.exists(logpath)) {
        logFile = SD.open(logpath, FILE_WRITE);
    }else {
      logFile = SD.open(logpath, FILE_APPEND);
    }
    // audioFile.print("AAAAAAAA\n");
    
    // for (pos = 120; pos >= 60; pos -= 1) { 
    //   myservo.write(pos);    
    //   delay(15);             
    // }
    delay(1000);
    millisLast = millis();
    
}

void loop(){
  millisCurrent = millis();

  int bytesRead = analogRead(microphonePin);
  int16_t scaledSample = map(bytesRead, 0, 4096,-32767,32767);
  audioFile.write((uint8_t*)&scaledSample, sizeof(int16_t));
  Serial.println(scaledSample);
  // if (millisCurrent - millisLast > 400) myservo.write(120);  
  if (millisCurrent - millisLast > 20000){
    Serial.print("Recording completed. Elapsed time: ");
    Serial.print(millisCurrent - millisLast);
    Serial.println(" milliseconds");
    audioFile.close();
    logFile.close();
    digitalWrite(LED_BUILTIN, LOW);
    while (1);
  }
  // Serial.println(scaledSample);
  
}

void writeWavHeader() {
  char header[44];
  int totalDataLen = 0;
  int audioDataLen = 0;

  audioDataLen = 0; // Update this with the actual data length

  totalDataLen = audioDataLen + 36;

  // WAV header
  strncpy(header, "RIFF", 4);
  memcpy(header + 4, &totalDataLen, 4);
  strncpy(header + 8, "WAVEfmt ", 8);
  int subChunk1Size = 16;
  memcpy(header + 16, &subChunk1Size, 4);
  int audioFormat = 1;  // PCM format
  memcpy(header + 20, &audioFormat, 2);
  memcpy(header + 22, &numChannels, 2);
  memcpy(header + 24, &sampleRate, 4);
  int byteRate = sampleRate * numChannels * (bitsPerSample / 8);
  memcpy(header + 28, &byteRate, 4);
  int blockAlign = numChannels * (bitsPerSample / 8);
  memcpy(header + 32, &blockAlign, 2);
  memcpy(header + 34, &bitsPerSample, 2);
  strncpy(header + 36, "data", 4);
  memcpy(header + 40, &audioDataLen, 4);

  audioFile.write((const uint8_t*)header, 44);
}

void writeAudioSample(int sample) {
  byte lowByte = sample & 0xFF;
  byte highByte = (sample >> 8) & 0xFF;

  audioFile.write(lowByte);
  audioFile.write(highByte);
}
