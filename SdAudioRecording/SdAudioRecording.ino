
/******** User Config ************************************/

#define SD_CS_PIN 5
#define AUDIO_DEBUG
#define RECORD_DEBUG

const char newWavFile[] = "/test.wav";

/*********************************************************/
#include <SPI.h>
#include <SD.h>
#include <AutoAnalogAudio.h>
/*********************************************************/
AutoAnalog aaAudio;
File myFile;
File recFile;
/*********************************************************/
#include "myWAV.h"
#include "myRecording.h"
/*********************************************************/

void setup() {

  Serial.begin(115200);

  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  Serial.println("SD ok\nAnalog Audio Begin");

  aaAudio.begin(1, 1);     // Start AAAudio with ADC & DAC
  aaAudio.autoAdjust = 0;  // Disable automatic timer adjustment

}

/*********************************************************/
uint32_t displayTimer = 0;
bool recording = false;

void loop() {

  if (millis() - displayTimer > 1000) {
    displayTimer = millis();
    if (counter) {
      Serial.print("Samples per Second: ");
      Serial.println(counter * MAX_BUFFER_SIZE);
    }
    counter = 0;
  }

  if (Serial.available()) {
    char input = Serial.read();
    switch (input) {

      case '1':  playAudio("/M8b24kM.wav");  break; //Play a *.wav file by name - 8bit, 24khz, Mono
      case '2':  playAudio("/M8b24kS.wav");  break; //Play  8bit, 24khz, Stereo
      case '3':  playAudio("/M16b24kS.wav"); break; //Play 16bit, 24khz, Stereo
      case '4':  playAudio("/M8b44kST.wav"); break; //Play  8bit, 44khz, Stereo
      case '5':  channelSelection = 0;      break; //Play the audio on DAC0
      case '6':  channelSelection = 1;      break; //Play the audio on DAC1
      case '7':  channelSelection = 2;      break; //Play the audio on DAC0 & DAC1
      case '8':  Serial.println("OK");      break;
      case '9':  startRecording(newWavFile, 24000); recording = true; break; //Start recording @11khz,8-bit,Mono
      case '0':  stopRecording(newWavFile, 24000);  recording = false; break; //Stop the recording and finalize the file
      case 'p':  playAudio(newWavFile);      break; //Play back the recorded audio
      case 'D':  SD.remove(newWavFile);      break; //Delete the file and start fresh
    }
  }

#if defined (ESP32)
  if(recording){
    ADC_Handler();
  }
#endif

}

/*********************************************************/
