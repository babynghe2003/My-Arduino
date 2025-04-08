

/*********************************************************/

/* WAV HEADER STRUCTURE */
struct wavStruct {
  char chunkID[4] = {'R', 'I', 'F', 'F'};
/// 36 + SubChunk2Size
uint32_t chunkSize = 36; // You Don't know this until you write your data but at a minimum it is 36 for an empty file
/// "should be characters "WAVE"
char format[4] = {'W', 'A', 'V', 'E'};
/// " This should be the letters "fmt ", note the space character
char subChunk1ID[4] = {'f', 'm', 't', ' '};
///: For PCM == 16, since audioFormat == uint16_t
uint32_t subChunk1Size = 16;
///: For PCM this is 1, other values indicate compression
uint16_t audioFormat = 1;
///: Mono = 1, Stereo = 2, etc.
uint16_t numChannels = 1;
///: Sample Rate of file
uint32_t sampleRate = 44100;
///: SampleRate * NumChannels * BitsPerSample/8
uint32_t byteRate = 44100 * 2;
///: The number of byte for one frame NumChannels * BitsPerSample/8
uint16_t blockAlign = 2;
///: 8 bits = 8, 16 bits = 16
uint16_t bitsPerSample = 16;
///: Contains the letters "data"
char subChunk2ID[4] = {'d', 'a', 't', 'a'};
///: == NumSamples * NumChannels * BitsPerSample/8  i.e. number of byte in the data.
uint32_t subChunk2Size = 0; // You Don't know this until you write your data

  //Data                                       //The audio data
};

/*********************************************************/
uint32_t counter = 0;
/*********************************************************/

void ADC_Handler(void) {                                   //ADC Interrupt triggered by ADC sampling completion
  aaAudio.getADC(MAX_BUFFER_SIZE);
  if (recFile) {
    #if defined (ESP32)
      for(int i=0; i<MAX_BUFFER_SIZE;i++){
        aaAudio.adcBuffer[i] = aaAudio.adcBuffer16[i]>>4;
      }
    #endif
    recFile.write(aaAudio.adcBuffer, MAX_BUFFER_SIZE);     //Write the data to SD as it is available
    counter++;

  }
}

/*********************************************************/

void startRecording(const char *fileName, uint32_t sampleRate) {

#if defined (RECORD_DEBUG)
  Serial.print("Start Recording: ");
  Serial.println(fileName);
#endif

  if (recFile) {
    aaAudio.adcInterrupts(false);
    recFile.close();
  }
  if (myFile) {                                   //Close any open playback files & disable the DAC
    aaAudio.disableDAC();
    myFile.close();
  }
  recFile = SD.open(fileName, FILE_WRITE);        //Open the file for writing

  if (!recFile) {
#if defined (RECORD_DEBUG)
    Serial.println("Failed to open file");
#endif
    return;
  }
  recFile.seek(0);                                //Write a blank WAV header
  uint8_t bb = 0;
  for (int i = 0; i < 44; i++) {
    recFile.write(bb);
  }

  aaAudio.adcBitsPerSample = 8;                   //Configure AAAudio
  aaAudio.setSampleRate(sampleRate);

  aaAudio.getADC();
  aaAudio.getADC();
  aaAudio.adcInterrupts(true);


}

/*********************************************************/

void createWavHeader(const char *fileName, uint32_t sampleRate ) {

  if (!SD.exists(fileName)) {
#if defined (RECORD_DEBUG)
    Serial.println("File does not exist, please write WAV/PCM data starting at byte 44");
#endif
    return;
  }
  #if !defined (ESP32)
    recFile = SD.open(fileName, FILE_WRITE);
  #endif
  
  if (recFile.size() <= 44) {
#if defined (RECORD_DEBUG)
    Serial.println("File contains no data, exiting");
#endif
    recFile.close();
    return;
  }

  wavStruct wavHeader;
  wavHeader.chunkSize = recFile.size() - 8;
  //wavHeader.numChannels = numChannels;
  wavHeader.sampleRate = sampleRate;
  wavHeader.byteRate = sampleRate * wavHeader.numChannels * wavHeader.bitsPerSample / 8;
  wavHeader.blockAlign = wavHeader.numChannels * wavHeader.bitsPerSample / 8;
  //wavHeader.bitsPerSample = bitsPerSample;
  wavHeader.subChunk2Size = recFile.size() - 44;

#if defined (RECORD_DEBUG)
  Serial.print("WAV Header Write ");
#endif

  recFile.seek(0);
  if ( recFile.write((byte*)&wavHeader, 44) > 0) {
#if defined (RECORD_DEBUG)
    Serial.println("OK");
  } else {
    Serial.println("Failed");
#endif
  }
  recFile.close();

}

/*********************************************************/

void stopRecording(const char *fileName, uint32_t sampleRate) {

  aaAudio.adcInterrupts(false);                        //Disable the ADC interrupt
  #if defined (ESP32)
    recFile.flush();
  #else
    recFile.close();                                         //Close the file
  #endif
  createWavHeader(fileName, sampleRate);                   //Add appropriate header info, to make it a valid *.wav file
#if defined (RECORD_DEBUG)
  Serial.println("Recording Stopped");
#endif
}

/*********************************************************/
