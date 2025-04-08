
unsigned long millisLast = 0;
unsigned long millisCurrent = 0;
unsigned long millisElapsed = 0;

int sampleBuffer = 0;

const int SAMPLE_TIME = 10;
void setup() {
  Serial.begin(9600);
}

void loop() {
  // millisCurrent = millis();
  // millisElapsed = millisCurrent - millisLast;

  // if (analogRead(14) == LOW){
  //   sampleBuffer++;
  // }

  // if (millisElapsed > SAMPLE_TIME){
  //   Serial.println(sampleBuffer);
  //   sampleBuffer = 0;
  //   millisLast = millisCurrent;
  // }
  Serial.print("Top:");
  Serial.println(2048);
  Serial.print("Audio:");
  Serial.println(analogRead(14)-2048);
  Serial.print("Bot:");
  Serial.println(-2048);
}
