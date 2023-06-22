#define CLOCK 12
#define DATA 13
void setup() {
  Serial.begin(115200);
  pinMode(CLOCK, INPUT);
  pinMode(DATA, INPUT);

}

void loop() {
  
  int scanval;

  for (int i = 0; i < 11; i ++){
    while(digitalRead(CLOCK)){
      scanval |= digitalRead(DATA) << i;
      while(!digitalRead(DATA));
    }
  }
  scanval >>= 1;
  scanval &=0xFF;
  Serial.println(scanval, HEX);
}
