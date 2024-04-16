void setup() {
  pinMode(14, INPUT);
  Serial.begin(115200);

}

void loop() {
  Serial.println(analogRead(14)-2048);

}
