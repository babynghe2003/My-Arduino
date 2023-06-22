

const uint8_t SENSORS_PIN[] = {  18, 5, 17, 16, 4, 2, 15 };

void setup() {
  for (int pin : SENSORS_PIN){
    pinMode(pin, INPUT);
  }
  Serial.begin(115200);
}

void loop() {
  for (int pin : SENSORS_PIN){
    Serial.print(digitalRead(pin));
    Serial.print(' ');
  }
  Serial.println();
  delay(250);
}
