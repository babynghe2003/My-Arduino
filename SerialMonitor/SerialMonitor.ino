void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

int value;

void loop() {
  value = random(100000, 999999);
  Serial.print('*');
  Serial.print(value);
  Serial.println('#');
  digitalWrite(LED_BUILTIN, HIGH);  
  delay(100);                      
  digitalWrite(LED_BUILTIN, LOW);  
  delay(100);                      
}