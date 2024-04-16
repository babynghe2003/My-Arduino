#define RXD2 16
#define TXD2 17

void setup(){

  Serial.begin(9600);

  Serial.println("Goodnight moon!");

  Serial2.begin(9600);
  Serial2.println("Hello world?");

}

void loop(){
  if (Serial2.available()){
    Serial.write(Serial2.read());
  }

  if (Serial.available()){
    Serial2.write(Serial.read());
  }
}