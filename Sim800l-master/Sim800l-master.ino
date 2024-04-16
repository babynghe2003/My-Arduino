String currentNumber = "";

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200,SERIAL_8N1,2,4); // sim
  Serial2.begin(115200,SERIAL_8N1,16,17); // rs485

  Serial.println("Initializing...");
  
  Serial1.println("AT");
  waitForResponse();

  Serial1.println("ATE1");
  waitForResponse();

  Serial1.println("AT+CMGF=1");
  waitForResponse();

  Serial1.println("AT+CNMI=1,2,0,0,0");
  waitForResponse();

  getCurrentNumber();

}

void loop()
{
  while(Serial1.available()){
    buff = Serial1.readString();
    Serial.println(buff);
    Serial2.println(buff); // send sms to other
  }
  while(Serial.available())  { // read command from computer
    buff = Serial.readString();
    buff.trim();
    if (buff.length() > 2 && buff[0]+buff[1] == "AT"){
      Serial1.println(buff);
      Serial2.println(buff); // send to other
    }
  }

  while(Serial2.available())  { 
    buff = Serial2.readString();
    buff.trim();
    if (buff.length() > 1 && buff[0]+buff[1] == "AT"){ // read command from other module AT command
      Serial1.println(buff);
    }else{ // read from other SMS income
      Serial.print(buff)
    }
  }
}


void waitForResponse(){
  delay(500);
  while(Serial1.available()){
    Serial.println(Serial1.readString());
  }
  Serial1.read();
}

void waitForResponseAndSendToOther(){
  delay(500);
  while(Serial1.available()){
    buff = Serial1.readString()
    buff.trim();
    Serial.println(buff);
    Serial2.println(buff);
  }
  Serial1.read();
}

void getCurrentNumber(){

  Serial1.println("AT+CNUM");
  delay(500);
  while(Serial1.available()){
    Serial.println(Serial1.readString());
  }
  Serial1.read();

}

