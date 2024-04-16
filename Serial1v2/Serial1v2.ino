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

  // TODO: get current number
}

void loop()
{
  while(Serial1.available()){
    buff = Serial1.readString();
    Serial.println(buff);
    Serial2.println(buff);
  }
  while(Serial.available())  {
    buff = Serial.readString();
    buff.trim();
    if(buff == "s")
      send_sms();
    else if(buff== "c")
      make_call();
    else
      Serial1.println(buff);
      Serial2.println(buff);
  }

  while(Serial2.available())  {
    buff = Serial2.readString();
    buff.trim();
    if(buff == "s")
      send_sms();
    else if(buff== "c")
      make_call();
    else
      Serial1.println(buff);
  }
}

void send_sms(){
  Serial1.print("AT+CMGS=\"+923001234567\"\r");
  waitForResponse();
  
  Serial1.print("Hello from Serial1");
  Serial1.write(0x1A);
  waitForResponse();
}


void make_call(){
  Serial1.println("ATD+923001234567;");
  waitForResponse();
}

void waitForResponse(){
  delay(1000);
  while(Serial1.available()){
    Serial.println(Serial1.readString());
  }
  Serial1.read();
}



/* *************************************************
 * SMS SENT STATUS
 * SMS SENT     -> OK
 * SMS NOT SENT -> ERROR or +CMS ERROR
 * *************************************************
AT+CMGS="+85291234567"
> It is easy to send text messages.
+CMGS: 12

OK


The value in the information response, 12, is the message reference number allocated to the SMS 
text message. The final result code OK indicates the sending of the SMS text message was successful. 
If the execution of the +CMGS AT command fails, the GSM/GPRS modem or mobile phone will return 
either the final result code ERROR or +CMS ERROR.
 */