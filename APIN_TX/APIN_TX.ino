#define BLYNK_PRINT Serial

/* Fill in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL69Tnoh0z8"
#define BLYNK_TEMPLATE_NAME "FoodTank"
#define BLYNK_AUTH_TOKEN "EezAsvMDgnUc3HN1edAsjTIpmUfpAYm4"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DFRobot_TFmini.h>

DFRobot_TFmini  TFmini;
BlynkTimer timer;
char ssid[] = "Phi HM WF6";
char pass[] = "432143210";
int distance = 0; 

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  Blynk.syncAll();
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  if (TFmini.measure2()){
    distance = TFmini.getDistance();
    Serial.print("Distance: ");
    Serial.println(distance);
    Blynk.virtualWrite(V0, distance); // Send distance value to Blynk V0
  }
  else{
    Serial.println("Failed to read from TFmini");
  }
}

void setup()
{
  // Debug console
  Serial.begin(115200);
  Serial.println("Hello w");
  
  // Attempt to connect to WiFi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  
  TFmini.begin();
  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
}

void loop()
{
  Blynk.run();
  timer.run(); // Add this line to ensure the timer events are processed
}