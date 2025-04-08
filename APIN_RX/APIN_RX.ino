
#define BLYNK_PRINT Serial

/* Fill in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID           "TMPL6kjG3Nn6Z"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "Bq1gD5_LuB05NxhjTAevk_XIMZmjjC_7"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>


char ssid[] = "Phi HM WF6";
char pass[] = "432143210";

void setup()
{
  // Debug console
  Serial.begin(9600);
  Serial.println("Hello");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop()
{
  Blynk.run();
}

