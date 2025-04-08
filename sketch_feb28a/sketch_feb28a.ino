#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>

const char* ssid = "Minh_A12A10";     // Tên mạng WiFi của bạn
const char* password = "0352353641";  // Mật khẩu mạng WiFi của bạn

Servo myservo;  // Khai báo đối tượng Servo
int pos = 0;    // Vị trí của Servo

AsyncWebServer server(80);  // Khai báo đối tượng HTTP Server

void setup() {
  Serial.begin(115200);

  // Kết nối ESP32 với WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  // Cấu hình HTTP Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    String html = "<html><head><title>ESP32 Servo Control</title></head><body>";
    html += "<h1>ESP32 Servo Control</h1>";
    html += "<p>Position: " + String(pos) + "</p>";
    html += "<form action=\"/update\" method=\"get\">";
    html += "<label for=\"pos\">Position (0-180): </label>";
    html += "<input type=\"text\" name=\"pos\">";
    html += "<input type=\"submit\" value=\"Update\">";
    html += "</form></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/update", HTTP_GET, [](AsyncWebServerRequest* request) {
    String strPos = request->getParam("pos")->value();
    int newPos = strPos.toInt();
    if (newPos >= 0 && newPos <= 180) {
      pos = newPos;
      myservo.write(pos);
    }
    request->redirect("/");
  });

  // Khởi tạo Servo
  myservo.attach(2);
  server.begin();
}
void loop() {
  // Xử lý các kết nối HTTP

  // Chạy Servo
  if (pos >= 0 && pos <= 180) {
    myservo.write(pos);
    delay(15);
  }
}