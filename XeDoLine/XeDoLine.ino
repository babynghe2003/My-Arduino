

// ####### LƯU Ý ######
// KHÔNG NẠP CODE VÀO LẠI XE, CODE NÀY CHƯA HIỆU CHỈNH CHO TỪNG XE 
// HỌC CÁC LỆNH DỰA THEO CÁC COMMENT BÊN DƯỚI 
// KHI SHOW CODE THÌ XÓA HẾT COMMENT


// Khai báo các chân giao tiếp với mạch điều khiển động cơ
#define MOTOR_L_1 7
#define MOTOR_L_2 6
#define MOTOR_R_1 5
#define MOTOR_R_2 4

// Khai báo các chân giao tiếp với sensor
const uint8_t SENSORS_PIN[] = { A0, A1, A2, A3, A4 };

const uint8_t maxspeed = 180;
const uint8_t minspeed = 10;

const int HeSoLech = 13;
int GiaTriLech = 0;

void setup() { // Hàm này chỉ chạy 1 lần khi cấp nguồn hoặc khi nhấn reset
  // Khởi tạo OUTPUT cho các chân motor
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);

  // Cho xe dừng bằng cách cho các chân in1, in2, in3, in4 LOW hết
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, LOW);

  // Khởi tạo INPUT cho các chân sensor
  for (auto sensorPin : SENSORS_PIN) {
    pinMode(sensorPin, INPUT);
  }
  Serial.begin(115200);
  delay(100);
}

bool isRunning = 1;
// Hàm này lặp lại liên tục
void loop() {

  while (isRunning) {
    read_sensors(); // Đọc giá trị sensor
    motor_control(); // Điều khiển động cơ
    delay(10);
  }
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, LOW);
  while (1){ // Khi gặp vạch đích, cho xe dừng và tạo lặp vô hạn tránh xe chạy lại
    
  }

}

void read_sensors() {
  // Đọc tất cả sensor và nối vào 1 chuỗi để dễ so sánh
  String sensorArray = "";
  for (auto sensorPin : SENSORS_PIN) {
    sensorArray += (char)(digitalRead(sensorPin) + 48); // + 48 để chuyển số thành kí tự số trong bảng mã ASCII
  }

  // Tùy vào từng trường hợp để set sensor, với 1 là cảm biến ở trong line, 0 là ngoài line
  // Line nằm bên phải thì error > 0 và ngược lại, line nằm ở giữa thì error = 0
  if (sensorArray == "00001") error = 5;
  else if (sensorArray == "00111") error = 4;
  else if (sensorArray == "00011") error = 3;
  else if (sensorArray == "00010") error = 2;
  else if (sensorArray == "00110") error = 1;
  else if (sensorArray == "00100") error = 0;
  else if (sensorArray == "01110") error = 0;
  else if (sensorArray == "01100") error = -1;
  else if (sensorArray == "01000") error = -2;
  else if (sensorArray == "11000") error = -3;
  else if (sensorArray == "11100") error = -4;
  else if (sensorArray == "10000") error = -5;
  else if (sensorArray == "00000"){ 
// Nếu không có cảm biến nào nhận diện được line, dừng kết quả error trước đó để quyết định error hiện tại
    if (error > 0) error = 6;
    else error = -6;
  }
   else if (sensorArray == "11111") { // Nếu tất cả cảm biến trong line tức là vạch dừng
    isRunning = 0;
  }

  // Nếu ngược hướng ( line nằm bên phải mà xe rẽ trái ) thì đổi dấu error
  // error = -error; 
}


void set_motor(int speedA, int speedB) {
  speedA = -speedA;
  if (speedA > 0){
    speedA = constrain(speedA, minspeed, maxspeed);
    digitalWrite(MOTOR_R_2, LOW);
    analogWrite(MOTOR_R_1, speedA); // HDT : speedA - 0 = speedA
  } else{
    speedA = constrain(speedA, -maxspeed, -minspeed);
    digitalWrite(MOTOR_R_2, HIGH); // HDT: 255 - (255 + speedA)= - speedA
    analogWrite(MOTOR_R_1,255 + speedA);
  }
  // speedB = -speedB;
  if (speedB > 0){
    speedB = constrain(speedB, minspeed, maxspeed);
    digitalWrite(MOTOR_L_1, LOW);
    analogWrite(MOTOR_L_2, speedB);
  } else{
    speedB = constrain(speedB, -maxspeed, -minspeed);
    digitalWrite(MOTOR_L_1, HIGH);
    analogWrite(MOTOR_L_2, 255 + speedB);
  }
}

void motor_control() {
  // Tính giá trị lệch bằng cách lấy hệ số lệch * độ lệch
  GiaTriLech = HeSoLech * error; 
  // Nếu error nhỏ hơn 0, tức là line lệch bên trái, thì cho xe rẽ trái bằng cách cho bánh trái chạy maxspeed, 
  // bánh phải giảm tốc độ lại, ngược lại tương tự
  if (error < 0) 
    set_motor(maxspeed , maxspeed + GiaTriLech);
  else 
    set_motor(maxspeed - GiaTriLech, maxspeed );
}
// UART

