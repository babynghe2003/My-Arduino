

// ####### LƯU Ý ######
// KHÔNG NẠP CODE VÀO LẠI XE, CODE NÀY CHƯA HIỆU CHỈNH CHO TỪNG XE 
// HỌC CÁC LỆNH DỰA THEO CÁC COMMENT BÊN DƯỚI 
// KHI SHOW CODE THÌ XÓA HẾT COMMENT


// Import các thư viện cần thiết
#include <Servo.h>

// Khai báo các chân giao tiếp với mạch điều khiển động cơ
#define MOTOR_L_1 7
#define MOTOR_L_2 6
#define MOTOR_R_1 5
#define MOTOR_R_2 4

// Khai báo các chân giao tiếp với servo
#define arm1_servo A2
#define arm2_servo A3
#define grip_servo A1
#define center_servo A4
#define flag_servo A0

// Khởi tạo các đối tượng để điều khiển servo
Servo arm1;
Servo arm2;
Servo grip;
Servo center;
Servo flag;

// Khai báo các biến để lưu trữ giá trị góc của từng servo
// 2 Servo cánh tay
float arm1_angle = 90;
float arm2_angle = 90;

// Servo gắp
bool is_grip = false;
float grip_angle = 120;
float ungrip_angle = 180;

// Servo trục giữa
float center_angle = 90;
float center_d = 3;

// Servo cờ 
bool is_flag = false;
float flag_angle = 180;
float unflag_angle = 90;

void setup() {
  // Dùng hàm attach để gán từng chân cho từng đối tượng servo
  arm1.attach(arm1_servo);
  arm2.attach(arm2_servo);
  grip.attach(grip_servo);
  center.attach(center_servo);
  flag.attach(flag_servo);

  // Khai báo output cho các chân của motor
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);

  // Bắt đầu chạy Serial để giao tiếp với HC-05 hoặc máy tính
  Serial.begin(9600);

  // Cho các servo về giá trị mặc định bằng lệnh write
  arm1.write(90);
  arm2.write(90);
  grip.write(ungrip_angle);
  center.write(center_angle);

}
// Khởi tạo biến maxspeed
int maxspeed = 255;
String message = "";

void loop() {
  // Đọc tín hiệu từ module HC-05 để nhận lệnh từ điện thoại
  if (Serial.available()) {
    message = Serial.readStringUntil('\n'); // 1 string
    // Serial.println(message);
  if (message == "F") {  // Đi thẳng ( 2 bánh tiến )
    set_motor(maxspeed, maxspeed);
  } else if (message == "S") { // Dừng lại ( 2 bánh dừng )
    set_motor(0, 0);
  } else if (message == "G") { // Chạy lùi ( 2 bánh lùi )
    set_motor(-maxspeed, -maxspeed);
  } else if (message == "Q") { // Tiến tới và rẽ trái  ( Trái dừng, phải Tiến )
    set_motor(0, maxspeed);
  } else if (message == "E") { // Tiến tới và rẽ phải ( Trái Tiến, phải dừng )
    set_motor(maxspeed, 0);
  } else if (message == "L") { // Xoay trái ( Trái lùi, phải Tiến )
    set_motor(-maxspeed, maxspeed);
  } else if (message == "R") { // Xoay phải ( Trái Tiến, Phải lùi)
    set_motor(maxspeed, -maxspeed); 
  } else if (message == "Z") { // Lùi và rẽ trái ( Trái lùi, phải dừng )
    set_motor(-maxspeed, maxspeed);
  } else if (message == "C") {// Lùi và rẽ phải ( Trái dừng, Phải lùi )
    set_motor(maxspeed, -maxspeed);
  } else if (message == "M") { // Toggle tay gắp
    if (is_grip){ // Nếu đang gắp, thì nhả
      grip.write(ungrip_angle);
    }else{ // Nếu nhả thì gắp
      grip.write(grip_angle);
    } // toggle grip
    is_grip = !is_grip; // Đổi trạng thái 
  } else if (message == "N") { // Toggle cờ
    if (is_flag){ // Nếu đang phất, thì hạ
      flag.write(unflag_angle);
    }else{ // Ngược lại thì phấtelse if (message == "X") { // tăng giá trị center
    center_angle = constrain(center_angle + center_d, 0, 180);
    center.write(center_angle);
  } else if (message == "Y") { // giảm giá trị  center
    center_angle = constrain(center_angle - center_d, 0, 180);
    center.write(center_angle);
      flag.write(flag_angle);
    }
    is_flag = !is_flag;
  } else if (message == "X") { // tăng giá trị center
    center_angle = constrain(center_angle + center_d, 0, 180);
    center.write(center_angle);
  } else if (message == "Y") { // giảm giá trị  center
    center_angle = constrain(center_angle - center_d, 0, 180);
    center.write(center_angle);
  } else if (message[0] == 'J'){ // Tín hiệu có dạng Jxxx với xxx là giá trị từ slider, Nếu tín hiệu bắt đầu bằng J thì đọc số xxx và xoay servo
    arm1_angle = message.substring(1).toInt(); // "J123".substring(1) -> "123", "123".toInt() -> 123 -> Servo quay về góc 123
    arm1.write(arm1_angle);
  } else if (message[0] == 'K'){// Tín hiệu có dạng Kxxx với xxx là giá trị từ slider, Nếu tín hiệu bắt đầu bằng K thì đọc số xxx và xoay servo
    arm2_angle = message.substring(1).toInt(); // "K123".substring(1) -> "123", "123".toInt() -> 123 -> Servo quay về góc 123
    arm2.write(arm2_angle);
  } 
  }
}


void setmotor(int speedA, int speedB) { // Điều khiển tốc độ bánh xe
// Giới hạn giá trị speed từ -maxspeed đến maxspeed
  speedA = constrain(speedA, -maxspeed, maxspeed); 
  speedB = constrain(speedB, -maxspeed, maxspeed); 
// Nếu bánh nào ngược thì uncomment bánh đó
  speedB = -speedB;
  speedA = -speedA;

  if (speedA > 0){ // Nếu giá trị lớn hơn 0, bánh tiến tới bằng 1 dây 0v, 1 dây có xung PWM 
    digitalWrite(MOTOR_R_2, LOW); // Set chân MOTOR_R_2 ở mức 0v
    analogWrite(MOTOR_R_1, speedA); // Tạo xung PWM bằng speedA
  } else{  // Ngược lại bánh tiến tới bằng 1 dây 5v, 1 dây có xung PWM 
    digitalWrite(MOTOR_R_2, HIGH);
    analogWrite(MOTOR_R_1,255 + speedA);
  }
  if (speedB > 0){// Nếu giá trị lớn hơn 0, bánh tiến tới bằng 1 dây 0v, 1 dây có xung PWM 
    digitalWrite(MOTOR_L_1, LOW);
    analogWrite(MOTOR_L_2, speedB);
  } else{// Ngược lại bánh tiến tới bằng 1 dây 5v, 1 dây có xung PWM 
    digitalWrite(MOTOR_L_1, HIGH);
    analogWrite(MOTOR_L_2, 255 + speedB);
  }
}

