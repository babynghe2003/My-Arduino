/*


  SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS CONTROLLED WITH YOUR SMARTPHONE
  Author: PHẠM THANH KHUYÊN
  Date: 31/10/2021
  Version: 1

  DỰ ĐỊNH dùng Arduino nano  kết hợp CNC Shield V4
  Điều khiển động cơ dùng 2 x A4988 hoặc  2 x DVR8825
  2 Đông cơ bước : size 42 1.8 step
  Cảm biến góc nghiêng : MPU6050



  thoi gian 1 xung = 20*x us = 0.00002*x s
  1 vong =3200 xung-------> thoi gian 1 vong ---> 3200*0.00002*x s
                                    V---------> 60 s
                                    V=60/(32*0.002*x)

  x=10 ---> v= 93.75 vong/ phut
  x=50----> v= 18.75 vong/phut
*/

#include "stmpu6050.h"
SMPU6050 mpu6050;



// ĐỊNH NGHĨA CHÂN CNC SHIELD V4
//                chân ARDUINO   ký hiệu trên          chân PORT AVR
//                             board Arduino nano       Atmega 328P
# define Enable       8            //D8                 //PORTB 0                    
# define Step_3       7            //D7                 //PORTD 7                    
# define Step_2       6            //D6                 //PORTD 6                    
# define Step_1       5            //D5                 //PORTD 5                    
# define Dir_3        4            //D4                 //PORTD 4                    
# define Dir_2        3            //D3                 //PORTD 3                    
# define Dir_1        2            //D2                 //PORTD 2  
# define MS3          9            //D9                 //PORTB 1 //các chân MS3 cua 2 MOtor1 và MS3 Motor2 nối chung
# define MS2          10           //D10                //PORTB 2 //các chân MS2 cua 2 MOtor1 và MS2 Motor2 nối chung
# define MS1          11           //D11                //PORTB 3 //các chân MS1 cua 2 MOtor1 và MS1 Motor2 nối chung





//     HÀM KHAI BÁO CÁC CHÂN ARDUINO NANO
//....................................
void  pin_INI() {
  pinMode(Enable, OUTPUT);
  pinMode(Step_1, OUTPUT);
  pinMode(Step_2, OUTPUT);
  pinMode(Step_3, OUTPUT);
  pinMode(Dir_1, OUTPUT);
  pinMode(Dir_2, OUTPUT);
  pinMode(Dir_3, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  digitalWrite(Enable, LOW);
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);
}



//     HÀM KHAI BÁO TIMER2
//....................................
void timer_INI() {

  /*fo=16.000.000/8=2.000.000 Hz
    To=1/fo=1/2.000.000 s=0.5us
    timer=40*0.5=20us */

  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000Hz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode Chế độ CTC bộ đếm được xóa về 0 khi giá trị bộ đếm (TCNT0) khớp với OCR0A
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
}


int8_t Dir_M1, Dir_M2, Dir_M3;                                               //Biến xác định hoạt động của động cơ và chiều quay Dir_Mx >0 quay thuận , Dir_Mx <0 quay ngược Dir_Mx =0 motor ngừng quay
volatile int Count_timer1, Count_timer2, Count_timer3;                       //đếm các lần TIMER xuất hiện trong chu kỳ xung STEP
volatile int32_t Step1, Step2, Step3;
int16_t Count_TOP1, Count_BOT1, Count_TOP2, Count_BOT2, Count_TOP3, Count_BOT3;  //vị trí cuối của phần cao và cuối phần thấp trong 1 chu kỳ xung STEP
float inputL, inputR, Offset, Output, I_L, I_R, input_lastL, input_lastR, OutputL, OutputR, M_L, M_R, MotorL, MotorR, Vgo, Vgo_L, Vgo_R;
float Kp = 15;
float Ki = 1.1;
float Kd = 0;




//     CHƯƠNG TRÌNH NGẮT CỦA TIMER2
//....................................
ISR(TIMER2_COMPA_vect) {

  if (Dir_M1 != 0) {                                                          //nếu MOTOR cho phép quay
    Count_timer1++;
    if (Count_timer1 <= Count_TOP1)PORTD |= 0b00100000;                        //nếu là nhịp nằm trong phần cao trong xung STEP
    else PORTD &= 0b11011111;                                                 //nếu là nhịp nằm trong phần thấp của xung STEP
    if (Count_timer1 > Count_BOT1) {
      Count_timer1 = 0;                             //nếu là nhịp cuối của 1 xung STEP
      if (Dir_M1 > 0)Step1++;
      else if (Dir_M1 < 0)Step1--;
    }




  }


  if (Dir_M2 != 0) {
    Count_timer2++;
    if (Count_timer2 <= Count_TOP2)PORTD |= 0b01000000;
    else PORTD &= 0b10111111;
    if (Count_timer2 > Count_BOT2) {
      Count_timer2 = 0;
      if (Dir_M2 > 0)Step2++;
      else if (Dir_M2 < 0)Step2--;
    }
  }


  if (Dir_M3 != 0) {
    Count_timer3++;
    if (Count_timer3 <= Count_TOP3)PORTD |= 0b10000000;
    else PORTD &= 0b01111111;
    if (Count_timer3 > Count_BOT3) {
      Count_timer3 = 0;
      if (Dir_M3 > 0)Step3++;
      else if (Dir_M3 < 0)Step3--;
    }
  }
}




//     HÀM TỐC ĐỘ DI CHUYỂN MOTOR1
//....................................
void Speed_M1(int16_t x) {
  if (x < 0) {
    Dir_M1 = -1;
    PORTD &= 0b11111011;
  }
  else if (x > 0) {
    Dir_M1 = 1;
    PORTD |= 0b00000100;
  }
  else Dir_M1 = 0;

  Count_BOT1 = abs(x);
  Count_TOP1 = Count_BOT1 / 2;
}





//     HÀM TỐC ĐỘ DI CHUYỂN MOTOR2
//....................................
void Speed_L(int16_t x) {
  if (x < 0) {
    Dir_M2 = -1;
    PORTD &= 0b11110111;
  }
  else if (x > 0) {
    Dir_M2 = 1;
    PORTD |= 0b00001000;
  }
  else Dir_M2 = 0;

  Count_BOT2 = abs(x);
  Count_TOP2 = Count_BOT2 / 2;
}






//     HÀM TỐC ĐỘ DI CHUYỂN MOTOR3
//....................................
void Speed_R(int16_t x) {
  if (x < 0) {
    Dir_M3 = -1;
    PORTD &= 0b11101111;
  }
  else if (x > 0) {
    Dir_M3 = 1;
    PORTD |= 0b00010000;
  }
  else Dir_M3 = 0;

  Count_BOT3 = abs(x);
  Count_TOP3 = Count_BOT3 / 2;
}



void setup() {
  mpu6050.init(0x68);
  Serial.begin(9600);               //Khai báo Serial
  pin_INI();                        //Khai báo PIN Arduino đấu nối 3 DRIVER A4988
  timer_INI();                      //Khai báo TIMER2
  delay(500);
}

void loop() {
  float AngleY = mpu6050.getYAngle();
  //Serial.println(AngleY);
  Offset = 5;
  Vgo=3;
  Vgo_L = -0.5;
  Vgo_R = 0.5;
  inputL = AngleY + Offset - Vgo_L; //Vgo>0  chạy lui,Vgo <0 chạy tới
  I_L += inputL;
  OutputL = Kp * inputL + Ki * I_L + Kd * (inputL - input_lastL);
  input_lastL = inputL;
  if (OutputL > -5 && OutputL < 5)OutputL = 0;
  OutputL = constrain(OutputL, -400, 400);



  inputR = AngleY + Offset - Vgo_R; //Vgo>0  chạy lui,Vgo <0 chạy tới
  I_R += inputR;
  OutputR = Kp * inputR + Ki * I_R + Kd * (inputR - input_lastR);
  input_lastR = inputR;
  if (OutputR > -5 && OutputR < 5)OutputR = 0;
  OutputR = constrain(OutputR, -400, 400);








  if (OutputL > 0)M_L = 405 - (1 / (OutputL + 9)) * 5500;
  else if (OutputL < 0)  M_L = -405 - (1 / (OutputL - 9)) * 5500;
  else M_L = 0;




  if (OutputR > 0)M_R = 405 - (1 / (OutputR + 9)) * 5500; //OutputR = 1    ----> M_R = -145
  //OutputR = 4.58 ----> M_R = 0
  //OutputR = 10   ----> M_R = 115.52
  //OutputR = 400  ----> M_R = 391.55
  else if (OutputR < 0)M_R = -405 - (1 / (OutputR - 9)) * 5500;
  else M_R = 0;



  if (M_L > 0)MotorL = 400 - M_L;
  else if (M_L < 0)MotorL = -400 - M_L;
  else MotorL = 0;

  if (M_R > 0)MotorR = 400 - M_R;
  else if (M_R < 0)MotorR = -400 - M_R;
  else MotorR = 0;



  Speed_L(MotorL);
  Speed_R(MotorR);


}
