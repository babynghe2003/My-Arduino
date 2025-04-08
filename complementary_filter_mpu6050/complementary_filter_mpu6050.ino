 #include<Wire.h>
#include <SimpleKalmanFilter.h>

#define MPU6050_AXOFFSET 158
#define MPU6050_AYOFFSET 9
#define MPU6050_AZOFFSET -91
#define MPU6050_GXOFFSET 19
#define MPU6050_GYOFFSET -42
#define MPU6050_GZOFFSET -26

long sampling_timer;
const int MPU_addr=0x68;  // I2C address of the MPU-6050

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // Raw data of MPU6050
float GAcX, GAcY, GAcZ; // Convert accelerometer to gravity value
float Cal_GyX,Cal_GyY,Cal_GyZ; // Pitch, Roll & Yaw of Gyroscope applied time factor
float acc_pitch, acc_roll, acc_yaw; // Pitch, Roll & Yaw from Accelerometer
float angle_pitch, angle_roll, angle_yaw; // Angle of Pitch, Roll, & Yaw
float alpha = 0.996; // Complementary constant
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

void setup(){
  Wire.begin();
 
  init_MPU6050();
  
  Serial.begin(115200);
}

void loop(){

  // Read raw data of MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // Raw data of accelerometer corrected by offset value
//  AcX -= MPU6050_AXOFFSET;
//  AcY -= MPU6050_AYOFFSET;
//  AcZ -= MPU6050_AZOFFSET;

  // Convert accelerometer to gravity value
  GAcX = (float) AcX / 4096.0;
  GAcY = (float) AcY / 4096.0;
  GAcZ = (float) AcZ / 4096.0;

  // Calculate Pitch, Roll & Yaw from Accelerometer value
  // Reference are 
  // https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
  // https://www.dfrobot.com/wiki/index.php/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
  acc_pitch = atan ((GAcY - (float)MPU6050_AYOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951; // 180 / PI = 57.29577951
  acc_roll = - atan ((GAcX - (float)MPU6050_AXOFFSET/4096.0) / sqrt(GAcY * GAcY + GAcZ * GAcZ)) * 57.29577951; 
  //acc_yaw = atan ((GAcZ - (float)MPU6050_AZOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951;
  acc_yaw = atan (sqrt(GAcX * GAcX + GAcZ * GAcZ) / (GAcZ - (float)MPU6050_AZOFFSET/4096.0)) * 57.29577951; 

  // Calculate Pitch, Roll & Yaw from Gyroscope value reflected cumulative time factor
  Cal_GyX += (float)(GyX - MPU6050_GXOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
  Cal_GyY += (float)(GyY - MPU6050_GYOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
  Cal_GyZ += (float)(GyZ - MPU6050_GZOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)

  // Calculate Pitch, Roll & Yaw by Complementary Filter
  // Reference is http://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/
  // Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)     
  // where α = τ/(τ + Δt)   and   (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
  // Δt = sampling rate, τ = time constant greater than timescale of typical accelerometer noise
  angle_pitch = alpha * (((float)(GyX - MPU6050_GXOFFSET) * 0.000244140625) + angle_pitch) + (1 - alpha) * acc_pitch;
  angle_roll = alpha * (((float)(GyY - MPU6050_GYOFFSET) * 0.000244140625) + angle_roll) + (1 - alpha) * acc_roll;
  angle_yaw += (float)(GyZ - MPU6050_GZOFFSET) * 0.000244140625; // Accelerometer doesn't have yaw value
  
  // Print raw of accelerometer & gyroscope reflected cumulative time factor
//  Serial.print("AcX = "); Serial.print(AcX);
//  Serial.print(" | AcY = "); Serial.print(AcY);
//  Serial.print(" | AcZ = "); Serial.println(AcZ);
//  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.print(" | Cal_GyX = "); Serial.print(Cal_GyX);
//  Serial.print(" | Cal_GyY = "); Serial.print(Cal_GyY);
//  Serial.print(" | Cal_GyZ = "); Serial.println(Cal_GyZ);

  // Print calculated value of Pitch, Roll & Yaw from Accelerometer value
//  Serial.print(" | acc_pitch = "); Serial.print(acc_pitch);
//  Serial.print(" | acc_roll = "); Serial.print(acc_roll);
//  Serial.print(" | acc_yaw = "); Serial.println(acc_yaw);

  // Print value of Pitch, Roll & Yaw reflected Complementary Filter
//  Serial.print(" | angle_pitch = "); Serial.print(angle_pitch);
//  Serial.print(" | angle_roll = "); Serial.print(angle_roll);
//  Serial.print(" | angle_yaw = "); Serial.println(angle_yaw);

  // Serial.print(" | Cal_GyX = "); Serial.print(Cal_GyX);
  // Serial.print("*"); 
  // Serial.print("10 -10 ");
  // Serial.print("Rawdata: ");
  Serial.print(acc_pitch - 3);
  Serial.print(" "); 

  Serial.print(simpleKalmanFilter.updateEstimate(acc_pitch-3));
  Serial.print(" "); 
  // Serial.print("Filtered: ");
  Serial.println(angle_pitch);
  // Serial.println("#");
  // delay(1);
  // Sampling Timer
  // while(micros() - sampling_timer < 4000); //
  // sampling_timer = micros(); //Reset the sampling timer  
}

void init_MPU6050(){
  //MPU6050 Initializing & Reset
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //MPU6050 Clock Type
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference'
  Wire.endTransmission(true);

  //MPU6050 Gyroscope Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // Gyroscope Configuration register
  //Wire.write(0x00);     // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
  //Wire.write(0x08);     // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
  //Wire.write(0x10);     // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
  Wire.write(0x18);     // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]
  Wire.endTransmission(true);

  //MPU6050 Accelerometer Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // Accelerometer Configuration register
  //Wire.write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g]
  //Wire.write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g]
  Wire.write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g]
  //Wire.write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g]
  Wire.endTransmission(true);

  //MPU6050 DLPF(Digital Low Pass Filter)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);  // DLPF_CFG register
  Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
  //Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz 
  //Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz 
  //Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz 
  //Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz 
  //Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz 
  //Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz 
  Wire.endTransmission(true);
}
