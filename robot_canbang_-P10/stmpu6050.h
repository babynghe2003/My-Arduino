#ifndef _SAIGONTECH_MPU6050_H_
#define _SAIGONTECH_MPU6050_H_

#include "Wire.h"
#define   RAD2DEG   57.295779

class SMPU6050 {
  public:
    SMPU6050() {
    };

    void init(int address) {
      this->i2cAddress = address;

      this->gyroXOffset = 0;
      this->gyroYOffset = 0;
      this->gyroZOffset = 0;

      this->xAngle = 0;
      this->yAngle = 0;
      this->zAngle = 0;


      this->accX = 0;
      this->accY = 0;

      this->prevMillis = millis();

// reset và cấu hình MPU6050
      Wire.begin();
      Wire.beginTransmission(this->i2cAddress);
      Wire.write(0x6B);// đi đến thanh ghi PWR_MGMT_1 
      Wire.write(0);   // reset MPU6050
      Wire.endTransmission(true);

      Wire.beginTransmission(this->i2cAddress);
      //đi tới thanh ghi SMPLRT_DIV thanh ghi chia tỉ lệ lấy mẫu
      Wire.write(0x19); //Sample Rate = Gyrocope Output Rate/(1+SMPLRT_DIV)
      Wire.write(0);  //Giá trị không dấu 8 bit. Tốc độ mẫu được xác định bằng cách chia tốc độ đầu ra của con quay hồi chuyển cho giá trị này
      Wire.endTransmission(true);

      Wire.beginTransmission(this->i2cAddress);
      Wire.write(0x1B); //thanh ghi cấu hình GYRO_CONFIG Thanh ghi này được sử dụng để kích hoạt tự kiểm tra con quay hồi chuyển và cấu hình phạm vi quy mô đầy đủ của con quay hồi chuyển.
      Wire.write(0); //=0 -->phạm vi quy mô đầy đủ của đầu ra con quay hồi chuyển = +- 250 dec/s không tự kiểm tra
      Wire.endTransmission(true);

      Wire.beginTransmission(this->i2cAddress);
      Wire.write(0x1C); //Cấu hình gia tốc ACCEL_CONFIG Thanh ghi này được sử dụng để kích hoạt tự kiểm tra gia tốc kế và định cấu hình phạm vi toàn thang đo gia tốc. Thanh ghi này cũng cấu hình Bộ lọc thông cao kỹ thuật số (DHPF).
      Wire.write(0);// =0 -->chọn phạm vi toàn thang đo của các đầu ra gia tốc = +- 2g .không tự kiểm tra
      Wire.endTransmission(true);
    }
//hàm hiệu chỉnh
    void calibrate(int times) {
      long gyroXTotal = 0, gyroYTotal = 0, gyroZTotal = 0;
      int count = 0;
      int gyroRawX, gyroRawY, gyroRawZ;
      for (int i = 0; i < times; i++) {
        Wire.beginTransmission(this->i2cAddress);
        Wire.write(0x43); //3thanh ghi 16 bit từ (0x43-0x48) GYRO_XOUT  GYRO_YOUT  GYRO_ZOUT Những thanh ghi này lưu trữ các phép đo con quay gần đây nhất
        Wire.endTransmission(false);
        Wire.requestFrom(this->i2cAddress, 6, true);

        gyroRawX = Wire.read() << 8 | Wire.read();
        gyroRawY = Wire.read() << 8 | Wire.read();
        gyroRawZ = Wire.read() << 8 | Wire.read();

        gyroXTotal += gyroRawX;
        gyroYTotal += gyroRawY;
        gyroZTotal += gyroRawZ;
        count += 1;
      }
      gyroXOffset = -gyroXTotal * 1.0 / count;
      gyroYOffset = -gyroYTotal * 1.0 / count;
      gyroZOffset = -gyroZTotal * 1.0 / count;
    }

    double getXAngle() {
      this->readAngles();
      return this->xAngle;
    };

    double getYAngle() {
      this->readAngles();
      return this->yAngle;
    };

    double getZAngle() {
      this->readAngles();
      return this->zAngle;
    };





    double getXAcc() {
      this->readAngles();
      return this->accX;
    };

    double getYAcc() {
      this->readAngles();
      return this->accY;
    };










    void getXYZAngles(double &x, double &y, double &z) {
      this->readAngles();
      x = xAngle;
      y = yAngle;
      z = zAngle;
    }

  private:
    int i2cAddress;
    double accX, accY, gyroX, gyroY, gyroZ;
    double gyroXOffset, gyroYOffset, gyroZOffset;
    double xAngle, yAngle, zAngle;
    unsigned long prevMillis;

    void readAngles() {
      if (millis() - this->prevMillis < 3)
        return;

      int accRawX, accRawY, accRawZ, gyroRawX, gyroRawY, gyroRawZ;


      Wire.beginTransmission(this->i2cAddress);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(this->i2cAddress, 14, true);

      accRawX = Wire.read() << 8 | Wire.read();
      accRawY = Wire.read() << 8 | Wire.read();
      accRawZ = Wire.read() << 8 | Wire.read();
      Wire.read(); Wire.read();
      gyroRawX = Wire.read() << 8 | Wire.read();
      gyroRawY = Wire.read() << 8 | Wire.read();
      gyroRawZ = Wire.read() << 8 | Wire.read();

      accX = atan((accRawY / 16384.0) / sqrt(pow((accRawX / 16384.0), 2) + pow((accRawZ / 16384.0), 2))) * RAD2DEG;
      accY = atan(-1 * (accRawX / 16384.0) / sqrt(pow((accRawY / 16384.0), 2) + pow((accRawZ / 16384.0), 2))) * RAD2DEG;

      gyroX = (gyroRawX + gyroXOffset) / 131.0;
      gyroY = (gyroRawY + gyroYOffset) / 131.0;
      gyroZ = (gyroRawZ + gyroZOffset) / 131.0;

      unsigned long curMillis = millis();
      double duration = (curMillis - prevMillis) * 1e-3;
      prevMillis = curMillis;

      xAngle = 0.98 * (xAngle + gyroX * duration) + 0.02 * accX;
      yAngle = 0.98 * (yAngle + gyroY * duration) + 0.02 * accY;
      zAngle = zAngle + gyroZ * duration;
    }
};

void mpu6050Init(SMPU6050 &smpu, int address) {
  smpu.init(address);
}

void mpu6050Calibrate(SMPU6050 &smpu, int times) {
  smpu.calibrate(times);
}

double mpu6050GetXAngle(SMPU6050 &smpu) {
  return smpu.getXAngle();
}

double mpu6050GetYAngle(SMPU6050 &smpu) {
  return smpu.getYAngle();
}

double mpu6050GetZAngle(SMPU6050 &smpu) {
  return smpu.getZAngle();
}

void mpu6050GetXYZAngles(SMPU6050 &smpu, double &xAngle, double &yAngle, double &zAngle) {
  smpu.getXYZAngles(xAngle, yAngle, zAngle);
}

#endif  /*_SAIGONTECH_MPU6050_H_*/
