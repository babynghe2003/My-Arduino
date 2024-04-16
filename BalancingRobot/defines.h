#ifndef DEFINES_H_
#define DEFINES_H_

#define ROBOT_NAME "Balancing Robot"
#define AP_PASSWORD "12345678"

#define PREF_VERSION 1 // if setting structure has been changed, count this number up to delete all settings
#define FORMAT_SPIFFS_IF_FAILED true

// -- Stepper motors
#define PIN_MOTOR_ENABLE 27
#define PIN_MOTOR_LEFT_STEP 5
#define PIN_MOTOR_LEFT_DIR 4
#define PIN_MOTOR_RIGHT_STEP 18
#define PIN_MOTOR_RIGHT_DIR 15
#define PIN_MOTOR_CURRENT 25

#define REVERSE_LEFT_MOTOR true
#define REVERSE_RIGHT_MOTOR false

#define MICRO_STEP 32
#define MAX_STEP_SPEED 360 // small wheels : 410, large wheels : 360

// -- PID control
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS / 1000000.0

#define PID_ANGLE_MAX 15
#define PID_POS_MAX 32
#define PID_SPEED_MAX 20

#define GYRO_FILTER_CONSTANT 0.996
#define GYRO_GAIN 1.0

// go
#define KP_ANGLE_GO 0.65
#define KI_ANGLE_GO 0
#define KD_ANGLE_GO 0.2
#define KN_ANGLE_GO 15

#define KP_POS_GO 1
#define KI_POS_GO 0
#define KD_POS_GO 1.2
#define KN_POS_GO 20

#define KP_SPEED_GO 6
#define KI_SPEED_GO 5
#define KD_SPEED_GO 0
#define KN_SPEED_GO 20

// stay (no swing pids, good for staying, bad for going)
#define KP_ANGLE_STAY 1.9
#define KI_ANGLE_STAY 0.57
#define KD_ANGLE_STAY 0.64
#define KN_ANGLE_STAY 8.2

#define KP_POS_STAY 1.0
#define KI_POS_STAY 1.85
#define KD_POS_STAY 0.3
#define KN_POS_STAY 10

// -- IMU
#define GYRO_SENSITIVITY 65.5

// -- Others
#define PIN_LED 2

#define DEAD_BATTERY_VOLT 11.1

#endif /* DEFINES_H_ */