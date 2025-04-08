#include <SimpleFOC.h>
#define _PWM_FREQUENCY 30000 // default
#define _PWM_FREQUENCY_MAX 50000 // mqx
// BLDC motor & driver instance
// BLDCMotor(pole pairs, phase resistance, KV rating)
BLDCMotor motor = BLDCMotor(7, 10, 150); 
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 2, 4, 16);

void setup() {
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor current (provided resistance)
  motor.current_limit = 0.3;   // [Amps]
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();
  motor.initFOC();

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Running motor speed from -20 to 20 rad/s");
  _delay(1000);
}

void loop() {
  static float target_velocity = 10;
  static bool increasing = true;

  // Set the target velocity
  motor.target = target_velocity;

  // Update motor control
  motor.loopFOC();
  motor.move();

  // // Change direction when limits are reached
  // if (increasing) {
  //   target_velocity += 0.1;
  //   if (target_velocity >= 100.0) {
  //     increasing = false;
  //   }
  // } else {
  //   target_velocity -= 0.1;
  //   if (target_velocity <= -100.0) {
  //     increasing = true;
  //   }
  // }

  // // Delay to control the speed change rate
  // _delay(10);
}