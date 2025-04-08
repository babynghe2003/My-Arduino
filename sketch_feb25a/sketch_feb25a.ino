#include <PS2X_lib.h>

#define PS2_DAT        19  // PS2 controller data pin
#define PS2_CMD        18  // PS2 controller command pin
#define PS2_SEL        5   // PS2 controller select pin
#define PS2_CLK        23  // PS2 controller clock pin

PS2X ps2x;

void setup() {
  Serial.begin(115200);
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_DAT, PS2_SEL, true, true);
}

void loop() {
  ps2x.read_gamepad();
  int x = ps2x.Analog(PSS_LX);
  int y = ps2x.Analog(PSS_LY);
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.println(y);
}