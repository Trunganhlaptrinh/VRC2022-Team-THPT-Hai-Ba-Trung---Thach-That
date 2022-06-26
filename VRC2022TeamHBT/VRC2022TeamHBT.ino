#include "motors.h"
#include "PS2_controller.h"
#include <PS2X_lib.h>

void setup()
{
  
  Serial.begin(115200);
  initMotors();
  setupPS2controller();
}

void loop()
{
  ps2x.read_gamepad(false, 0);
  PS2control();
}
