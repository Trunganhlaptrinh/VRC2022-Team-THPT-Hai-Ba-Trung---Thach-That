#include "DC1_motors.h"
#include "DC1_PS2_controller.h"

void setup()
{
  Serial.begin(115200);
  initMotors();
  setupPS2controller();
  Serial.println("Done setup!");
}

void loop()
{
  ps2x.read_gamepad(false, 0);
  PS2control();
}
