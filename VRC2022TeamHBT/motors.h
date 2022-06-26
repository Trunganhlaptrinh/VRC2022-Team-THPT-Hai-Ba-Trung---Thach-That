#include <stdio.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PWM 0
#define MAX_PWM 4095

//#define SDA 3
//#define SCL 0
// PWM channels of pca9685 0-16
#define PWM_CHANNEL1 8
#define PWM_CHANNEL2 9
#define PWM_CHANNEL3 10
#define PWM_CHANNEL4 11
#define PWM_CHANNEL5 12
#define PWM_CHANNEL6 13
#define PWM_CHANNEL7 14
#define PWM_CHANNEL8 15

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setPWMMotors(int c1, int c2, int c3, int c4)
{


  pwm.setPWM(PWM_CHANNEL1, c1, MAX_PWM - c1);
  pwm.setPWM(PWM_CHANNEL2, c2, MAX_PWM - c2);
  pwm.setPWM(PWM_CHANNEL7, c3, MAX_PWM - c3);
  pwm.setPWM(PWM_CHANNEL8, c4, MAX_PWM - c4);
}


void initMotors()
{
  Wire.begin(); //SDA, SCL,400000);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);

  setPWMMotors(0, 0, 0, 0);
  pwm.setPWM(PWM_CHANNEL5, 0, 0);
  pwm.setPWM(PWM_CHANNEL6, 0, 0);
  pwm.setPWM(7,0,0);
  pwm.setPWM(6,0,0);
}
