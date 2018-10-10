#include "DualMC33926MotorShield.h"

// Set speed for motor 1, speed is a number betwenn -400 and 400
void DualMC33926MotorShield::setM1Speed(int speed)
{
  unsigned char reverse = 0;

  _m1dir = 1;
  if (speed < 0)
  {
    speed = -speed; // Make speed a positive quantity
    reverse = 1;    // Preserve the direction
    _m1dir = -1;
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;

  _M1PWM.write(float(speed) / 400.0); // map 400 to 1

  if (reverse)
    _M1DIR = 1;
  else
    _M1DIR = 0;
}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void DualMC33926MotorShield::setM2Speed(int speed)
{
  unsigned char reverse = 0;

  _m2dir = 1;
  if (speed < 0)
  {
    speed = -speed; // Make speed a positive quantity
    reverse = 1;    // Preserve the direction
    _m2dir = -1;
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;

  _M2PWM.write(float(speed) / 400.0); // map 400 to 1

  if (reverse)
    _M2DIR = 1;
  else
    _M2DIR = 0;
}

// Set speed for motor 1 and 2
void DualMC33926MotorShield::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

int DualMC33926MotorShield::readAverageM1FB(int times) {
    long sum = 0;
    for (int i = 0; i < times; i++) {
        sum += _M1FB.read_u16();
    }
    return sum / times;
}

int DualMC33926MotorShield::readAverageM2FB(int times) {
    long sum = 0;
    for (int i = 0; i < times; i++) {
        sum += _M2FB.read_u16();
    }
    return sum / times;
}

// Return motor 1 current value in milliamps.
float DualMC33926MotorShield::getM1CurrentMilliamps()
{
  // 3.3V = 65535 ADC counts = 3.3V / 0.525V/A = 6.286 A
  // 3.3/65535 V = 1 ADC counts = 6.286/65535 A
  // 3.3v / 0.525v/A / 65535 ADC counts = 0.000095914 A/count = 0.095914 mA/count
  _m1fb_lpf = (float(_m1dir*(_M1FB.read_u16() - _m1fb_offset)) * 0.095914f) * _lpf_gain + _m1fb_lpf * (1.0 - _lpf_gain);
  return _m1fb_lpf;
}

// Return motor 2 current value in milliamps.
float DualMC33926MotorShield::getM2CurrentMilliamps()
{
  _m2fb_lpf = (float(_m2dir*(_M2FB.read_u16() - _m2fb_offset)) * 0.095914f) * _lpf_gain + _m2fb_lpf * (1.0 - _lpf_gain);
  return _m2fb_lpf;
}

// Return error status
unsigned char DualMC33926MotorShield::getFault()
{
  return !_nSF.read();
}
