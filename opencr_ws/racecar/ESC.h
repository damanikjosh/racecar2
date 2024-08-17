#ifndef ESC_H
#define ESC_H

#include <Servo.h>

class ESC {
public:
  #define PWM_HIGH 2000
  #define PWM_NEUTRAL 1500
  #define PWM_LOW 1000
  #define GEAR_RATIO 2.8

  void init(int pin) {
    motor_.attach(pin);
    motor_.writeMicroseconds(PWM_NEUTRAL);
  }

  void spin(int traction) {
    motor_.writeMicroseconds(traction2PWM(traction));
  }

private:
  Servo motor_;
};

#endif