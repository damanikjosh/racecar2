#ifndef Encoder_H
#define Encoder_H

#include <Arduino.h>
#include "Utils.h"


HardwareTimer timer(TIMER_CH1);
EMA rpm_ema_(0.1);

volatile unsigned long encoder_ = 0;
volatile unsigned long last_encoder_ = 0;
volatile float rpm_ = 0;

class Encoder {

public:
  #define ENCODER_FREQ 100


  void init(int pin) {
    pinMode(pin, INPUT_PULLUP);
    digitalWrite(pin, HIGH);
    attachInterrupt(digitalPinToInterrupt(pin), encoderInterrupt, RISING);

    timer.stop();
    timer.setPeriod(1000000 / ENCODER_FREQ);
    timer.attachInterrupt(timerInterrupt);
    timer.start();
  }

private:
  static void timerInterrupt() {
    noInterrupts();
    float current_rpm = 2.0 * M_PI * ENCODER_FREQ * (encoder_ - last_encoder_) / GEAR_RATIO;
    last_encoder_ = encoder_;
    interrupts();
    rpm_ = rpm_ema_.update(current_rpm);
  }

  static void encoderInterrupt() {
    encoder_++;
  }


};

#endif