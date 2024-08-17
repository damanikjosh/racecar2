#include <Arduino.h>
#include "PID.h"
#include "Utils.h"
#include "Encoder.h"
#include "ESC.h"
#include "ROS.h"

#define ENCODER_PIN 2
#define ESC_PIN 11
#define STEERING_PIN 10

int reverse_ = 1;
EMA setpoint_ema_(0.1);
ESC esc;
Servo steering;

Encoder encoder;

// PID variables
float setpoint_ = 0;
float output_ = 0;
float input_ = 0;

float pidSource() {
  return (input_ - setpoint_);
}

void pidOutput(float output) {
  output_ = output;
}


ROS ros;
PIDController<float> pid(0.5, 0.0035, 1.0, pidSource, pidOutput);

void setup() {
  ros.init();
  pid.registerTimeFunction(micros);
  encoder.init(ENCODER_PIN);
  esc.init(ESC_PIN);
  steering.attach(STEERING_PIN);
  steering.writeMicroseconds(PWM_NEUTRAL);

  last_traction_time = millis();
  last_steering_time = millis();
}

void loop() {
  unsigned long curr_time = millis();
  
  ros.spinOnce();
  if (curr_time - last_traction_time > 500) {
    remote_traction = 0;
  }
  if (curr_time - last_steering_time > 500) {
    remote_steering = 0;
  }

  steering.writeMicroseconds(steering2PWM(remote_steering));
  setpoint_ = remote_traction;

  if (rpm_ < 10 && setpoint_ > 0) {
    reverse_ = 1;
  }
  else if (rpm_ < 10 && setpoint_ < -0) {
    reverse_ = -1;
  }
  input_ = rpm_ * reverse_;

  pid.tick();
  output_ = pid.getOutput();
  if (abs(setpoint_) > 20) {
    esc.spin(output_);
  } else {
    esc.spin(0);
  }
  delay(1);
}
