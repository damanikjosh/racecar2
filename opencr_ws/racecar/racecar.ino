#include <Arduino.h>
#include <IMU.h>


#include "PID.h"
#include "Utils.h"
#include "Encoder.h"
#include "ESC.h"
#include "ROS.h"

#define ENCODER_PIN 8
#define ESC_PIN 11
#define STEERING_PIN 10

#define CONTROL_FREQ 100
#define IMU_FREQ 100

int reverse_ = 1;
EMA setpoint_ema_(0.1);
ESC esc;
Servo steering;
cIMU imu;

Encoder encoder;

// PID variables
float setpoint_ = 0;
float output_ = 0;
float input_ = 0;

unsigned long last_control_time;
unsigned long last_imu_time;


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
  imu.begin();
  encoder.init(ENCODER_PIN);
  esc.init(ESC_PIN);
  steering.attach(STEERING_PIN);
  steering.writeMicroseconds(steering2PWM(0));

  last_traction_time = millis();
  last_steering_time = millis();
  last_control_time = millis();
  last_imu_time = millis();
}

void loop() {
  unsigned long curr_time = millis();
  
  if (curr_time - last_traction_time >= 500) {
    remote_traction = 0;
  }
  if (curr_time - last_steering_time >= 500) {
    remote_steering = 0;
  }

  ros.spinOnce();
  imu.update();

  if (curr_time - last_imu_time >= 1000 / IMU_FREQ) {
    last_imu_time = curr_time;
    ros.publishImu(&imu);
  }

  if (curr_time - last_control_time >= 1000 / CONTROL_FREQ) {
    last_control_time = curr_time;
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
    if (setpoint_ >= 0 && output_ < 0) {
      output_ = 0;
    } else if (setpoint_ < 0 && output_ > 0) {
      output_ = 0;
    }
    if (abs(setpoint_) > 20) {
      esc.spin(output_);
    } else {
      esc.spin(0);
    }

    ros.publishThrottle(input_);
    ros.publishSteering(remote_steering);
  }
}
