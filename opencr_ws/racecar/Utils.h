#ifndef Utils_H
#define Utils_H

#define GEAR_RATIO 2.8

#define PWM_HIGH 2000
#define PWM_NEUTRAL 1500
#define PWM_LOW 1000

#define TRACTION_HIGH 500
#define TRACTION_LOW -500

#define STEERING_HIGH -0.785398
#define STEERING_LOW 0.785398

#define STEERING_TRIM -0.07

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int traction2PWM(float traction) {
  // return constrain(PWM_NEUTRAL + traction, PWM_LOW, PWM_HIGH);
  return mapFloat(traction, TRACTION_LOW, TRACTION_HIGH, PWM_LOW, PWM_HIGH);
}

float PWM2Traction(int pwm) {
  return 1.0 * constrain(pwm - PWM_NEUTRAL, TRACTION_LOW, TRACTION_HIGH);
}

int steering2PWM(float steering) {
  return mapFloat(steering + STEERING_TRIM, STEERING_LOW, STEERING_HIGH, PWM_LOW, PWM_HIGH);
}

class EMA {
  float alpha;        // Smoothing factor. Lower value smooths more.
  float value;        // Current EMA value

public:
  EMA(float alpha) : alpha(alpha), value(0) {}

  float update(float newValue) {
    value = alpha * newValue + (1 - alpha) * value;
    return value;
  }

  float get() {
    return value;
  }

  void reset(float newValue) {
    value = newValue;
  }
};

#endif // Utils_H