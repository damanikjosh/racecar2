#include "racecar_control/racecar_odometry.hpp"

namespace racecar_control {

RacecarOdometry::RacecarOdometry(size_t rolling_window_size)
: x_(0.0),
  y_(0.0),
  yaw_(0.0),
  v_(0.0),
  w_(0.0),
  wheel_base_(0.0),
  wheel_track_(0.0),
  wheel_radius_(0.0),
  rolling_window_size_(rolling_window_size),
  v_accumulator_(rolling_window_size),
  w_accumulator_(rolling_window_size)
{
}

void RacecarOdometry::init() {
  reset_accumulators();
}

void RacecarOdometry::update(double throttle_velocity, double steering_angle, double dt) {
  steering_angle_ = steering_angle;
  double v = throttle_velocity * wheel_radius_;  // Assuming wheel_radius_ is correctly set to account for the actual wheel radius.
  if (is_close_to_zero(steering_angle)) {
    update_odometry(v, 0.0, dt);
    return;
  }
  double w = v * std::tan(steering_angle) / wheel_base_;
  update_odometry(v, w, dt);
}

bool RacecarOdometry::update_odometry(const double v, const double w, const double dt) {
  integrate_fk(v, w, dt);

  if (dt < 0.0001) {
    return false;
  }

  // v_ = v;
  // w_ = w;

  v_accumulator_.accumulate(v);
  w_accumulator_.accumulate(w);

  v_ = v_accumulator_.getRollingMean();
  w_ = w_accumulator_.getRollingMean();

  return true;
}

void RacecarOdometry::set_wheel_params(double wheel_base, double wheel_track, double wheel_radius) {
  wheel_base_ = wheel_base;
  wheel_track_ = wheel_track;
  wheel_radius_ = wheel_radius;
}

void RacecarOdometry::set_rolling_window_size(size_t rolling_window_size) {
  rolling_window_size_ = rolling_window_size;
  reset_accumulators();
}

void RacecarOdometry::integrate_runge_kutta_2(
  const double v, const double w, const double dt) {
  const double yaw_mid = yaw_ + w * 0.5 * dt;
  x_ += v * std::cos(yaw_mid) * dt;
  y_ += v * std::sin(yaw_mid) * dt;
  yaw_ += w * dt;
}

void RacecarOdometry::integrate_fk(const double v, const double w, const double dt) {
  // const double delta_x = v * dt;
  const double delta_yaw = w * dt;

  if (is_close_to_zero(delta_yaw)) {
    integrate_runge_kutta_2(v, w, dt);
  } else {
    const double yaw_old = yaw_;
    const double R = v / w;
    yaw_ += delta_yaw;
    x_ += R * (sin(yaw_) - std::sin(yaw_old));
    y_ += -R * (cos(yaw_) - std::cos(yaw_old));

  }
}

void RacecarOdometry::reset_odometry() {
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
  v_ = 0.0;
  w_ = 0.0;
  reset_accumulators();
}

void RacecarOdometry::reset_accumulators() {
  v_accumulator_ = RollingMeanAccumulator<double>(rolling_window_size_);
  w_accumulator_ = RollingMeanAccumulator<double>(rolling_window_size_);
}

}  // namespace racecar_control
