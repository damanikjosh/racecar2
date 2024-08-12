#ifndef RACECAR_ODOMETRY__RACECAR_ODOMETRY_HPP_
#define RACECAR_ODOMETRY__RACECAR_ODOMETRY_HPP_

#include <cmath>
#include "racecar_control/racecar_odometry.hpp"
#include "racecar_control/rolling_mean_accumulator.hpp"

namespace racecar_control
{
inline bool is_close_to_zero(double val) { return std::fabs(val) < 1e-6; }

class RacecarOdometry
{
public:
  explicit RacecarOdometry(size_t rolling_window_size);

  void init();
  void update(double throttle_velocity, double steering_angle, double dt);
  void set_wheel_params(double wheel_base, double wheel_track, double wheel_radius);
  void set_rolling_window_size(size_t rolling_window_size);
  double get_x() const { return x_; }
  double get_y() const { return y_; }
  double get_yaw() const { return yaw_; }
  double get_v() const { return v_; }
  double get_w() const { return w_; }
  void reset_odometry();

private:

  bool update_odometry(const double v, const double w, const double dt);
  void integrate_runge_kutta_2(const double v, const double w, const double dt);
  void integrate_fk(const double v, const double w, const double dt);
  void reset_accumulators();

  double x_;
  double y_;
  double yaw_;

  double v_;
  double w_;

  double steering_angle_;

  double wheel_base_;
  double wheel_track_;
  double wheel_radius_;

  size_t rolling_window_size_;
  RollingMeanAccumulator<double> v_accumulator_;
  RollingMeanAccumulator<double> w_accumulator_;
};
}

#endif // RACECAR_ODOMETRY__RACECAR_ODOMETRY_HPP_