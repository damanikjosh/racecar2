#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include "realtime_tools/realtime_publisher.h"
#include "racecar_control/racecar_odometry.hpp"

namespace racecar_control
{
class RacecarControllerNode : public rclcpp::Node
{
public:
  RacecarControllerNode() : Node("racecar_controller"), odometry_(10), initialized_(false)
  {
    // Publisher setup
    auto odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>(odom_pub));
    auto throttle_pub = this->create_publisher<std_msgs::msg::Float32>("throttle/velocity/command", 10);
    throttle_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::msg::Float32>(throttle_pub));
    auto steering_pub = this->create_publisher<std_msgs::msg::Float32>("steering/position/command", 10);
    steering_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::msg::Float32>(steering_pub));

    // Get the parameters from the parameter server
    this->declare_parameter("wheel_base", 0.356);
    this->declare_parameter("wheel_track", 0.325);
    this->declare_parameter("wheel_radius", 0.069);
    this->declare_parameter("rolling_window_size", 10);
    this->declare_parameter("odom_frequency", 100);
    this->declare_parameter("pose_covariance", std::vector<double>{0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0});
    this->declare_parameter("twist_covariance", std::vector<double>{0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0});

    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("wheel_track", wheel_track_);
    this->get_parameter("wheel_radius", wheel_radius_);

    int rolling_window_size, odom_frequency;
    this->get_parameter("rolling_window_size", rolling_window_size);
    this->get_parameter("odom_frequency", odom_frequency);

    // Get covariance diagonal values
    std::vector<double> pose_covariance, twist_covariance;
    this->get_parameter("pose_covariance", pose_covariance);
    this->get_parameter("twist_covariance", twist_covariance);

    // Set the covariance values
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = "base_link";
    odom_pub_->msg_.pose.covariance = {
      pose_covariance[0], 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, pose_covariance[1], 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, pose_covariance[2], 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, pose_covariance[3], 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, pose_covariance[4], 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, pose_covariance[5]
    };

    odom_pub_->msg_.twist.covariance = {
      twist_covariance[0], 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, twist_covariance[1], 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, twist_covariance[2], 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, twist_covariance[3], 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, twist_covariance[4], 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, twist_covariance[5]
    };
    
    odometry_.set_wheel_params(wheel_base_, wheel_track_, wheel_radius_);
    odometry_.set_rolling_window_size(rolling_window_size);
    odometry_.init();

    odom_frequency_ = odom_frequency;

    // Subscriber setup
    throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "throttle/velocity/reference", 10,
      std::bind(&RacecarControllerNode::throttle_callback, this, std::placeholders::_1));
    
    steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "steering/position/reference", 10,
      std::bind(&RacecarControllerNode::steering_callback, this, std::placeholders::_1));

    ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
      "ackermann_cmd", 10,
      std::bind(&RacecarControllerNode::ackermann_callback, this, std::placeholders::_1));
  
    // Create a timer to update odometry
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / odom_frequency_),
      std::bind(&RacecarControllerNode::update_odometry, this)
    );
  }

private:
  // Callback function to process the input message
  void throttle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // Lock the mutex before updating the throttle velocity
    throttle_velocity_ = msg->data;
  }

  void steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // Lock the mutex before updating the steering angle
    steering_angle_ = msg->data;
  }

  void ackermann_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
  {
    last_cmd_time_ = this->now();

    // Lock the mutex before updating the throttle velocity and steering angle
    double throttle_cmd = msg->speed / wheel_radius_;
    double steering_cmd = msg->steering_angle;

    // Publish the throttle and steering angle
    std_msgs::msg::Float32 throttle_msg, steering_msg;
    throttle_msg.data = throttle_cmd;
    steering_msg.data = steering_cmd;

    throttle_pub_->msg_ = throttle_msg;
    throttle_pub_->unlockAndPublish();
    steering_pub_->msg_ = steering_msg;
    steering_pub_->unlockAndPublish();
  }

  // Function to update the odometry
  void update_odometry()
  {
    if (!initialized_)
    {
      last_update_time_ = this->now();
      last_cmd_time_ = this->now();
      initialized_ = true;
      return;
    }

    double dt = (this->now() - last_update_time_).seconds();
    last_update_time_ = this->now();

    odometry_.update(throttle_velocity_, steering_angle_, dt);
    // double x, y, yaw, v, w;
    
    // odometry_.get_pose(x, y, yaw);
    // odometry_.get_velocity(v, w);

    // Publish the odometry message
    odom_pub_->msg_.header.stamp = this->now();
    odom_pub_->msg_.pose.pose.position.x = odometry_.get_x();
    odom_pub_->msg_.pose.pose.position.y = odometry_.get_y();
    odom_pub_->msg_.pose.pose.position.z = 0.0;
    odom_pub_->msg_.pose.pose.orientation.x = 0.0;
    odom_pub_->msg_.pose.pose.orientation.y = 0.0;
    odom_pub_->msg_.pose.pose.orientation.z = std::sin(odometry_.get_yaw() / 2.0);
    odom_pub_->msg_.pose.pose.orientation.w = std::cos(odometry_.get_yaw() / 2.0);
    odom_pub_->msg_.twist.twist.linear.x = odometry_.get_v();
    odom_pub_->msg_.twist.twist.angular.z = odometry_.get_w();
    // nav_msgs::msg::Odometry odom_msg;
    // odom_msg.header.stamp = this->now();
    // odom_msg.pose.pose.position.x = x;
    // odom_msg.pose.pose.position.y = y;
    // odom_msg.pose.pose.position.z = 0.0;
    // odom_msg.pose.pose.orientation.x = 0.0;
    // odom_msg.pose.pose.orientation.y = 0.0;
    // odom_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    // odom_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    // odom_msg.twist.twist.linear.x = v;
    // odom_msg.twist.twist.angular.z = w;
    // odom_pub_->msg_ = odom_msg;
    odom_pub_->unlockAndPublish();
  }

  // Publisher and Subscriber members
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float32>> throttle_pub_, steering_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_, steering_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_sub_;

  // Create a timer to update the odometry
  rclcpp::TimerBase::SharedPtr timer_;

  // Last time the odometry was updated
  rclcpp::Time last_update_time_;
  rclcpp::Time last_cmd_time_;

  RacecarOdometry odometry_;

  bool initialized_;

  double throttle_velocity_;
  double steering_angle_;

  int odom_frequency_;

  // Parameters
  double wheel_base_;
  double wheel_track_;
  double wheel_radius_;

  // mutex for throttle and steering angle
  std::mutex throttle_mutex_;
  std::mutex steering_mutex_;
};
} // namespace racecar_control

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<racecar_control::RacecarControllerNode>());
  rclcpp::shutdown();
  return 0;
}
