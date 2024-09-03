#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <algorithm>
#include <cmath>

class SafetyNode : public rclcpp::Node
{
public:
    SafetyNode() : Node("safety_node")
    {
        // Declare and get the threshold parameter
        this->declare_parameter<double>("threshold", 1.0);
        this->get_parameter("threshold", threshold_);

        // Initialize publisher with high QoS profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().best_effort();
        e_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("e_stop", 1);
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("safety/ackermann_cmd", 1);

        // Initialize subscriber with high QoS profile
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", qos, std::bind(&SafetyNode::laserScanCallback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", qos, std::bind(&SafetyNode::odomCallback, this, std::placeholders::_1));
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "bluetooth_teleop/joy", qos, std::bind(&SafetyNode::joyCallback, this, std::placeholders::_1));
    }

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (e_stop_)
        {
            stop();
            return;
        }
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double range = msg->ranges[i];
            double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
            double cos_angle = std::cos(angle);

            if (range / std::max(std::pow(speed_, 1.2)* std::pow(cos_angle, 20), 0.0001) < threshold_)
            {
                e_stop_ = true;
                break;
            }
        }

        // Publish the e_stop message
        auto e_stop_msg = std_msgs::msg::Bool();
        e_stop_msg.data = e_stop_;
        e_stop_publisher_->publish(e_stop_msg);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get the speed_ from the odometry message
        speed_ = msg->twist.twist.linear.x;
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check if buttons 8 and 9 are pressed together
        if (msg->buttons[8] && msg->buttons[9])
        {
            e_stop_ = false;
        }
    }

    void stop()
    {
        auto ackermann_msg = ackermann_msgs::msg::AckermannDrive();
        ackermann_msg.speed = 0.0;
        ackermann_msg.steering_angle = 0.0;
        drive_publisher_->publish(ackermann_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_publisher_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    double threshold_;
    bool e_stop_ = false;
    double speed_ = 1.0; // Example speed value, adjust as needed
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Use a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<SafetyNode>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}