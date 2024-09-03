#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <cmath>

class TwistToAckermannNode : public rclcpp::Node
{
public:
    TwistToAckermannNode()
    : Node("twist_to_ackermann_node")
    {
        // Declare the wheelbase parameter with a default value
        this->declare_parameter<double>("wheelbase", 0.320);
        
        // Get the parameter value
        wheelbase_ = this->get_parameter("wheelbase").as_double();

        // Subscriber to /cmd_vel
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&TwistToAckermannNode::twist_callback, this, std::placeholders::_1));

        // Publisher to /ackermann_cmd
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/ackermann_cmd", 10);
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Create an AckermannDrive message
        auto ackermann_msg = ackermann_msgs::msg::AckermannDrive();

        // Convert Twist to AckermannDrive
        ackermann_msg.speed = msg->linear.x; // Linear velocity

        // Calculate the steering angle using the wheelbase
        if (msg->linear.x != 0.0) {
            ackermann_msg.steering_angle = std::atan2(wheelbase_ * msg->angular.z, msg->linear.x);
        } else {
            ackermann_msg.steering_angle = 0.0;
        }

        // Publish the AckermannDrive message
        ackermann_publisher_->publish(ackermann_msg);
    }

    double wheelbase_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToAckermannNode>());
    rclcpp::shutdown();
    return 0;
}
