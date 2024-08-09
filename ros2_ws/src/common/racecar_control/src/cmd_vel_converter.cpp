#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class CmdVelConverter : public rclcpp::Node
{
public:
    CmdVelConverter()
    : Node("cmd_vel_converter")
    {
        // Create a publisher for the cmd_vel topic (TwistStamped)
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_out", 10);

        // Create a subscriber for the cmd_vel_unstamped topic (Twist)
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_in", 10,
            std::bind(&CmdVelConverter::cmdVelCallback, this, std::placeholders::_1)
        );
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Create a TwistStamped message
        geometry_msgs::msg::TwistStamped twist_stamped_msg;

        // Set the current time as the header's timestamp
        twist_stamped_msg.header.stamp = this->now();

        // Set the frame id (optional, usually "base_link" or similar)
        twist_stamped_msg.header.frame_id = "base_link";

        // Copy the Twist data from the received message
        twist_stamped_msg.twist = *msg;

        // Publish the TwistStamped message
        cmd_vel_pub_->publish(twist_stamped_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char** argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a shared pointer to the node and spin
    rclcpp::spin(std::make_shared<CmdVelConverter>());

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
