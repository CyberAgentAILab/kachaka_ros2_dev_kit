#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>


namespace joy_controller
{
class JoyControllerComponent : public rclcpp::Node
{
public:
    // constructor supporting zero copy as default
    explicit JoyControllerComponent(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions().use_intra_process_comms(true)
    );

    // destructor
    ~JoyControllerComponent();

private:
    // subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    void joyCallback(const sensor_msgs::msg::Joy::UniquePtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr autonomous_twist_sub_;
    void twistCallback(const geometry_msgs::msg::Twist::UniquePtr msg);

    // publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr js_checker_;

    // util functions
    void createTwistFromJoy(
        float v,
        float w,
        geometry_msgs::msg::Twist::UniquePtr & twist
    );

    // mode
    enum class Mode
    {
        MANUAL,
        AUTONOMOUS,
        DISCONNECTED,
    };

    Mode mode_;

    // parameters
    int joy_top_left_button_idx_;
    int joy_top_right_button_idx_;
    int joy_left_stick_x_idx_;
    int joy_left_stick_y_idx_;
    int joy_right_stick_x_idx_;
    int joy_right_stick_y_idx_;
    int joy_autonomous_mode_idx_;
    int joy_manual_mode_idx_;
    float abs_max_linear_speed_;
    float abs_max_angular_speed_;

    std::string input_joy_device_;


}; // class JoyControllerComponent
} // namespace joy_controller
