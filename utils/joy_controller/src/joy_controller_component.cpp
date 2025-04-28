#include "joy_controller/joy_controller_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace joy_controller
{
    // constructor
    JoyControllerComponent::JoyControllerComponent(const rclcpp::NodeOptions & options)
    : Node("joy_controller", options)
    {
        // load parameters
        //// joy controller configuration
        declare_parameter("joy_top_left_button_idx", 4);
        get_parameter("joy_top_left_button_idx", joy_top_left_button_idx_);
        declare_parameter("joy_top_right_button_idx", 5);
        get_parameter("joy_top_right_button_idx", joy_top_right_button_idx_);
        declare_parameter("joy_left_stick_x_idx", 0);
        get_parameter("joy_left_stick_x_idx", joy_left_stick_x_idx_);
        declare_parameter("joy_left_stick_y_idx", 1);
        get_parameter("joy_left_stick_y_idx", joy_left_stick_y_idx_);
        declare_parameter("joy_right_stick_x_idx", 3);
        get_parameter("joy_right_stick_x_idx", joy_right_stick_x_idx_);
        declare_parameter("joy_right_stick_y_idx", 4);
        get_parameter("joy_right_stick_y_idx", joy_right_stick_y_idx_);
        declare_parameter("joy_autonomous_mode_idx", 0);
        get_parameter("joy_autonomous_mode_idx", joy_autonomous_mode_idx_);
        declare_parameter("joy_manual_mode_idx", 1);
        get_parameter("joy_manual_mode_idx", joy_manual_mode_idx_);

        mode_ = Mode::MANUAL;


        //// maximum speed settings
        declare_parameter("abs_max_linear_speed", 0.5);
        get_parameter("abs_max_linear_speed", abs_max_linear_speed_);
        declare_parameter("abs_max_angular_speed", 0.6);
        get_parameter("abs_max_angular_speed", abs_max_angular_speed_);

        //// input device
        declare_parameter("input_joy_device", "/dev/input/js0");
        get_parameter("input_joy_device", input_joy_device_);

        //// subscribing topic names
        std::string sub_topic_joy;
        declare_parameter("sub_topic.joy", "/joy");
        get_parameter("sub_topic.joy", sub_topic_joy);

        std::string sub_topic_autonomous_twist;
        declare_parameter("sub_topic.autonomous_cmd_vel", "/cmd_vel");
        get_parameter("sub_topic.autonomous_cmd_vel", sub_topic_autonomous_twist);

        //// publishing topic names
        std::string pub_topic_cmd_vel;
        declare_parameter("pub_topic.cmd_vel", "/cmd_vel");
        get_parameter("pub_topic.cmd_vel", pub_topic_cmd_vel);

        // create subscriber
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            sub_topic_joy,
            rclcpp::QoS(rclcpp::KeepLast(1)), // keep only the last message
            std::bind(&JoyControllerComponent::joyCallback, this, std::placeholders::_1)
        );

        autonomous_twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            sub_topic_autonomous_twist,
            rclcpp::QoS(rclcpp::KeepLast(1)), // keep only the last message
            std::bind(&JoyControllerComponent::twistCallback, this, std::placeholders::_1)
        );

        mode_pub_ = create_publisher<std_msgs::msg::String>(
            "mode",
            rclcpp::QoS(rclcpp::KeepLast(1))
        );

        // create publisher
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            pub_topic_cmd_vel, 
            rclcpp::QoS(rclcpp::KeepLast(1)) // keep only the last message
        );

        // create a timer to publish the mode
        auto timer_callback = [this]() -> void {
            auto mode_msg = std::make_unique<std_msgs::msg::String>();
            mode_msg->data = (mode_ == Mode::AUTONOMOUS) ? "AUTONOMOUS" : "MANUAL";
            mode_pub_->publish(std::move(mode_msg));
        };
        timer_ = create_wall_timer(std::chrono::duration<double>(0.1), timer_callback);

        js_checker_ = create_wall_timer(std::chrono::duration<double>(1.0), [this]() -> void {
            if (access(input_joy_device_.c_str(), F_OK) == -1)
            {
                mode_ = Mode::DISCONNECTED;
                RCLCPP_ERROR(get_logger(), "Joystick is not connected. Please connect the joystick and restart the node.");
            } else if (mode_ == Mode::DISCONNECTED){
                mode_ = Mode::MANUAL;
            }
        });

    }

    // destructor
    JoyControllerComponent::~JoyControllerComponent()
    {
        // No contents
    }

    void JoyControllerComponent::createTwistFromJoy(float v, float w, geometry_msgs::msg::Twist::UniquePtr & twist)
    {
        twist->linear.x = v;
        twist->linear.y = 0.0f;
        twist->linear.z = 0.0f;
        twist->angular.x = 0.0f;
        twist->angular.y = 0.0f;
        twist->angular.z = w;
    }


    // joy callback
    void JoyControllerComponent::joyCallback(const sensor_msgs::msg::Joy::UniquePtr msg)
    {
        float val_left_stick_y  = 0.0f;
        float val_right_stick_x = 0.0f;
        
        // change mode
        if (msg->buttons[joy_autonomous_mode_idx_] && mode_ == Mode::MANUAL)
        {
            auto twist = std::make_unique<geometry_msgs::msg::Twist>();
            createTwistFromJoy(0.0f, 0.0f, twist);
            cmd_vel_pub_->publish(std::move(twist));
            mode_ = Mode::AUTONOMOUS;
        }
        else if (msg->buttons[joy_manual_mode_idx_] && mode_ == Mode::AUTONOMOUS)
        {
            mode_ = Mode::MANUAL;
        }

        // if the mode is autonomous, do not publish the command
        if (mode_ == Mode::AUTONOMOUS) return;
        
        // enable the operation only if the top right button is pressed
        if (msg->buttons[joy_top_right_button_idx_])
        {
            // slow down the command if the top right button is pressed
            float scale = 1.0f;
            if (msg->buttons[joy_top_left_button_idx_]) // scale down the command (0.5x)
            {
                scale = scale * 0.5f;
            }

            // set forward / backword translation speed
            val_left_stick_y  = scale * msg->axes[joy_left_stick_y_idx_];

            // set left / right angular speed
            val_right_stick_x = scale * msg->axes[joy_right_stick_x_idx_];
        }
        else
        {
            // set forward / backword translation speed as 0.0
            val_left_stick_y  = 0.0f;

            // set left / right angular speed as 0.0
            val_right_stick_x = 0.0f;
        }

        // calculate Twist message to publish
        geometry_msgs::msg::Twist::UniquePtr cmd_vel(new geometry_msgs::msg::Twist);
        cmd_vel->linear.x =  abs_max_linear_speed_ * val_left_stick_y; // forward : positive, backward : negative
        cmd_vel->linear.y = 0.0f;
        cmd_vel->linear.z =  0.0f;
        cmd_vel->angular.x = 0.0f;
        cmd_vel->angular.y = 0.0f;
        cmd_vel->angular.z = abs_max_angular_speed_ * val_right_stick_x; // left    : positive, right    : negative

        // publish Twist message
        cmd_vel_pub_->publish(std::move(cmd_vel));
    }

    void JoyControllerComponent::twistCallback(const geometry_msgs::msg::Twist::UniquePtr msg)
    {
        if (mode_ == Mode::AUTONOMOUS)
        {
            cmd_vel_pub_->publish(std::move(*msg));
        }
    }

} // namespace joy_controller

RCLCPP_COMPONENTS_REGISTER_NODE(joy_controller::JoyControllerComponent)
