#include "joy_controller/joy_controller_component.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto component = std::make_shared<joy_controller::JoyControllerComponent>();
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}
