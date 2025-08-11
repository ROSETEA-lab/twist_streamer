#include "twist_streamer/twist_streamer_class.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<twist_streamer_class>());
    } catch (const std::exception& e) {
        rclcpp::shutdown();
    }

    rclcpp::shutdown();

    return 0;
}
