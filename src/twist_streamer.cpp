#include "twist_streamer/twist_streamer_class.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<twist_streamer_class>());

    rclcpp::shutdown();

    return 0;
}
