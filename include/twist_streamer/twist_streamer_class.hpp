#ifndef TWIST_STREAMER_CLASS_H_
#define TWIST_STREAMER_CLASS_H_

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define NAME_OF_THIS_NODE "twist_streamer"

using namespace std::chrono_literals;

class twist_streamer_class : public rclcpp::Node
{
public:

    twist_streamer_class();
    ~twist_streamer_class();

private:

    /* ROS topics */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVel_publisher_;

    /* Parameters from ROS parameter server */
    std::string csv_filename_;

    /* Node state variables */
    int run_period_;
    unsigned int csv_data_idx_;
    std::vector<std::vector<double>> csv_data_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* ROS topic callbacks */
    void timer_callback();

    /* Other functions */
    void read_csv(std::string filename, std::vector<std::vector<double>>& csv_data);
};

#endif /* TWIST_STREAMER_CLASS_H_ */
