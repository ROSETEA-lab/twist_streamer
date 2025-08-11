#ifndef TWIST_STREAMER_CLASS_H_
#define TWIST_STREAMER_CLASS_H_

#include <chrono>
#include <functional>
#include <memory>
#include <interpolation.h>

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
    double t_end_;
    double linVel_min_, linVel_max_, angVel_min_, angVel_max_, publish_frequency_;

    /* Node state variables */
    unsigned int run_period_;
    unsigned int time_idx_;
    alglib::spline1dinterpolant v_spline_, omega_spline_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* ROS topic callbacks */
    void timer_callback();

    /* Other functions */
    bool read_csv(std::string filename, std::vector<std::vector<double>>& csv_data);
    bool write_csv(std::string filename, std::vector<std::vector<double>> csv_data);
};

#endif /* TWIST_STREAMER_CLASS_H_ */
