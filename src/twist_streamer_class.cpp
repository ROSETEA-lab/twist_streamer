#include "twist_streamer/twist_streamer_class.hpp"

#include <fstream>
#include <iostream>

using std::placeholders::_1;

twist_streamer_class::twist_streamer_class()
        : Node(NAME_OF_THIS_NODE)
{
    /* Retrieve parameters from ROS parameter server */
    std::string parameter_name;

    // csv_filename
    parameter_name = "csv_filename";
    this->declare_parameter(parameter_name, "data.csv");
    if (false == this->get_parameter(parameter_name, csv_filename_))
        RCLCPP_ERROR(this->get_logger(), "Node %s: unable to retrieve parameter %s.", this->get_name(), parameter_name.c_str());

    // linVel_min
    parameter_name = "linVel_min";
    this->declare_parameter(parameter_name, 0.0);
    if (false == this->get_parameter(parameter_name, linVel_min_))
        RCLCPP_ERROR(this->get_logger(), "Node %s: unable to retrieve parameter %s.", this->get_name(), parameter_name.c_str());

    // linVel_max
    parameter_name = "linVel_max";
    this->declare_parameter(parameter_name, 0.0);
    if (false == this->get_parameter(parameter_name, linVel_max_))
        RCLCPP_ERROR(this->get_logger(), "Node %s: unable to retrieve parameter %s.", this->get_name(), parameter_name.c_str());

    // angVel_min
    parameter_name = "angVel_min";
    this->declare_parameter(parameter_name, 0.0);
    if (false == this->get_parameter(parameter_name, angVel_min_))
        RCLCPP_ERROR(this->get_logger(), "Node %s: unable to retrieve parameter %s.", this->get_name(), parameter_name.c_str());

    // angVel_max
    parameter_name = "angVel_max";
    this->declare_parameter(parameter_name, 0.0);
    if (false == this->get_parameter(parameter_name, angVel_max_))
        RCLCPP_ERROR(this->get_logger(), "Node %s: unable to retrieve parameter %s.", this->get_name(), parameter_name.c_str());

    /* ROS topics */
    cmdVel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    /* Initialize node state */
    csv_data_idx_ = 0;

    // Read data from csv file
    read_csv(csv_filename_, csv_data_);

    // Check time vector is equally spaced
    double delta_t = csv_data_.at(1).at(0)-csv_data_.at(0).at(0);
    for (unsigned int i=1; i<csv_data_.size(); i++) {
        if (std::fabs((csv_data_.at(i).at(0)-csv_data_.at(i-1).at(0))-delta_t) >= 1.0e-6) {
            RCLCPP_ERROR(this->get_logger(), "Node %s: the time vector in the CSV file is not equally spaced.", this->get_name());
        }
    }

    // Compute run_period and check it is an integer value
    run_period_ = round(delta_t*1000.0);
    if (std::fabs(run_period_-delta_t*1000.0)>=1.0e-6) {
        RCLCPP_ERROR(this->get_logger(), "Node %s: the time vector in the CSV file should be spaced by an integer period in milliseconds.", this->get_name());
    }

    /* Create node timer */
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(run_period_), std::bind(&twist_streamer_class::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node %s running periodically (T=%.2fs, f=%.2fHz).", this->get_name(), run_period_/1000.0, 1000.0/run_period_);
}

void twist_streamer_class::timer_callback()
{
    /* Publish one element of CSV data */
    auto twistMsg = geometry_msgs::msg::Twist();
    twistMsg.linear.x = twistMsg.linear.y = twistMsg.linear.z = 0.0;
    twistMsg.angular.x = twistMsg.angular.y = twistMsg.angular.z = 0.0;

    if (csv_data_idx_<csv_data_.size()) {
        twistMsg.linear.x = std::min(linVel_max_, std::max(linVel_min_, csv_data_.at(csv_data_idx_).at(1)));
        twistMsg.angular.z = std::min(angVel_max_, std::max(angVel_min_, csv_data_.at(csv_data_idx_).at(2)));
    }

    if (csv_data_idx_==csv_data_.size()) {
        RCLCPP_INFO(this->get_logger(), "Node %s: data streaming completed, set linear and angular velocity to zero.", this->get_name());
    }
    
    cmdVel_publisher_->publish(twistMsg);

    csv_data_idx_++;
}

twist_streamer_class::~twist_streamer_class()
{
    // Nothing to be done

    RCLCPP_INFO(this->get_logger(), "Node %s shutting down.", this->get_name());
}

void  twist_streamer_class::read_csv(std::string filename, std::vector<std::vector<double>>& csv_data)
{
    // Open an existing csv file
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Node %s: cannot open CSV file.", this->get_name());
    }

    // Read data from the file line by line
    std::string line;
    while (std::getline(fin, line)) {
        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Vector to store one row of data
        std::vector<double> csv_row;

        // Extract each double
        double val;
        while (ss >> val) {

            // Add the current value to the row
            csv_row.push_back(val);

            // If the next token is a comma, ignore it and move on
            if (ss.peek() == ',') {
                ss.ignore();
            }
        }

        // Store the extracted row
        csv_data.push_back(csv_row);
    }

    // Close file
    fin.close();
}
