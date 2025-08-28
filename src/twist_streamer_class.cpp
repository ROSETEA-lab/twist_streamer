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

    // publish_frequency
    parameter_name = "publish_frequency";
    this->declare_parameter(parameter_name, 0.01);
    if (false == this->get_parameter(parameter_name, publish_frequency_))
        RCLCPP_ERROR(this->get_logger(), "Node %s: unable to retrieve parameter %s.", this->get_name(), parameter_name.c_str());

    /* ROS topics */
    cmdVel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    /* Initialize node state */
    time_idx_ = 0;

    // Read velocities from csv file
    std::vector<std::vector<double>> csv_data;
    if (!read_csv(csv_filename_, csv_data)) {
        RCLCPP_FATAL(this->get_logger(), "Node %s: cannot open CSV file.", this->get_name());

        throw std::runtime_error("CSV file does not exist.");
		return;
	}

    // Interpolate velocities with linear spline
    alglib::real_1d_array time, v, omega;

    time.setlength(csv_data.size());
    v.setlength(csv_data.size());
    omega.setlength(csv_data.size());

    for (unsigned int i=0; i<csv_data.size(); i++) {
        time[i] = csv_data.at(i).at(0)-csv_data.at(0).at(0);
        v[i] = csv_data.at(i).at(1);
        omega[i] = csv_data.at(i).at(2);
    }
	t_end_ = time[time.length()-1];

    spline1dbuildlinear(time, v, v_spline_);
    spline1dbuildlinear(time, omega, omega_spline_);

    /* Create node timer */
    run_period_ = round(1000.0/publish_frequency_);
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

    if (time_idx_*run_period_<=round(t_end_*1000.0)) {
        twistMsg.linear.x = std::min(linVel_max_, std::max(linVel_min_, spline1dcalc(v_spline_, time_idx_/publish_frequency_)));
        twistMsg.angular.z = std::min(angVel_max_, std::max(angVel_min_, spline1dcalc(omega_spline_, time_idx_/publish_frequency_)));
    }
	else {
        RCLCPP_INFO(this->get_logger(), "Node %s: data streaming completed, set linear and angular velocity to zero.", this->get_name());
    }
    
    cmdVel_publisher_->publish(twistMsg);

	// Update time index
    time_idx_++;
}

twist_streamer_class::~twist_streamer_class()
{
    // Nothing to be done

    RCLCPP_INFO(this->get_logger(), "Node %s shutting down.", this->get_name());
}

bool twist_streamer_class::read_csv(std::string filename, std::vector<std::vector<double>>& csv_data)
{
    // Open an existing csv file
    std::ifstream fin(filename);
    if (!fin.is_open()) {
		return false;
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

	return true;
}

bool twist_streamer_class::write_csv(std::string filename, std::vector<std::vector<double>> csv_data)
{
    // Open an existing csv file
    std::ofstream fout(filename);
    if (!fout.is_open()) {
		return false;
    }

    // Write data to the file line by line
    for (unsigned int i=0; i<csv_data.size(); i++) {
        for (unsigned int j=0; j<csv_data.at(i).size()-1; j++) {
            fout << csv_data.at(i).at(j) << ",";
        }
        fout << csv_data.at(i).at(csv_data.at(i).size()-1) << std::endl;
    }

    // Close file
    fout.close();

	return true;
}
