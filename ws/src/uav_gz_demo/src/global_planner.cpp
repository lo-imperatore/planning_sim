#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node 
{
public:
    GlobalPlanner() : rclcpp::Node("global_planner"), 
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Starting Global Planner Node");
        // Parameters
        ns_         = this->declare_parameter<std::string>("ns", "X3");
        RCLCPP_INFO(this->get_logger(), "Namespace: %s", ns_.c_str());
        csv_path_   = this->declare_parameter<std::string>("csv_path", "drone_trajectory_rpy_rad.csv");
        RCLCPP_INFO(this->get_logger(), "CSV Path: %s", csv_path_.c_str());
        twist_topic_ = this->declare_parameter<std::string>("twist_topic", "/" + ns_ + "/twist");
        RCLCPP_INFO(this->get_logger(), "Twist Topic: %s", twist_topic_.c_str());
        enable_topic_ = this->declare_parameter<std::string>("enable_topic", "/" + ns_ + "/enable");
        RCLCPP_INFO(this->get_logger(), "Enable Topic: %s", enable_topic_.c_str());
        rate_hz_    = this->declare_parameter<double>("rate_hz", 50.0);
        RCLCPP_INFO(this->get_logger(), "Rate (Hz): %.2f", rate_hz_);
        
        // Publisher
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic_, rclcpp::QoS(10).reliable());
        RCLCPP_INFO(this->get_logger(), "Publisher created on topic: %s", twist_topic_.c_str());

        // Read the CSV file
        if (!readCSV()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read CSV file: %s", csv_path_.c_str());
            rclcpp::shutdown();
            return;
        }
        
        // Start the timer to publish commands
        auto dt = std::chrono::duration<double>(1.0 / rate_hz_);
        RCLCPP_INFO(this->get_logger(), "Timer set with period: %.2f ms", 
            dt.count());
        timer_ = this->create_wall_timer(
            dt,
            std::bind(&GlobalPlanner::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Global planner initialized with %zu waypoints", waypoints_.size());
    }

private:
    struct Waypoint {
        double x, y, z;
        double psi, phi, theta;
        double linear_vel_x, linear_vel_y, linear_vel_z;
        double angular_vel_x, angular_vel_y, angular_vel_z;
        int mode;
    };

    bool readCSV() {
        std::ifstream file(csv_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", csv_path_.c_str());
            return false;
        }

        std::string line;
        std::getline(file, line); // skip header line (x,y,z,qw,qx,qy,qz,vx,vy,vz)
        while (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string value;
        
            Waypoint waypoint;
            std::getline(ss, value, ','); waypoint.x = std::stod(value);
            std::getline(ss, value, ','); waypoint.y = std::stod(value);
            std::getline(ss, value, ','); waypoint.z = std::stod(value);
            std::getline(ss, value, ','); waypoint.psi = std::stod(value);
            std::getline(ss, value, ','); waypoint.phi = std::stod(value);
            std::getline(ss, value, ','); waypoint.theta = std::stod(value);
            std::getline(ss, value, ','); waypoint.linear_vel_x = std::stod(value);
            std::getline(ss, value, ','); waypoint.linear_vel_y = std::stod(value);
            std::getline(ss, value, ','); waypoint.linear_vel_z = std::stod(value);
            std::getline(ss, value, ','); waypoint.angular_vel_x = std::stod(value);
            std::getline(ss, value, ','); waypoint.angular_vel_y = std::stod(value);
            std::getline(ss, value, ','); waypoint.angular_vel_z = std::stod(value);
            // std::getline(ss, value, ','); waypoint.mode = std::stoi(value);
            waypoints_.push_back(waypoint);
        }

        file.close();
        return !waypoints_.empty();
    }

    void timerCallback() {
        if (current_waypoint_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "Reached end of waypoints");
            twist_pub_->publish(geometry_msgs::msg::Twist()); // Publish zero velocities
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing waypoints...");
        const Waypoint& wp = waypoints_[current_waypoint_];
        
        // Create and publish the twist message
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = wp.linear_vel_x;
        twist_msg.linear.y = wp.linear_vel_y;
        twist_msg.linear.z = wp.linear_vel_z;
        twist_msg.angular.x = wp.angular_vel_x;
        twist_msg.angular.y = wp.angular_vel_y;
        twist_msg.angular.z = wp.angular_vel_z;
        
        twist_pub_->publish(twist_msg);

        RCLCPP_INFO(this->get_logger(), "Published waypoint %zu: lin_vel=[%.2f, %.2f, %.2f], ang_vel=[%.2f, %.2f, %.2f], mode=%d",
                 current_waypoint_,
                 wp.linear_vel_x, wp.linear_vel_y, wp.linear_vel_z,
                 wp.angular_vel_x, wp.angular_vel_y, wp.angular_vel_z,
                 wp.mode);
        
        current_waypoint_++;
    }

    // Parameters and topics
    std::string ns_, csv_path_, twist_topic_, enable_topic_;
    double rate_hz_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Waypoints
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlanner>());
    rclcpp::shutdown();    
    return 0;
}