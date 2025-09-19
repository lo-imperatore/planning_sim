#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
using namespace std::chrono_literals;

class CmdVelPinger : public rclcpp::Node {
public:
  CmdVelPinger() : rclcpp::Node("cmdvel_pinger") {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/X3/gazebo/command/twist", 10);
    timer_ = create_wall_timer(500ms, [this](){
      geometry_msgs::msg::Twist m;
      m.linear.x = 0.2;   // forward
      m.linear.z = 0.1;   // up
      m.angular.z = 0.05;  // yaw
      RCLCPP_INFO(this->get_logger(), "publishing cmd_vel");
      pub_->publish(m);
    });
  }
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelPinger>());
  rclcpp::shutdown();
  return 0;
}

