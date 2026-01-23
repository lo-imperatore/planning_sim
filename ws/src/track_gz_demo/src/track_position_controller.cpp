#include <cmath>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

class TrackPositionController : public rclcpp::Node
{
public:
  TrackPositionController()
  : rclcpp::Node("track_position_controller")
  {
    model_ = declare_parameter<std::string>("model", "MYBOT");

    // Gazebo topics (match TrackController defaults)
    left_cmd_topic_  = declare_parameter<std::string>(
      "left_cmd_topic",  "/model/" + model_ + "/link/left_track/track_cmd_vel");
    right_cmd_topic_ = declare_parameter<std::string>(
      "right_cmd_topic", "/model/" + model_ + "/link/right_track/track_cmd_vel");

    pose_topic_ = declare_parameter<std::string>(
      "pose_topic", "/model/" + model_ + "/pose");
    setpoint_topic_ = declare_parameter<std::string>(
      "setpoint_topic", "/" + model_ + "/position_setpoint");

    // Controller params
    pos_tol_ = declare_parameter<double>("pos_tol", 0.10);
    yaw_tol_ = declare_parameter<double>("yaw_tol", 0.08);

    kv_ = declare_parameter<double>("kv", 1.0);
    kw_ = declare_parameter<double>("kw", 2.0);

    max_v_ = declare_parameter<double>("max_v", 1.2);
    max_w_ = declare_parameter<double>("max_w", 1.5);

    track_sep_ = declare_parameter<double>("track_separation", 0.5); // meters

    double rate = declare_parameter<double>("rate", 50.0);

    pub_left_  = create_publisher<std_msgs::msg::Float64>(left_cmd_topic_, 10);
    pub_right_ = create_publisher<std_msgs::msg::Float64>(right_cmd_topic_, 10);

    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      setpoint_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped & msg) { goal_ = msg; });

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped & msg) { pose_ = msg; });

    auto dt = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(dt),
      std::bind(&TrackPositionController::tick, this));
  }

private:
  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    // Minimal yaw extraction without tf2 (ok if quaternion is valid)
    // yaw = atan2(2(wz+xy), 1-2(y^2+z^2))
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    return std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
  }

  static double wrapAngle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  void publishTracks(double v_left, double v_right)
  {
    std_msgs::msg::Float64 ml, mr;
    ml.data = v_left;
    mr.data = v_right;
    pub_left_->publish(ml);
    pub_right_->publish(mr);
  }

  void tick()
  {
    if (!goal_ || !pose_) return;

    const auto & P = pose_->pose.position;
    const auto & Q = pose_->pose.orientation;
    const double yaw = yawFromQuat(Q);

    const auto & G = goal_->pose.position;
    const double gyaw = yawFromQuat(goal_->pose.orientation);

    // World-frame planar error
    const double ex = G.x - P.x;
    const double ey = G.y - P.y;
    const double eyaw = wrapAngle(gyaw - yaw);

    const double pos_err = std::hypot(ex, ey);
    if (pos_err < pos_tol_ && std::fabs(eyaw) < yaw_tol_) {
      publishTracks(0.0, 0.0);
      return;
    }

    // Convert planar error to body frame (Rz(-yaw))
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double ex_b =  c*ex + s*ey;   // forward error
    const double ey_b = -s*ex + c*ey;   // left error

    // Simple controller in body frame
    double v = kv_ * ex_b;
    double w = kw_ * std::atan2(ey_b, std::max(0.05, ex_b));  // heading-to-goal

    v = clamp(v, -max_v_, max_v_);
    w = clamp(w, -max_w_, max_w_);

    // Differential track mapping
    const double half = 0.5 * track_sep_;
    double v_left  = v - w * half;
    double v_right = v + w * half;

    // Optional: clamp to your plugin limits (you set ±1.2)
    v_left  = clamp(v_left,  -max_v_, max_v_);
    v_right = clamp(v_right, -max_v_, max_v_);

    publishTracks(v_left, v_right);
  }

  std::string model_;
  std::string left_cmd_topic_, right_cmd_topic_, pose_topic_, setpoint_topic_;

  double pos_tol_{0.1}, yaw_tol_{0.08};
  double kv_{1.0}, kw_{2.0};
  double max_v_{1.2}, max_w_{1.5};
  double track_sep_{0.5};

  std::optional<geometry_msgs::msg::PoseStamped> goal_, pose_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_, pub_right_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_, sub_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackPositionController>());
  rclcpp::shutdown();
  return 0;
}
