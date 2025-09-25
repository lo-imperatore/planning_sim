#include <deque>
#include <cmath>
#include <string>
#include <optional>  // NEW

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class X3PositionController : public rclcpp::Node
{
public:
  X3PositionController()
  : rclcpp::Node("x3_position_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Parameters
    ns_ = this->declare_parameter<std::string>("ns", "X3");
    // world_frame_ = this->declare_parameter<std::string>("world_frame", "world");
    // body_frame_  = this->declare_parameter<std::string>("body_frame", ns_ + "/base_link");

    pos_tol_ = this->declare_parameter<double>("pos_tol", 0.10);
    yaw_tol_ = this->declare_parameter<double>("yaw_tol", 0.08);  // ~4.5 deg

    kp_xy_ = this->declare_parameter<double>("kp_xy", 1.0);
    kp_z_  = this->declare_parameter<double>("kp_z", 1.0);
    kp_yaw_= this->declare_parameter<double>("kp_yaw", 1.5);

    max_v_xy_ = this->declare_parameter<double>("max_v_xy", 1.0);
    max_v_z_  = this->declare_parameter<double>("max_v_z", 0.8);
    max_w_z_  = this->declare_parameter<double>("max_w_z", 1.0);

    double rate = this->declare_parameter<double>("rate", 50.0);

    // --- Topic params (NEW) ---
    // Gazebo plugin default is "cmd_vel"; commonly appears as /model/<ns>/cmd_vel
    // cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/model/" + ns_ + "/cmd_vel");
    twist_topic_ = this->declare_parameter<std::string>("twist_topic", "/" + ns_ + "/twist");
    enable_topic_ = this->declare_parameter<std::string>("enable_topic", "/" + ns_ + "/enable");
    setpoint_topic_ = this->declare_parameter<std::string>("setpoint_topic", "/" + ns_ + "/position_setpoint");
    path_topic_   = this->declare_parameter<std::string>("path_topic", "/" + ns_ + "/path");

    // Publishers
    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic_, rclcpp::QoS(10).reliable());
    // pub_enable_ = this->create_publisher<std_msgs::msg::Bool>(enable_topic_, rclcpp::QoS(1).reliable());

    // Subscribers
    sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      setpoint_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped & msg) {
        // queue_.clear();
        current_goal_ = msg;
        // RCLCPP_INFO(this->get_logger(), "Setpoint frame: %s", msg.header.frame_id.c_str());
        // RCLCPP_INFO(this->get_logger(), "Setpoint position: (%.2f, %.2f, %.2f)",
        //             msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
      });
    
    sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/model/" + ns_ + "/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped & msg) {
        if (std::strcmp(msg.header.frame_id.c_str(), "default") == 0) {
          // Save pose only if in world frame
          // RCLCPP_INFO(this->get_logger(), "Received robot pose in world frame.");
          current_pose_ = msg;
        }
      });

    // Arm/enable once on startup
    // std_msgs::msg::Bool en; en.data = true;
    // pub_enable_->publish(en);

    // Control loop timer
    using namespace std::chrono_literals;
    auto dt = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(dt),
      std::bind(&X3PositionController::tick, this));

    RCLCPP_INFO(get_logger(),
      "X3PositionController started.\n  cmd  : %s\n  enable: %s\n  setpoint: %s\n  path: %s\n  frames: world=%s, body=%s",
      twist_topic_.c_str(), enable_topic_.c_str(), setpoint_topic_.c_str(), path_topic_.c_str(),
      world_frame_.c_str(), body_frame_.c_str());
  }

private:
  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    tf2::Quaternion tq;
    tf2::fromMsg(q, tq);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    return yaw;
  }

  void advance_goal_()
  {
    if (!queue_.empty()) {
      geometry_msgs::msg::Pose p = queue_.front(); queue_.pop_front();
      current_goal_.emplace();
      current_goal_->header.frame_id = world_frame_;
      current_goal_->pose = p;
      RCLCPP_INFO(get_logger(), "Advancing to next waypoint (%zu left).", queue_.size());
    } else {
      current_goal_.reset();
      RCLCPP_INFO(get_logger(), "Path complete.");
      pub_cmd_->publish(geometry_msgs::msg::Twist()); // stop
    }
  }

  void tick()
  {
    if (!current_goal_) return;

    // geometry_msgs::msg::TransformStamped tf;
    // try {
    //   tf = tf_buffer_.lookupTransform(world_frame_, body_frame_, tf2::TimePointZero);
    // } catch (const tf2::TransformException & ex) {
    //   RCLCPP_INFO(get_logger(), "Waiting for TF %s → %s...",
    //     world_frame_.c_str(), body_frame_.c_str());
    //   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
    //   return;
    // }

    if (!current_pose_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for robot pose in world frame...");
      return;
    }
    // Use current_pose_ instead of TF lookup
    const auto & robot_pose = current_pose_->pose;
    const auto & T = robot_pose.position;
    const auto & R = robot_pose.orientation;
    const double yaw = yawFromQuat(R);

    // Goal pose
    const auto & g = current_goal_->pose;
    const double gx = g.position.x, gy = g.position.y, gz = g.position.z;
    const double gyaw = yawFromQuat(g.orientation);

    // Errors (world frame)
    double ex = gx - T.x;
    double ey = gy - T.y;
    double ez = gz - T.z;
    double eyaw = std::atan2(std::sin(gyaw - yaw), std::cos(gyaw - yaw));

    const double pos_err = std::sqrt(ex*ex + ey*ey + ez*ez);
    if (pos_err < pos_tol_ && std::fabs(eyaw) < yaw_tol_) {
      pub_cmd_->publish(geometry_msgs::msg::Twist()); // stop
      return;
    }

    // P controller -> velocity setpoints (still world frame here)
    double vx_w = kp_xy_ * ex;
    double vy_w = kp_xy_ * ey;
    double vz   = kp_z_  * ez;
    double wz   = kp_yaw_ * eyaw;

    // --- Convert to body frame (REQUIRED by MulticopterVelocityControl) ---
    // v_body = Rz(-yaw) * v_world
    double cos_y = std::cos(yaw), sin_y = std::sin(yaw);
    double vx_b =  cos_y * vx_w + sin_y * vy_w;
    double vy_b = -sin_y * vx_w + cos_y * vy_w;

    // Saturation (body frame)
    double vxy = std::hypot(vx_b, vy_b);
    if (vxy > max_v_xy_) {
      double s = max_v_xy_ / (vxy + 1e-9);
      vx_b *= s; vy_b *= s;
    }
    if (vz >  max_v_z_) vz =  max_v_z_;
    if (vz < -max_v_z_) vz = -max_v_z_;
    if (wz >  max_w_z_) wz =  max_w_z_;
    if (wz < -max_w_z_) wz = -max_w_z_;

    // Publish (body linear vel + yaw rate)
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = vx_b;
    cmd.linear.y  = vy_b;
    cmd.linear.z  = vz;
    cmd.angular.z = wz;
    RCLCPP_INFO(get_logger(), "cmd: vx=%.2f vy=%.2f vz=%.2f wz=%.2f", vx_b, vy_b, vz, wz);
    pub_cmd_->publish(cmd);

  }

  // Params / topics
  std::string ns_, world_frame_, body_frame_;
  std::string twist_topic_, enable_topic_, setpoint_topic_, path_topic_;

  // Gains/limits
  double kp_xy_, kp_z_, kp_yaw_;
  double max_v_xy_, max_v_z_, max_w_z_;
  double pos_tol_, yaw_tol_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Goal/path
  std::optional<geometry_msgs::msg::PoseStamped> current_goal_, current_pose_;
  std::deque<geometry_msgs::msg::Pose> queue_;

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_, sub_robot_pose_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<X3PositionController>());
  rclcpp::shutdown();
  return 0;
}
