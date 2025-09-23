#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cctype>

struct Waypoint {
  double t;   // seconds
  double x, y, z;
  double yaw_rad;
  double roll_rad;
  double pitch_rad;
};

class TrajectoryStreamer : public rclcpp::Node {
public:
  TrajectoryStreamer() : rclcpp::Node("trajectory_streamer") {
    csv_path_   = declare_parameter<std::string>("csv_path", "drone_trajectory_rpy_rad.csv");
    topic_name_ = declare_parameter<std::string>("topic_name", "/drone/position_setpoint");
    frame_id_   = declare_parameter<std::string>("frame_id", "map");
    rate_hz_    = declare_parameter<double>("rate_hz", 50.0);

    // New diagnostics/timing params
    use_steady_clock_ = declare_parameter<bool>("use_steady_clock", true);
    print_every_s_    = declare_parameter<double>("print_every_s", 1.0);
    min_subscribers_  = declare_parameter<int>("min_subscribers", 0);

    // Optional takeoff prepend
    takeoff_prepend_    = declare_parameter<bool>("takeoff_prepend", false);
    takeoff_height_     = declare_parameter<double>("takeoff_height", 2.0);
    takeoff_duration_s_ = declare_parameter<double>("takeoff_duration_s", 3.0);
    takeoff_start_z0_   = declare_parameter<double>("takeoff_start_z0", 0.0);
    force_zero_rp_      = declare_parameter<bool>("force_zero_roll_pitch", true);

    if (!loadCsv(csv_path_, wps_) || wps_.empty()) {
      RCLCPP_FATAL(get_logger(), "Failed to load CSV: %s", csv_path_.c_str());
      rclcpp::shutdown();
      return;
    }
    std::sort(wps_.begin(), wps_.end(),
      [](const Waypoint&a,const Waypoint&b){return a.t<b.t;});
    wps_.erase(std::unique(wps_.begin(), wps_.end(),
      [](const Waypoint&a,const Waypoint&b){return a.t==b.t;}), wps_.end());

    if (takeoff_prepend_) {
      Waypoint f = wps_.front();
      std::vector<Waypoint> aug;
      aug.push_back(Waypoint{0.0, takeoff_prepend_keep_x_ ? f.x : f.x,
                                   takeoff_prepend_keep_y_ ? f.y : f.y,
                                   takeoff_start_z0_, f.yaw_rad, 0.0, 0.0});
      aug.push_back(Waypoint{takeoff_duration_s_, f.x, f.y, takeoff_height_, f.yaw_rad, 0.0, 0.0});
      for (auto w : wps_) { w.t += takeoff_duration_s_; aug.push_back(w); }
      wps_.swap(aug);
    }

    // Summary logs
    RCLCPP_INFO(get_logger(),
      "Loaded %zu waypoints (t0=%.3f, t1=%.3f) from %s",
      wps_.size(), wps_.front().t, wps_.back().t, csv_path_.c_str());
    for (size_t i = 0; i < std::min<size_t>(wps_.size(), 3); ++i) {
      const auto &w = wps_[i];
      RCLCPP_INFO(get_logger(), "  wp[%zu]: t=%.2f pos(%.2f,%.2f,%.2f) yaw=%.3f rad",
                  i, w.t, w.x, w.y, w.z, w.yaw_rad);
    }

    // Timing bases
    start_ros_time_  = now();
    start_wall_time_ = std::chrono::steady_clock::now();
    last_print_wall_time_ = start_wall_time_;
    done_logged_ = false;

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz_)),
      std::bind(&TrajectoryStreamer::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "Streaming on %s @ %.1f Hz (frame_id=%s, use_steady_clock=%s, print_every=%.2fs).",
      topic_name_.c_str(), rate_hz_, frame_id_.c_str(),
      use_steady_clock_ ? "true" : "false", print_every_s_);
  }

private:
  static bool loadCsv(const std::string& path, std::vector<Waypoint>& out) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::string line; bool header_checked=false;
    while (std::getline(f, line)) {
      if (line.empty() || line[0]=='#') continue;
      if (!header_checked) {
        header_checked = true;
        bool has_alpha = std::any_of(line.begin(), line.end(),
                          [](char c){return std::isalpha(static_cast<unsigned char>(c));});
        if (has_alpha) continue; // skip header
      }
      std::stringstream ss(line); std::string tok; std::vector<double> v;
      while (std::getline(ss, tok, ',')) {
        tok.erase(0, tok.find_first_not_of(" \t\r\n"));
        tok.erase(tok.find_last_not_of(" \t\r\n")+1);
        if (tok.empty()) continue;
        try { v.push_back(std::stod(tok)); } catch (...) { v.clear(); break; }
      }
      // Accept either 5 cols (t,x,y,z,yaw_rad) or 7 (… ,roll_rad,pitch_rad)
      if (v.size()==5 || v.size()==7) {
        double roll = (v.size()==7)? v[5] : 0.0;
        double pitch= (v.size()==7)? v[6] : 0.0;
        out.push_back(Waypoint{v[0], v[1], v[2], v[3], v[4], roll, pitch});
      }
    }
    return !out.empty();
  }

  static double interpAngle(double a0, double a1, double alpha) {
    double d = std::atan2(std::sin(a1-a0), std::cos(a1-a0));
    return a0 + alpha*d;
  }

  Waypoint sampleAt(double tnow) {
    if (tnow <= wps_.front().t) return wps_.front();
    if (tnow >= wps_.back().t)  return wps_.back();
    auto it = std::upper_bound(wps_.begin(), wps_.end(), tnow,
      [](double t, const Waypoint& w){return t < w.t;});
    size_t i1 = std::distance(wps_.begin(), it), i0 = i1-1;
    const Waypoint &a = wps_[i0], &b = wps_[i1];
    double alpha = (tnow - a.t) / std::max(1e-9, (b.t - a.t));
    Waypoint o;
    o.t = tnow;
    o.x = a.x + alpha*(b.x-a.x);
    o.y = a.y + alpha*(b.y-a.y);
    o.z = a.z + alpha*(b.z-a.z);
    o.yaw_rad   = interpAngle(a.yaw_rad,   b.yaw_rad,   alpha);
    o.roll_rad  = 0.0; // force zero
    o.pitch_rad = 0.0; // force zero
    return o;
  }

  double elapsedTrajectorySeconds() {
    if (use_steady_clock_) {
      auto now_wall = std::chrono::steady_clock::now();
      std::chrono::duration<double> d = now_wall - start_wall_time_;
      return d.count();
    } else {
      return (now() - start_ros_time_).seconds();
    }
  }

  void maybePrint(const Waypoint& w, double tnow) {
    auto now_wall = std::chrono::steady_clock::now();
    std::chrono::duration<double> d = now_wall - last_print_wall_time_;
    if (d.count() >= print_every_s_) {
      last_print_wall_time_ = now_wall;
      size_t subs = pub_->get_subscription_count();
      RCLCPP_INFO(get_logger(),
        "traj t=%.2f  setpoint: x=%.2f y=%.2f z=%.2f yaw=%.3f  subs=%zu",
        tnow, w.x, w.y, w.z, w.yaw_rad, subs);
      if (subs < static_cast<size_t>(min_subscribers_)) {
        RCLCPP_WARN(get_logger(),
          "Subscribers on %s: %zu (min required: %d). Is the controller subscribed?",
          topic_name_.c_str(), subs, min_subscribers_);
      }
    }
  }

  void onTimer() {
    double t = elapsedTrajectorySeconds();
    Waypoint w = sampleAt(t);

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();         // stamp with ROS time
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = w.x;
    msg.pose.position.y = w.y;
    msg.pose.position.z = w.z;

    tf2::Quaternion q;
    const double roll  = force_zero_rp_ ? 0.0 : w.roll_rad;
    const double pitch = force_zero_rp_ ? 0.0 : w.pitch_rad;
    q.setRPY(roll, pitch, w.yaw_rad);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    pub_->publish(msg);
    // maybePrint(w, t);

    if (!done_logged_ && t >= wps_.back().t) {
      done_logged_ = true;
      RCLCPP_INFO(get_logger(),
        "Reached final waypoint (t=%.2f). Holding last pose.", wps_.back().t);
    }
  }

  // params
  std::string csv_path_, topic_name_, frame_id_;
  double rate_hz_;
  bool takeoff_prepend_;
  double takeoff_height_, takeoff_duration_s_, takeoff_start_z0_;
  bool force_zero_rp_;
  bool use_steady_clock_;
  double print_every_s_;
  int min_subscribers_{0};

  // state
  std::vector<Waypoint> wps_;
  rclcpp::Time start_ros_time_;
  std::chrono::steady_clock::time_point start_wall_time_;
  std::chrono::steady_clock::time_point last_print_wall_time_;
  bool done_logged_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;

  // placeholders to quiet potential warnings if you later add options
  bool takeoff_prepend_keep_x_{true};
  bool takeoff_prepend_keep_y_{true};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryStreamer>());
  rclcpp::shutdown();
  return 0;
}
