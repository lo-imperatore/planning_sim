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
#include <limits>
#include <stdexcept>

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
    csv_path_   = declare_parameter<std::string>("csv_path", "traj_drone_hybrid_2.csv");
    topic_name_ = declare_parameter<std::string>("topic_name", "/drone/position_setpoint");
    frame_id_   = declare_parameter<std::string>("frame_id", "map");
    rate_hz_    = declare_parameter<double>("rate_hz", 50.0);

    // CSV/angles handling
    csv_has_time_       = declare_parameter<bool>("csv_has_time", false);
    csv_dt_             = declare_parameter<double>("csv_dt", 1.0 / rate_hz_);
    angles_in_degrees_  = declare_parameter<bool>("angles_in_degrees", false);

    // Diagnostics/timing
    use_steady_clock_ = declare_parameter<bool>("use_steady_clock", true);
    print_every_s_    = declare_parameter<double>("print_every_s", 1.0);
    min_subscribers_  = declare_parameter<int>("min_subscribers", 0);

    // Optional takeoff prepend
    takeoff_prepend_    = declare_parameter<bool>("takeoff_prepend", false);
    takeoff_height_     = declare_parameter<double>("takeoff_height", 2.0);
    takeoff_duration_s_ = declare_parameter<double>("takeoff_duration_s", 3.0);
    takeoff_start_z0_   = declare_parameter<double>("takeoff_start_z0", 0.0);
    force_zero_rp_      = declare_parameter<bool>("force_zero_roll_pitch", true);

    std::string why;
    if (!loadCsvFlexible(csv_path_, csv_has_time_, csv_dt_, angles_in_degrees_, wps_, why) || wps_.empty()) {
      RCLCPP_FATAL(get_logger(), "Failed to load/parse CSV (%s): %s", why.c_str(), csv_path_.c_str());
      throw std::runtime_error("CSV load failed");
    }

    std::sort(wps_.begin(), wps_.end(),
      [](const Waypoint&a,const Waypoint&b){return a.t<b.t;});
    wps_.erase(std::unique(wps_.begin(), wps_.end(),
      [](const Waypoint&a,const Waypoint&b){return a.t==b.t;}), wps_.end());

    if (takeoff_prepend_) {
      Waypoint f = wps_.front();
      std::vector<Waypoint> aug;
      aug.push_back(Waypoint{0.0, f.x, f.y, takeoff_start_z0_, f.yaw_rad, 0.0, 0.0});
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
  // ---------- CSV helpers ----------
  static std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> v; std::string tok; std::stringstream ss(s);
    while (std::getline(ss, tok, delim)) {
      auto b = tok.find_first_not_of(" \t\r\n");
      auto e = tok.find_last_not_of(" \t\r\n");
      v.push_back(b==std::string::npos ? "" : tok.substr(b, e-b+1));
    }
    return v;
  }

  static std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
  }

  static int find_col(const std::vector<std::string>& cols,
                      std::initializer_list<const char*> names) {
    for (size_t i = 0; i < cols.size(); ++i) {
      auto c = lower(cols[i]);
      for (auto n : names) { if (c == n) return static_cast<int>(i); }
    }
    return -1;
  }

  static bool loadCsvFlexible(const std::string& path,
                              bool csv_has_time, double csv_dt,
                              bool angles_in_degrees,
                              std::vector<Waypoint>& out,
                              std::string& why) {
    std::ifstream f(path);
    if (!f.is_open()) { why = "cannot open file"; return false; }

    // Find the header line (skip comments/empties)
    std::string header;
    while (std::getline(f, header)) {
      if (!header.empty() && header[0] != '#') break;
    }
    if (header.empty()) { why = "empty file / missing header"; return false; }

    // delimiter detection
    auto cnt = [&](char c){ return std::count(header.begin(), header.end(), c); };
    char delim = ','; if (cnt(';') > cnt(',')) delim = ';';
    if (cnt('\t') > cnt(delim)) delim = '\t';

    auto cols = split(header, delim);
    int ix = find_col(cols, {"x"});
    int iy = find_col(cols, {"y"});
    int iz = find_col(cols, {"z"});
    int iyaw = find_col(cols, {"yaw","psi","psi_rad"});
    int iroll = find_col(cols, {"roll","phi"});
    int ipitch= find_col(cols, {"pitch","theta"});
    int it = find_col(cols, {"t","time","sec","secs","seconds"});

    if (ix<0 || iy<0 || iz<0 || iyaw<0) { why = "missing x/y/z/yaw(psi) columns"; return false; }
    if (it >= 0) csv_has_time = true;

    const double to_rad = angles_in_degrees ? (M_PI / 180.0) : 1.0;

    std::string line; size_t row = 0; size_t kept = 0, skipped = 0;
    while (std::getline(f, line)) {
      if (line.empty() || line[0]=='#') continue;
      auto toks = split(line, delim);

      auto getd = [&](int i)->double {
        if (i < 0 || i >= static_cast<int>(toks.size())) return std::numeric_limits<double>::quiet_NaN();
        try { return std::stod(toks[i]); } catch (...) { return std::numeric_limits<double>::quiet_NaN(); }
      };

      double t     = csv_has_time ? getd(it) : (row * csv_dt);
      double x     = getd(ix);
      double y     = getd(iy);
      double z     = getd(iz);
      double yaw   = getd(iyaw) * to_rad;
      double roll  = (iroll >= 0 ? getd(iroll)  : 0.0) * to_rad;
      double pitch = (ipitch>= 0 ? getd(ipitch) : 0.0) * to_rad;

      if (std::isnan(t) || std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(yaw)) { ++skipped; ++row; continue; }

      out.push_back(Waypoint{t,x,y,z,yaw,roll,pitch});
      ++kept; ++row;
    }

    if (out.empty()) { why = "parsed 0 valid rows"; return false; }
    why = "ok";
    return true;
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
    o.roll_rad  = 0.0; // default to zero; see force_zero_rp_ below
    o.pitch_rad = 0.0;
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

  void onTimer() {
    double t = elapsedTrajectorySeconds();
    Waypoint w = sampleAt(t);

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
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

  // CSV handling
  bool csv_has_time_;
  double csv_dt_;
  bool angles_in_degrees_;

  // state
  std::vector<Waypoint> wps_;
  rclcpp::Time start_ros_time_;
  std::chrono::steady_clock::time_point start_wall_time_;
  std::chrono::steady_clock::time_point last_print_wall_time_;
  bool done_logged_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;

  // placeholders
  bool takeoff_prepend_keep_x_{true};
  bool takeoff_prepend_keep_y_{true};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TrajectoryStreamer>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_streamer"), "Initialization failed: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
