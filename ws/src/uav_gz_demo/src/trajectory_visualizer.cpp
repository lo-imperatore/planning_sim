#include <rclcpp/rclcpp.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <chrono>
#include <thread>

namespace GM = gz::msgs;

// ---------- helpers ----------
static void set_color(GM::Marker &m, float r,float g,float b,float a)
{
  auto *diff = m.mutable_material()->mutable_diffuse();
  diff->set_r(r); diff->set_g(g); diff->set_b(b); diff->set_a(a);
  auto *amb  = m.mutable_material()->mutable_ambient();
  amb->set_r(r); amb->set_g(g); amb->set_b(b); amb->set_a(a);
}

static bool wait_for_marker_service(gz::transport::Node &node,
                                    const std::string &svc,
                                    int timeout_ms_total = 8000)
{
  const int step = 200; // ms
  int waited = 0;
  while (waited < timeout_ms_total) {
    std::vector<gz::transport::ServicePublisher> pubs;
    if (node.ServiceInfo(svc, pubs) && !pubs.empty())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(step));
    waited += step;
  }
  return false;
}

// Templated request so we can try Empty OR Boolean easily.
template <typename ReplyT>
static bool request_marker(gz::transport::Node &node,
                           const std::string &svc,
                           const GM::Marker &m,
                           int timeout_ms,
                           std::string &err)
{
  ReplyT rep;
  bool result = false;
  const bool ok = node.Request(svc, m, timeout_ms, rep, result);
  if (!ok)  { err = "transport request failed (timeout / not available)"; return false; }
  if (!result) { err = "handler returned failure"; return false; }
  return true;
}

// ---------- node ----------
class TrajToMarkers : public rclcpp::Node {
public:
  TrajToMarkers(): Node("traj_to_gz_markers")
  {
    // params
    declare_parameter<std::string>("csv_path", "");
    declare_parameter<std::string>("marker_service", "/marker");
    declare_parameter<double>("line_width", 0.20);
    declare_parameter<double>("point_size", 0.30);
    declare_parameter<std::vector<double>>("traj_color", {1.0,0.2,0.2,0.9});
    declare_parameter<bool>("show_waypoints", true);

    csv_   = get_parameter("csv_path").as_string();
    svc_   = get_parameter("marker_service").as_string();
    lw_    = get_parameter("line_width").as_double();
    ps_    = get_parameter("point_size").as_double();
    color_ = get_parameter("traj_color").as_double_array();
    show_pts_ = get_parameter("show_waypoints").as_bool();

    if (csv_.empty()) {
      RCLCPP_FATAL(get_logger(), "csv_path parameter is empty");
      return;
    }

    load_points(csv_);

    if (pts_.empty()) {
      RCLCPP_FATAL(get_logger(), "No valid points parsed from CSV: %s", csv_.c_str());
      return;
    }

    if (!wait_for_marker_service(node_, svc_, 10000)) {
      RCLCPP_FATAL(get_logger(), "Marker service %s not found; is the GUI plugin loaded?", svc_.c_str());
      return;
    }

    publish_markers_once();
  }

private:
  void load_points(const std::string &path)
  {
    std::ifstream f(path);
    if (!f) {
      RCLCPP_FATAL(get_logger(), "Cannot open CSV: %s", path.c_str());
      return;
    }

    std::string line;
    bool header = true;
    size_t n_bad = 0;

    while (std::getline(f, line)) {
      if (line.empty()) continue;
      if (header) { header = false; continue; }  // skip header

      std::vector<std::string> tok;
      tok.reserve(8);
      std::stringstream ss(line);
      std::string t;
      while (std::getline(ss, t, ',')) {
        // trim spaces
        size_t a = t.find_first_not_of(" \t");
        size_t b = t.find_last_not_of(" \t");
        if (a == std::string::npos) tok.emplace_back("");
        else tok.emplace_back(t.substr(a, b - a + 1));
      }

      if (tok.size() < 3) { n_bad++; continue; }

      try {
        double x = std::stod(tok[0]);
        double y = std::stod(tok[1]);
        double z = std::stod(tok[2]);
        pts_.push_back({x,y,z});
      } catch (...) {
        n_bad++;
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints (skipped %zu) from %s",
                pts_.size(), n_bad, path.c_str());
  }

  void publish_markers_once()
  {
    // Trajectory line
    GM::Marker traj;
    traj.set_id(1001);
    traj.set_action(GM::Marker::ADD_MODIFY);
    traj.set_type(GM::Marker::LINE_STRIP);
    traj.mutable_scale()->set_x(lw_);       // thickness in X for lines
    set_color(traj, color_[0], color_[1], color_[2], color_[3]);

    for (const auto &p : pts_) {
      auto *v = traj.add_point();
      v->set_x(p[0]); v->set_y(p[1]); v->set_z(p[2]);
    }

    std::string err;
    bool ok = request_marker<GM::Empty>(node_, svc_, traj, 2000, err);
    if (!ok) {
      // Try Boolean if this Gazebo uses that reply type
      ok = request_marker<GM::Boolean>(node_, svc_, traj, 2000, err);
    }
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Failed to draw trajectory via %s: %s", svc_.c_str(), err.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Trajectory marker sent: %zu points", pts_.size());
    }

    // Optional waypoint points
    if (show_pts_) {
      GM::Marker pts;
      pts.set_id(1002);
      pts.set_action(GM::Marker::ADD_MODIFY);
      pts.set_type(GM::Marker::POINTS);
      pts.mutable_scale()->set_x(ps_);      // size for points
      set_color(pts, 0.2f, 1.0f, 0.2f, 1.0f);

      for (const auto &p : pts_) {
        auto *v = pts.add_point();
        v->set_x(p[0]); v->set_y(p[1]); v->set_z(p[2]);
      }

      // reuse same try-empty-then-boolean pattern
      std::string err2;
      bool ok2 = request_marker<GM::Empty>(node_, svc_, pts, 2000, err2);
      if (!ok2) ok2 = request_marker<GM::Boolean>(node_, svc_, pts, 2000, err2);

      if (!ok2)
        RCLCPP_WARN(get_logger(), "Failed to draw waypoint points: %s", err2.c_str());
      else
        RCLCPP_INFO(get_logger(), "Waypoint points sent: %zu points", pts_.size());
    }
  }

  // members
  gz::transport::Node node_;
  std::vector<std::array<double,3>> pts_;
  std::string csv_, svc_;
  double lw_{0.2}, ps_{0.3};
  std::vector<double> color_;
  bool show_pts_{true};
};

int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<TrajToMarkers>());
  rclcpp::shutdown();
  return 0;
}
